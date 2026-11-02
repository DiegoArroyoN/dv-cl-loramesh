/* SPDX-License-Identifier: GPL-2.0-only */

#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/dv-cl-app.h"
#include "ns3/dv-cl-helper.h"
#include "ns3/dv-cl-stats-sink.h"
#include "ns3/node-container.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"

#include <map>
#include <set>
#include <string>
#include <tuple>

using namespace ns3;
using namespace ns3::dvcl;

/**
 * \ingroup dv-cl
 * \brief Sink that timestamps what each node does, to check what a node
 * does after its battery is empty.
 */
class ShutdownSink : public DvClStatsSink
{
  public:
    using Key = std::tuple<uint32_t, uint32_t, uint32_t>;

    std::map<uint32_t, double> deathTime;      //!< first snapshot with an empty battery
    std::map<uint32_t, double> lastTx;         //!< last transmission per node
    std::map<uint32_t, double> lastRx;         //!< last reception per node
    std::map<uint32_t, double> lastGenerated;  //!< last data generation per node
    std::map<uint32_t, uint32_t> txCount;
    std::set<Key> generated;
    std::map<Key, double> deliveredAt;
    std::map<Key, std::string> terminated;
    uint32_t duplicateDeliveries{0};

    void RecordEnergySnapshot(uint32_t nodeId, double energyJ, double energyFrac) override
    {
        const bool empty =
            (energyFrac >= 0.0 && energyFrac <= 1e-9) || (energyJ >= 0.0 && energyJ <= 1e-9);
        if (empty && deathTime.find(nodeId) == deathTime.end())
        {
            deathTime[nodeId] = Simulator::Now().GetSeconds();
        }
    }

    void RecordTx(uint32_t nodeId, uint32_t, uint32_t, uint32_t, uint8_t, uint8_t, int16_t,
                  uint16_t, uint16_t, uint8_t, uint32_t, double, double, bool ok) override
    {
        if (ok)
        {
            lastTx[nodeId] = Simulator::Now().GetSeconds();
            ++txCount[nodeId];
        }
    }

    void RecordRx(uint32_t nodeId, uint32_t, uint32_t, uint32_t, uint8_t, uint8_t, int16_t,
                  uint16_t, uint16_t, uint8_t, double, double, bool) override
    {
        lastRx[nodeId] = Simulator::Now().GetSeconds();
    }

    void RecordDataGenerated(uint32_t src, uint32_t dst, uint32_t seq) override
    {
        generated.insert({src, dst, seq});
        lastGenerated[src] = Simulator::Now().GetSeconds();
    }

    void RecordE2eDelay(uint32_t src,
                        uint32_t dst,
                        uint32_t seq,
                        uint8_t,
                        double,
                        uint32_t,
                        uint8_t,
                        bool wasDelivered) override
    {
        if (wasDelivered &&
            !deliveredAt.emplace(Key{src, dst, seq}, Simulator::Now().GetSeconds()).second)
        {
            ++duplicateDeliveries;
        }
    }

    void RecordDataTerminated(uint32_t src,
                              uint32_t dst,
                              uint32_t seq,
                              const std::string& fate) override
    {
        terminated.emplace(Key{src, dst, seq}, fate);
    }

    void RecordRoute(uint32_t, uint32_t, uint32_t, uint8_t, uint16_t, uint32_t,
                     std::string) override {}
    void RecordRouteUsed(uint32_t, uint32_t, uint32_t, uint8_t, uint16_t, uint32_t) override {}
    void RecordQuantizationSample(uint32_t, uint32_t, uint32_t, bool, double, uint16_t) override {}
    void RecordRuntimeNodeStats(const DvClNodeStats&) override {}
    void RecordOverhead(uint32_t, const std::string&, uint32_t, uint32_t, uint32_t, uint32_t,
                        uint8_t, uint8_t) override {}
    void RecordDuty(uint32_t, double, uint32_t, uint32_t) override {}
    void RecordConnectivity(uint32_t, uint32_t, bool) override {}
    void RecordBeaconDelay(double) override {}
    double GetFirstTxTime(uint32_t, uint32_t, uint32_t) const override { return -1.0; }
};

namespace
{

/// Packets that appear both as delivered and as written off.
uint32_t
CountOverlaps(const Ptr<ShutdownSink>& sink)
{
    uint32_t n = 0;
    for (const auto& kv : sink->terminated)
    {
        n += (sink->deliveredAt.count(kv.first) != 0) ? 1 : 0;
    }
    return n;
}

/// Four nodes in a strong 2x2 grid, all-to-all traffic, no duty cycle. Node 0
/// starts with almost no charge, so it empties a few minutes into the run
/// while the others keep working.
Ptr<ShutdownSink>
RunScenario(bool shutdown, Ptr<DvClApp>* app0Out)
{
    Config::SetDefault("ns3::dvcl::DvClApp::TrafficMode", StringValue("pueyo_all_to_all"));
    Config::SetDefault("ns3::dvcl::DvClApp::PueyoPacketsPerPair", UintegerValue(1000));
    Config::SetDefault("ns3::dvcl::DvClApp::DataStartTimeSec", DoubleValue(60.0));
    Config::SetDefault("ns3::dvcl::DvClApp::DataStopTimeSec", DoubleValue(-1.0));

    DvClMeshConfig cfg;
    cfg.nEd = 4;
    cfg.simTimeSec = 1500.0;
    cfg.enableDutyCycle = false;
    cfg.enableCsma = true;
    cfg.nodePlacementMode = "grid";
    cfg.gridSide = 2;
    cfg.gridSpacingX = 100.0;
    cfg.gridSpacingY = 100.0;

    NodeContainer nodes;
    nodes.Create(cfg.nEd);
    auto helper = CreateObject<DvClHelper>();
    helper->SetConfig(cfg);
    helper->Install(nodes);

    auto sink = CreateObject<ShutdownSink>();
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        for (uint32_t a = 0; a < nodes.Get(i)->GetNApplications(); ++a)
        {
            if (auto app = DynamicCast<DvClApp>(nodes.Get(i)->GetApplication(a)))
            {
                app->SetStatsSink(sink);
                app->SetAttribute("ShutdownOnDepletion", BooleanValue(shutdown));
                if (i == 0)
                {
                    // 0.0005 x 300 mAh = 0.15 mAh: some 5 min of idle listening.
                    app->SetAttribute("InitialSocFraction", DoubleValue(0.0005));
                    *app0Out = app;
                }
            }
        }
    }

    Simulator::Stop(Seconds(cfg.simTimeSec));
    Simulator::Run();
    Simulator::Destroy();
    return sink;
}

} // namespace

/**
 * \ingroup dv-cl
 * \brief With ShutdownOnDepletion=true a node whose battery empties stops
 * generating, transmitting and receiving, and the rest of the network keeps
 * working.
 */
class DvClShutdownTestCase : public TestCase
{
  public:
    DvClShutdownTestCase()
        : TestCase("dv-cl shutdown: a depleted node falls silent, the others carry on")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<DvClApp> app0;
        auto sink = RunScenario(true, &app0);

        NS_TEST_ASSERT_MSG_EQ((sink->deathTime.count(0) == 1), true, "node 0 must deplete");
        const double t0 = sink->deathTime[0];
        NS_TEST_ASSERT_MSG_GT(t0, 60.0, "node 0 dies after the data starts");
        NS_TEST_ASSERT_MSG_LT(t0, 1000.0, "node 0 dies well before the end");
        NS_TEST_ASSERT_MSG_EQ(app0->IsShutDown(), true, "node 0 reports the shutdown");
        NS_TEST_ASSERT_MSG_EQ((sink->deathTime.count(1) + sink->deathTime.count(2) +
                               sink->deathTime.count(3)),
                              0u,
                              "the full nodes must not deplete");

        // A frame that started at the very instant of death is legitimate;
        // anything later is not.
        const double tol = 1e-6;
        NS_TEST_ASSERT_MSG_LT_OR_EQ(sink->lastTx[0], t0 + tol, "no transmission after death");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(sink->lastRx[0], t0 + tol, "no reception after death");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(sink->lastGenerated[0],
                                    t0 + tol,
                                    "no data generated after death");

        // The rest of the network keeps working.
        NS_TEST_ASSERT_MSG_GT(sink->lastTx[1], t0 + 300.0, "node 1 keeps transmitting");
        NS_TEST_ASSERT_MSG_GT(sink->lastGenerated[2], t0 + 300.0, "node 2 keeps generating");
        uint32_t deliveredLater = 0;
        uint32_t deliveredToDead = 0;
        for (const auto& kv : sink->deliveredAt)
        {
            if (kv.second > t0 + tol)
            {
                ++deliveredLater;
                if (std::get<1>(kv.first) == 0)
                {
                    ++deliveredToDead;
                }
            }
        }
        NS_TEST_ASSERT_MSG_GT(deliveredLater, 0u, "packets are still delivered after the death");
        NS_TEST_ASSERT_MSG_EQ(deliveredToDead, 0u, "nothing is delivered to a dead node");

        // Conservation still holds. One overlap is legitimate: the destination
        // accepts a frame it overhears even when the frame was addressed to a
        // relay (dv-cl-app.cc, "solo el siguiente salto designado o el
        // destino"), so the copy that the dead relay still held in its queue is
        // written off although the packet already arrived. The same happens when
        // a full queue evicts such a copy. It can only concern packets of OTHER
        // nodes: node 0's own queued packets never went on the air.
        NS_TEST_ASSERT_MSG_EQ(sink->duplicateDeliveries, 0u, "a packet is delivered at most once");
        uint32_t bothOther = 0;
        uint32_t relayCopies = 0;
        uint32_t ownBoth = 0;
        uint32_t unknown = 0;
        uint32_t depleted = 0;
        for (const auto& kv : sink->terminated)
        {
            const bool delivered = sink->deliveredAt.count(kv.first) != 0;
            const bool byShutdown = (kv.second == "node_depleted");
            unknown += (sink->generated.count(kv.first) == 0) ? 1 : 0;
            depleted += byShutdown ? 1 : 0;
            if (!delivered)
            {
                continue;
            }
            if (!byShutdown)
            {
                ++bothOther;
            }
            else if (std::get<0>(kv.first) == 0)
            {
                ++ownBoth;
            }
            else
            {
                ++relayCopies;
            }
        }
        std::map<std::string, uint32_t> overlapFates;
        for (const auto& kv : sink->terminated)
        {
            if (sink->deliveredAt.count(kv.first) != 0)
            {
                ++overlapFates[kv.second + (std::get<0>(kv.first) == 0 ? "/propio" : "/ajeno")];
            }
        }
        std::clog << "  [shutdown] superposiciones entrega-descarte:";
        for (const auto& f : overlapFates)
        {
            std::clog << " " << f.first << "=" << f.second;
        }
        std::clog << std::endl;
        NS_TEST_ASSERT_MSG_EQ(bothOther + relayCopies + ownBoth,
                              CountOverlaps(sink),
                              "overlap bookkeeping is consistent");
        NS_TEST_ASSERT_MSG_EQ(ownBoth,
                              0u,
                              "a packet the dead node never sent cannot have been delivered");
        NS_TEST_ASSERT_MSG_EQ(unknown, 0u, "nothing is dropped that was never generated");
        std::clog << "  [shutdown] t_muerte=" << t0 << " s tx_nodo0=" << sink->txCount[0]
                  << " entregados_despues=" << deliveredLater
                  << " descartados_por_apagado=" << depleted
                  << " (copias_de_relevo_ya_entregadas=" << relayCopies << ")" << std::endl;
    }
};

/**
 * \ingroup dv-cl
 * \brief Without the attribute nothing changes: the depleted node keeps
 * transmitting, which is what the attribute exists to stop. Guards against
 * the scenario above passing for the wrong reason.
 */
class DvClNoShutdownTestCase : public TestCase
{
  public:
    DvClNoShutdownTestCase()
        : TestCase("dv-cl shutdown: by default a depleted node keeps operating")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<DvClApp> app0;
        auto sink = RunScenario(false, &app0);

        NS_TEST_ASSERT_MSG_EQ((sink->deathTime.count(0) == 1), true, "node 0 must deplete");
        const double t0 = sink->deathTime[0];
        NS_TEST_ASSERT_MSG_EQ(app0->IsShutDown(), false, "the default never shuts down");
        NS_TEST_ASSERT_MSG_GT(sink->lastTx[0], t0 + 300.0, "the depleted node keeps transmitting");
        NS_TEST_ASSERT_MSG_GT(sink->lastGenerated[0], t0 + 300.0, "and keeps generating");
        std::clog << "  [sin apagado] t_muerte=" << t0 << " s tx_nodo0=" << sink->txCount[0]
                  << " superposiciones entrega-descarte=" << CountOverlaps(sink) << std::endl;
    }
};

class DvClShutdownTestSuite : public TestSuite
{
  public:
    DvClShutdownTestSuite()
        : TestSuite("dv-cl-shutdown", Type::SYSTEM)
    {
        AddTestCase(new DvClShutdownTestCase, TestCase::Duration::QUICK);
        AddTestCase(new DvClNoShutdownTestCase, TestCase::Duration::QUICK);
    }
};

static DvClShutdownTestSuite g_dvClShutdownTestSuite; //!< Static instance
