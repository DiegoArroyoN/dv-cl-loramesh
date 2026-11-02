/* SPDX-License-Identifier: GPL-2.0-only */

#include "ns3/basic-energy-source.h"
#include "ns3/double.h"
#include "ns3/dv-cl-app.h"
#include "ns3/dv-cl-lora-energy-model.h"
#include "ns3/node.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

namespace ns3
{
namespace dvcl
{
namespace test
{

class DvClEnergyLedgerTestCase : public TestCase
{
  public:
    DvClEnergyLedgerTestCase()
        : TestCase("dv-cl energy ledger closes and E = sum(I*V*t)")
    {
    }

  private:
    Ptr<DvClLoraEnergyModel> m_model;

    void Check()
    {
        // Charged: TX 2 s @120 mA, RX 1 s @10.3 mA, CAD 0.5 s @10.3 mA, plus
        // idle for the whole 10 s window at 1.6 mA (folded in lazily).
        const double v = m_model->GetSupplyVoltageV();
        const double txMah = m_model->GetTxMah();
        const double rxMah = m_model->GetRxMah();
        const double cadMah = m_model->GetCadMah();
        const double idleMah = m_model->GetIdleMah();

        NS_TEST_ASSERT_MSG_EQ_TOL(m_model->GetTimeInState(DvClRadioState::TX).GetSeconds(),
                                  2.0, 1e-9, "TX time recorded");
        NS_TEST_ASSERT_MSG_EQ_TOL(m_model->GetTimeInState(DvClRadioState::RX).GetSeconds(),
                                  1.0, 1e-9, "RX time recorded");

        const double spentMah = txMah + rxMah + cadMah + idleMah;
        const double remainingMah = m_model->GetEnergyFraction() * 300.0;
        NS_TEST_ASSERT_MSG_EQ_TOL(spentMah + remainingMah, 300.0, 1e-9,
                                  "spent-per-activity + remaining = capacity");

        // E = V * sum(I_s * t_s). Idle spans the 10 s the events sit inside.
        const double expectedJ =
            v * (0.120 * 2.0 + 0.0103 * 1.0 + 0.0103 * 0.5 + 0.0016 * 10.0);
        NS_TEST_ASSERT_MSG_EQ_TOL(m_model->GetTotalEnergyConsumption(), expectedJ, 1e-6,
                                  "energy closure E = sum(I_s*V*t_s)");
    }

    void DoRun() override
    {
        m_model = CreateObject<DvClLoraEnergyModel>();
        m_model->SetCapacityMah(300.0);
        m_model->SetInitialSocFraction(1.0);

        Simulator::Schedule(Seconds(1), [this]() { m_model->ChargeTx(2.0); });
        Simulator::Schedule(Seconds(4), [this]() { m_model->ChargeRx(1.0); });
        Simulator::Schedule(Seconds(6), [this]() { m_model->ChargeCad(0.5); });
        Simulator::Schedule(Seconds(10), &DvClEnergyLedgerTestCase::Check, this);

        Simulator::Stop(Seconds(11));
        Simulator::Run();
        Simulator::Destroy();
        m_model = nullptr;
    }
};

class DvClEnergyDepletionTestCase : public TestCase
{
  public:
    DvClEnergyDepletionTestCase()
        : TestCase("dv-cl energy: a flat battery draws nothing and depletion fires once")
    {
    }

  private:
    uint32_t m_depletions{0};
    double m_spentAtDeath{0.0};
    Ptr<DvClLoraEnergyModel> m_model;

    void OnDepleted(uint32_t, double)
    {
        ++m_depletions;
    }

    void DrainHard()
    {
        m_model->ChargeTx(3600.0); // 120 mA for 1 h = 120 mAh, far past a 1 mAh cell
    }

    void CheckDead()
    {
        NS_TEST_ASSERT_MSG_EQ_TOL(m_model->GetEnergyFraction(), 0.0, 1e-12, "battery flat");
        m_spentAtDeath = m_model->GetTxMah() + m_model->GetRxMah() + m_model->GetCadMah() +
                         m_model->GetIdleMah();
        NS_TEST_ASSERT_MSG_EQ_TOL(m_spentAtDeath, 1.0, 1e-9, "charged no more than the 1 mAh there was");
    }

    void CheckStaysDead()
    {
        m_model->ChargeRx(10.0);
        const double later = m_model->GetTxMah() + m_model->GetRxMah() + m_model->GetCadMah() +
                             m_model->GetIdleMah();
        NS_TEST_ASSERT_MSG_EQ_TOL(later, m_spentAtDeath, 1e-9, "a dead node keeps drawing nothing");
        NS_TEST_ASSERT_MSG_EQ(m_depletions, 1u, "depletion trace fired exactly once");
    }

    void DoRun() override
    {
        m_model = CreateObject<DvClLoraEnergyModel>();
        m_model->SetCapacityMah(1.0); // tiny on purpose: drains inside the test
        m_model->SetInitialSocFraction(1.0);
        m_model->TraceConnectWithoutContext(
            "EnergyDepleted", MakeCallback(&DvClEnergyDepletionTestCase::OnDepleted, this));

        Simulator::Schedule(Seconds(10), &DvClEnergyDepletionTestCase::DrainHard, this);
        Simulator::Schedule(Seconds(20), &DvClEnergyDepletionTestCase::CheckDead, this);
        Simulator::Schedule(Seconds(100000), &DvClEnergyDepletionTestCase::CheckStaysDead, this);

        Simulator::Stop(Seconds(100001));
        Simulator::Run();
        Simulator::Destroy();
        m_model = nullptr;
    }
};

/**
 * \ingroup dv-cl
 * \brief It is a real ns-3 DeviceEnergyModel: attaches to a source and takes
 *        the supply voltage from it.
 */
class DvClEnergyFrameworkTestCase : public TestCase
{
  public:
    DvClEnergyFrameworkTestCase()
        : TestCase("dv-cl energy: attaches to an EnergySource and uses its supply voltage")
    {
    }

  private:
    void DoRun() override
    {
        auto src = CreateObject<energy::BasicEnergySource>();
        src->SetAttribute("BasicEnergySourceInitialEnergyJ", DoubleValue(1000.0));
        src->SetAttribute("BasicEnergySupplyVoltageV", DoubleValue(3.0));
        auto model = CreateObject<DvClLoraEnergyModel>();
        model->SetEnergySource(src);

        NS_TEST_ASSERT_MSG_EQ_TOL(model->GetSupplyVoltageV(), 3.0, 1e-9,
                                  "supply voltage taken from the attached source");
        // Without a source it falls back to the SoC-window midpoint (3.6 V).
        auto standalone = CreateObject<DvClLoraEnergyModel>();
        NS_TEST_ASSERT_MSG_EQ_TOL(standalone->GetSupplyVoltageV(), 3.6, 1e-9,
                                  "standalone falls back to the window midpoint");

        TypeId tid = DvClLoraEnergyModel::GetTypeId();
        NS_TEST_ASSERT_MSG_EQ((tid.LookupTraceSourceByName("EnergyDepleted") != nullptr), true,
                              "EnergyDepleted trace source registered");
    }
};

class DvClEnergyIdleCurrentTestCase : public TestCase
{
  public:
    DvClEnergyIdleCurrentTestCase()
        : TestCase("dv-cl energy: IdleCurrentMa on the app sets the idle draw of the ledger")
    {
    }

  private:
    Ptr<DvClLoraEnergyModel> m_model;

    void Check()
    {
        // 100 s of idle at 10.3 mA and nothing else.
        NS_TEST_ASSERT_MSG_EQ_TOL(m_model->GetIdleMah(), 10.3 * 100.0 / 3600.0, 1e-9,
                                  "idle charged at the configured current");
    }

    void DoRun() override
    {
        // The app needs a node: disposing it runs StopApplication, which logs
        // through GetNode().
        auto node = CreateObject<Node>();
        auto app = CreateObject<DvClApp>();
        node->AddApplication(app);
        DoubleValue v;
        app->GetAttribute("IdleCurrentMa", v);
        NS_TEST_ASSERT_MSG_EQ_TOL(v.Get(), DvClLoraEnergyModel::kDefaultIdleCurrentMa, 1e-12,
                                  "the default stays at standby");
        app->SetAttribute("IdleCurrentMa", DoubleValue(10.3));
        app->GetAttribute("IdleCurrentMa", v);
        NS_TEST_ASSERT_MSG_EQ_TOL(v.Get(), 10.3, 1e-12, "the app forwards the value to its ledger");

        m_model = CreateObject<DvClLoraEnergyModel>();
        m_model->SetIdleCurrentMa(10.3);
        m_model->SetInitialSocFraction(1.0);
        Simulator::Schedule(Seconds(100), &DvClEnergyIdleCurrentTestCase::Check, this);
        Simulator::Stop(Seconds(101));
        Simulator::Run();
        Simulator::Destroy();
        m_model = nullptr;
    }
};

/**
 * \ingroup dv-cl
 * The dv-cl-energy test suite.
 */
class DvClEnergyTestSuite : public TestSuite
{
  public:
    DvClEnergyTestSuite()
        : TestSuite("dv-cl-energy", Type::UNIT)
    {
        AddTestCase(new DvClEnergyLedgerTestCase, TestCase::Duration::QUICK);
        AddTestCase(new DvClEnergyDepletionTestCase, TestCase::Duration::QUICK);
        AddTestCase(new DvClEnergyFrameworkTestCase, TestCase::Duration::QUICK);
        AddTestCase(new DvClEnergyIdleCurrentTestCase, TestCase::Duration::QUICK);
    }
};

static DvClEnergyTestSuite g_dvClEnergyTestSuite; //!< static suite registration

} // namespace test
} // namespace dvcl
} // namespace ns3
