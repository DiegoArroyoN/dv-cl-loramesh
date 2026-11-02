/* SPDX-License-Identifier: GPL-2.0-only */

#include "ns3/dv-cl-routing.h"
#include "ns3/simulator.h"
#include "ns3/double.h"
#include "ns3/test.h"

#include <algorithm>
#include <limits>
#include <map>
#include <vector>

namespace ns3
{
namespace dvcl
{
namespace test
{

/**
 * \ingroup dv-cl
 * \brief A synthetic network of DvClRouting instances driven like the protocol.
 *
 * Each node holds a real routing table; a round hands every node the
 * announcements of each neighbour (GetBestRoutes, the same call the
 * application makes) wrapped in a DvMessage, exactly as a received beacon
 * would. Nothing about cost composition is reimplemented here, so what the
 * test observes is the protocol's own convergence rather than a model of it.
 */
class DvNetwork
{
  public:
    /// \param linkToaUs adjacency: linkToaUs[i][j] > 0 means i and j are neighbours
    DvNetwork(std::size_t n,
              const std::vector<std::vector<uint32_t>>& linkToaUs,
              const std::vector<uint16_t>& battMv = {})
        : m_n(n),
          m_toa(linkToaUs),
          m_batt(battMv.empty() ? std::vector<uint16_t>(n, 4200) : battMv)
    {
        for (std::size_t i = 0; i < n; ++i)
        {
            auto r = CreateObject<DvClRouting>();
            // Each instance must know which node it is, as the application sets
            // it at startup; left at its default every node believes it is node
            // 0 and refuses routes to that destination as if to itself.
            r->SetNodeId(static_cast<NodeId>(i));
            m_routing.push_back(r);
        }
    }

    /// One exchange: every node hears one beacon from each of its neighbours.
    void Round(uint32_t seq)
    {
        // Snapshot first: within a round every node advertises the table it had
        // at the start, so the outcome does not depend on iteration order.
        std::vector<std::vector<RouteAnnouncement>> adverts(m_n);
        for (std::size_t i = 0; i < m_n; ++i)
        {
            adverts[i] = m_routing[i]->GetBestRoutes(64);
        }

        for (std::size_t rx = 0; rx < m_n; ++rx)
        {
            for (std::size_t tx = 0; tx < m_n; ++tx)
            {
                const uint32_t toa = m_toa[rx][tx];
                if (rx == tx || toa == 0)
                {
                    continue;
                }
                NeighborLinkInfo link;
                link.neighbor = static_cast<NodeId>(tx);
                link.sequence = seq;
                link.hops = 1;
                link.sf = 7;
                link.toaUs = toa;
                link.batt_mV = m_batt[tx]; // the advertising neighbour's charge
                link.scoreX100 = 0;

                DvMessage msg;
                msg.origin = static_cast<NodeId>(tx);
                msg.sequence = seq;
                for (const auto& a : adverts[tx])
                {
                    if (a.destination == static_cast<NodeId>(rx))
                    {
                        continue; // a node does not learn a route to itself
                    }
                    DvEntry e;
                    e.destination = a.destination;
                    // Campo 2 de la entrada de 6 B, como en el aire.
                    e.nextHop = a.nextHop;
                    e.seqno = a.seqno;
                    e.hops = a.hops;
                    e.sf = a.sf;
                    e.scoreX100 = a.scoreX100;
                    e.toaUs = 0;
                    e.batt_mV = a.batt_mV;
                    msg.entries.push_back(e);
                }
                m_routing[rx]->UpdateFromDvMsg(msg, link);
            }
        }
    }

    Ptr<DvClRouting> At(std::size_t i) const { return m_routing[i]; }

  private:
    std::size_t m_n;
    std::vector<std::vector<uint32_t>> m_toa;
    std::vector<uint16_t> m_batt;
    std::vector<Ptr<DvClRouting>> m_routing;
};

/**
 * \brief Hop-count shortest paths, computed independently of the module.
 *
 * The reference is deliberately naive Bellman-Ford relaxation over the
 * adjacency, sharing no code with the routing under test.
 */
static std::vector<std::vector<int>>
BellmanFordHops(std::size_t n, const std::vector<std::vector<uint32_t>>& adj)
{
    const int kInf = std::numeric_limits<int>::max() / 4;
    std::vector<std::vector<int>> d(n, std::vector<int>(n, kInf));
    for (std::size_t i = 0; i < n; ++i)
    {
        d[i][i] = 0;
        for (std::size_t j = 0; j < n; ++j)
        {
            if (i != j && adj[i][j] > 0)
            {
                d[i][j] = 1;
            }
        }
    }
    for (std::size_t it = 0; it < n; ++it)
    {
        for (std::size_t i = 0; i < n; ++i)
        {
            for (std::size_t k = 0; k < n; ++k)
            {
                for (std::size_t j = 0; j < n; ++j)
                {
                    if (d[i][k] + d[k][j] < d[i][j])
                    {
                        d[i][j] = d[i][k] + d[k][j];
                    }
                }
            }
        }
    }
    return d;
}

class DvClConvergenceTestCase : public TestCase
{
  public:
    DvClConvergenceTestCase()
        : TestCase("dv-cl convergence: routes are complete, loop-free and hop-optimal")
    {
    }

  private:
    /// Walk next hops from `src` to `dst`; -1 if broken or looping.
    int WalkHops(const DvNetwork& net, std::size_t n, NodeId src, NodeId dst)
    {
        std::vector<bool> seen(n, false);
        NodeId cur = src;
        int hops = 0;
        while (cur != dst)
        {
            if (seen[cur] || hops > static_cast<int>(n))
            {
                return -1; // loop
            }
            seen[cur] = true;
            const RouteEntry* r = net.At(cur)->GetRoute(dst);
            if (!r)
            {
                return -1; // no route
            }
            cur = r->nextHop;
            ++hops;
        }
        return hops;
    }

    void DoRun() override
    {
        // 3x3 grid, four-neighbourhood: diameter 4, several ties, and paths
        // that only appear after multiple exchanges.
        //   0 1 2
        //   3 4 5
        //   6 7 8
        const std::size_t n = 9;
        const uint32_t kToa = 102656;
        std::vector<std::vector<uint32_t>> adj(n, std::vector<uint32_t>(n, 0));
        auto connect = [&](std::size_t a, std::size_t b) {
            adj[a][b] = kToa;
            adj[b][a] = kToa;
        };
        for (std::size_t r = 0; r < 3; ++r)
        {
            for (std::size_t c = 0; c < 3; ++c)
            {
                const std::size_t id = r * 3 + c;
                if (c + 1 < 3)
                {
                    connect(id, id + 1);
                }
                if (r + 1 < 3)
                {
                    connect(id, id + 3);
                }
            }
        }

        DvNetwork net(n, adj);
        // Diameter is 4, so five exchanges settle the table; a few more confirm
        // the result is stable rather than caught mid-flight.
        for (uint32_t round = 1; round <= 8; ++round)
        {
            net.Round(round);
        }

        const auto ref = BellmanFordHops(n, adj);
        std::size_t missing = 0;
        std::size_t looping = 0;
        std::size_t suboptimal = 0;
        std::size_t checked = 0;

        for (NodeId s = 0; s < n; ++s)
        {
            for (NodeId d = 0; d < n; ++d)
            {
                if (s == d)
                {
                    continue;
                }
                ++checked;
                const int walked = WalkHops(net, n, s, d);
                if (walked < 0)
                {
                    if (net.At(s)->GetRoute(d))
                    {
                        ++looping;
                    }
                    else
                    {
                        ++missing;
                    }
                    continue;
                }
                if (walked > ref[s][d])
                {
                    ++suboptimal;
                }
            }
        }

        NS_TEST_ASSERT_MSG_EQ(checked, n * (n - 1), "every ordered pair examined");
        NS_TEST_ASSERT_MSG_EQ(missing, 0u, "connected pairs must have a route");
        NS_TEST_ASSERT_MSG_EQ(looping, 0u, "next hops must not form a loop");
        NS_TEST_ASSERT_MSG_EQ(suboptimal, 0u, "paths must match Bellman-Ford hop counts");

        Simulator::Destroy();
    }
};

/**
 * \ingroup dv-cl
 * \brief A cheap detour wins over an expensive direct link.
 *
 * Hop count alone would take the direct link; the composite metric prices
 * time on air, so a two-hop path over fast links must beat one slow hop.
 * This is the property that makes the metric cross-layer, and it is what
 * distinguishes the routing from a plain hop-count DV.
 */
class DvClConvergenceCostAwareTestCase : public TestCase
{
  public:
    DvClConvergenceCostAwareTestCase()
        : TestCase("dv-cl convergence: a cheap two-hop path beats a costly direct link")
    {
    }

  private:
    void DoRun() override
    {
        // 0 --(slow SF12-scale link)-- 2
        // 0 --(fast)-- 1 --(fast)-- 2
        const std::size_t n = 3;
        const uint32_t kFast = 10000;   // short air time
        const uint32_t kSlow = 1908736; // SF12-scale: far more expensive
        std::vector<std::vector<uint32_t>> adj(n, std::vector<uint32_t>(n, 0));
        adj[0][1] = adj[1][0] = kFast;
        adj[1][2] = adj[2][1] = kFast;
        adj[0][2] = adj[2][0] = kSlow;

        DvNetwork net(n, adj);
        for (std::size_t i = 0; i < n; ++i)
        {
            auto* cm = dynamic_cast<DvClCompositeMetric*>(PeekPointer(net.At(i)->GetMetric()));
            if (cm)
            {
                cm->SetAttribute("WHop", DoubleValue(0.05));
            }
            net.At(i)->SetAttribute("CompositeCostStep", DoubleValue(0.005));
        }
        for (uint32_t round = 1; round <= 6; ++round)
        {
            net.Round(round);
        }

        const RouteEntry* r = net.At(0)->GetRoute(2);
        NS_TEST_ASSERT_MSG_NE(r, nullptr, "node 0 must have a route to 2");
        NS_TEST_ASSERT_MSG_EQ(r->nextHop,
                              1u,
                              "the fast detour must win over the slow direct link");

        Simulator::Destroy();
    }
};

class DvClEnergyPreferenceTestCase : public TestCase
{
  public:
    DvClEnergyPreferenceTestCase()
        : TestCase("dv-cl energy: the fuller relay wins an otherwise equal tie")
    {
    }

  private:
    void DoRun() override
    {
        const std::size_t n = 4;
        const uint32_t kToa = 102656;
        std::vector<std::vector<uint32_t>> adj(n, std::vector<uint32_t>(n, 0));
        auto connect = [&](std::size_t a, std::size_t b) { adj[a][b] = adj[b][a] = kToa; };
        connect(0, 1);
        connect(0, 2);
        connect(1, 3);
        connect(2, 3);

        // 3000 mV is empty and 4200 full on the scale the metric uses, so node 1
        // sits at Psi = 1 and node 2 at Psi = 0 -- the largest possible gap.
        std::vector<uint16_t> batt = {4200, 3000, 4200, 4200};

        DvNetwork net(n, adj, batt);
        for (uint32_t round = 1; round <= 8; ++round)
        {
            net.Round(round);
        }

        const RouteEntry* r = net.At(0)->GetRoute(3);
        NS_TEST_ASSERT_MSG_NE(r, nullptr, "node 0 must find node 3");
        NS_TEST_ASSERT_MSG_EQ(r->nextHop, 2u, "the charged relay must be preferred");
        Simulator::Destroy();

        // Control: swap the charges and the choice must swap with them. Without
        // this the test would also pass if the routing simply favoured the
        // higher node id, which happens to be the answer above.
        std::vector<uint16_t> swapped = {4200, 4200, 3000, 4200};
        DvNetwork mirror(n, adj, swapped);
        for (uint32_t round = 1; round <= 8; ++round)
        {
            mirror.Round(round);
        }
        const RouteEntry* r2 = mirror.At(0)->GetRoute(3);
        NS_TEST_ASSERT_MSG_NE(r2, nullptr, "node 0 must find node 3 in the mirror");
        NS_TEST_ASSERT_MSG_EQ(r2->nextHop, 1u, "the choice follows the charge, not the id");

        Simulator::Destroy();
    }
};

class DvClConvergenceTestSuite : public TestSuite
{
  public:
    DvClConvergenceTestSuite()
        : TestSuite("dv-cl-convergence", Type::UNIT)
    {
        AddTestCase(new DvClConvergenceTestCase, TestCase::Duration::QUICK);
        AddTestCase(new DvClConvergenceCostAwareTestCase, TestCase::Duration::QUICK);
        AddTestCase(new DvClEnergyPreferenceTestCase, TestCase::Duration::QUICK);
    }
};

static DvClConvergenceTestSuite g_dvClConvergenceTestSuite; //!< Static instance

} // namespace test
} // namespace dvcl
} // namespace ns3
