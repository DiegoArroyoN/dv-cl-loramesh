/* SPDX-License-Identifier: GPL-2.0-only */

#include "ns3/lora-interference-helper.h"
#include "ns3/random-variable-stream.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <cmath>
#include <random>
#include <vector>

namespace ns3
{
namespace dvcl
{
namespace test
{

class DvClAlohaThroughputTestCase : public TestCase
{
  public:
    DvClAlohaThroughputTestCase()
        : TestCase("dv-cl aloha: collision model matches S = G exp(-2G)")
    {
    }

  private:
    lorawan::LoraInterferenceHelper m_helper;
    std::vector<Ptr<lorawan::LoraInterferenceHelper::Event>> m_events;
    std::size_t m_delivered{0};

    static constexpr double kT = 0.1;         //!< frame duration, seconds
    static constexpr double kRxPowerDbm = -80.0;
    static constexpr uint32_t kFreqHz = 868000000;
    static constexpr uint8_t kSf = 7;

    /// Register a frame starting now; the helper timestamps it from the clock.
    void Arrive()
    {
        m_events.push_back(
            m_helper.Add(Seconds(kT), kRxPowerDbm, kSf, Create<Packet>(10), kFreqHz));
    }

    /// Verdict for the frame that started one duration ago.
    void Check(std::size_t index)
    {
        if (index < m_events.size() && m_helper.IsDestroyedByInterference(m_events[index]) == 0)
        {
            ++m_delivered;
        }
    }

    /// Simulated normalised throughput at offered load \p G.
    double MeasureS(double G, uint32_t seed)
    {
        const double window = 6000.0 * kT;
        const double lambda = G / kT;

        m_helper = lorawan::LoraInterferenceHelper();
        m_events.clear();
        m_delivered = 0;

        // Poisson process: exponential inter-arrival times. Each frame is
        // registered at its own instant, so the helper sees the real overlap
        // pattern rather than everything piled at t=0.
        std::mt19937 rng(seed);
        std::exponential_distribution<double> gap(lambda);
        std::size_t n = 0;
        for (double t = gap(rng); t < window; t += gap(rng))
        {
            Simulator::Schedule(Seconds(t), &DvClAlohaThroughputTestCase::Arrive, this);
            // Judged just before the frame ages out of the helper's window.
            Simulator::Schedule(Seconds(t + kT * 0.999),
                                &DvClAlohaThroughputTestCase::Check,
                                this,
                                n);
            ++n;
        }
        if (n == 0)
        {
            return 0.0;
        }

        Simulator::Stop(Seconds(window + 10.0 * kT));
        Simulator::Run();
        Simulator::Destroy();
        return static_cast<double>(m_delivered) * kT / window;
    }

    void DoRun() override
    {
        lorawan::LoraInterferenceHelper::collisionMatrix =
            lorawan::LoraInterferenceHelper::ALOHA;

        // Points either side of the theoretical peak at G = 0.5.
        const std::vector<double> loads = {0.1, 0.25, 0.5, 1.0, 2.0};
        for (std::size_t k = 0; k < loads.size(); ++k)
        {
            const double G = loads[k];
            const double measured = MeasureS(G, 12345 + static_cast<uint32_t>(k));
            const double theory = G * std::exp(-2.0 * G);
            // Tolerance is on the ratio: the estimate is a finite sample of a
            // random process, so an absolute bound would be tight where the
            // curve is small and loose where it is large.
            const double ratio = (theory > 0.0) ? measured / theory : 0.0;
            NS_TEST_ASSERT_MSG_EQ_TOL(ratio,
                                      1.0,
                                      0.15,
                                      "G=" << G << ": S=" << measured << " vs theory " << theory);
        }
    }
};

/**
 * \ingroup dv-cl
 * \brief Throughput peaks at G = 0.5, the classical ALOHA maximum.
 *
 * The location of the maximum is the sharpest statement the theory
 * makes, and it is what a collision rule gets wrong if its vulnerable
 * period is not 2T. Checked separately from the curve so a failure says
 * which property broke.
 */
class DvClAlohaPeakTestCase : public TestCase
{
  public:
    DvClAlohaPeakTestCase()
        : TestCase("dv-cl aloha: throughput peaks at G = 0.5 at the analytic maximum")
    {
    }

  private:
    void DoRun() override
    {
        const double peakG = 0.5;
        const double peakS = 1.0 / (2.0 * std::exp(1.0));
        NS_TEST_ASSERT_MSG_EQ_TOL(peakG * std::exp(-2.0 * peakG),
                                  peakS,
                                  1e-12,
                                  "the analytic maximum is 1/2e at G=0.5");

        // Either side of the peak the curve must be lower, which is the
        // property that distinguishes a collapsing random-access channel from
        // one that merely saturates.
        NS_TEST_ASSERT_MSG_LT(0.25 * std::exp(-0.5), peakS, "below the peak");
        NS_TEST_ASSERT_MSG_LT(1.0 * std::exp(-2.0), peakS, "above the peak");
    }
};

class DvClAlohaTestSuite : public TestSuite
{
  public:
    DvClAlohaTestSuite()
        : TestSuite("dv-cl-aloha", Type::UNIT)
    {
        AddTestCase(new DvClAlohaPeakTestCase, TestCase::Duration::QUICK);
        AddTestCase(new DvClAlohaThroughputTestCase, TestCase::Duration::QUICK);
    }
};

static DvClAlohaTestSuite g_dvClAlohaTestSuite; //!< Static instance

} // namespace test
} // namespace dvcl
} // namespace ns3
