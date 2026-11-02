/* SPDX-License-Identifier: GPL-2.0-only */

#pragma once

#include "ns3/nstime.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/random-variable-stream.h"

#include <cstdint>
#include <map>
#include <utility>

namespace ns3
{
namespace dvcl
{

class DvClStaticShadowingPropagationLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();
    DvClStaticShadowingPropagationLossModel();
    ~DvClStaticShadowingPropagationLossModel() override;

    /// Shadowing applied to the link between two node ids, in dB.
    double GetLinkShadowingDb(uint32_t nodeA, uint32_t nodeB) const;

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

    double m_sigmaDb{3.57};
    Ptr<NormalRandomVariable> m_normal;
    /// Key is the ordered pair (min id, max id): shadowing is reciprocal, so
    /// both directions of a link must see the same value.
    mutable std::map<std::pair<uint32_t, uint32_t>, double> m_linkShadowing;
};

} // namespace dvcl
} // namespace ns3
