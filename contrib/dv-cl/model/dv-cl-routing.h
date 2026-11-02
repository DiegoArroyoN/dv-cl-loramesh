/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef DV_CL_ROUTING_H
#define DV_CL_ROUTING_H

#include "dv-cl-metric.h"

#include "ns3/callback.h"
#include "ns3/mac48-address.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/random-variable-stream.h"

#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ns3
{

using NodeId = uint32_t;

namespace dvcl
{

struct RouteEntry
{
    NodeId destination{0};
    NodeId nextHop{0};
    uint32_t seqNum{0};
    uint8_t hops{0};
    uint8_t sf{0};
    uint32_t toaUs{0};
    uint16_t batt_mV{0};
    uint16_t scoreX100{0};
    uint16_t costX1000{1000};
    uint32_t toaCostUnits{0};
    double rawMetric{0.0};
    Time lastUpdate{Seconds(0)};
    Time expiryTime{Seconds(0)};
    Mac48Address nextHopMac;
    uint8_t nextHopDcRemaining{0xFF};
};

struct RouteAnnouncement
{
    NodeId destination{0};
    NodeId nextHop{0};
    uint16_t seqno{0}; // seqno del destino (modo FEASIBILITY)
    uint8_t hops{0};
    uint8_t sf{0};
    uint16_t scoreX100{0};
    uint16_t batt_mV{0};
};

inline constexpr uint32_t kDvBeaconEntryBytes = 6;

enum class LoopPrevention : uint8_t
{
    LEGACY = 0,
    FEASIBILITY = 1
};

struct DvEntry
{
    NodeId destination{0};
    uint16_t seqno{0};
    NodeId nextHop{0};
    uint8_t hops{0};
    uint8_t sf{0};
    uint16_t scoreX100{0};
    uint32_t toaUs{0};
    uint16_t batt_mV{0};
};

struct DvMessage
{
    NodeId origin{0};
    uint32_t sequence{0};
    std::vector<DvEntry> entries;
};

struct NeighborLinkInfo
{
    NodeId neighbor{0};
    uint32_t sequence{0};
    uint8_t hops{0};
    uint8_t sf{0};
    uint32_t toaUs{0};
    double rssiDbm{0.0};
    uint16_t batt_mV{0};
    uint16_t scoreX100{0};
    Mac48Address mac;
    uint8_t dc_remaining{0xFF};
};

class DvClRouting : public Object
{
  public:
    using RouteChangeCallback = Callback<void, const RouteEntry&, const std::string&>;
    using FloodCallback = Callback<void, const DvMessage&>;
    using LocalEnergyFractionCallback = Callback<double>;

    enum class AdvertRoutePolicy
    {
        TOP_SCORE,
        UNIFORM,
        COST_WEIGHTED
    };

    enum class MetricMode
    {
        COMPOSITE_SCORE,
        TOA_ONLY,
        RSSI
    };

    enum class CostEncoding
    {
        SCORE100,
        COST255,
        SCORE255
    };

    static TypeId GetTypeId();

    DvClRouting();
    ~DvClRouting() override = default;

    void SetNodeId(NodeId id);
    void SetRouteTimeout(Time timeout);
    void SetRouteSwitchMinDeltaX100(uint16_t deltaX100);
    void SetInitTtl(uint8_t ttl);
    void SetMaxRoutes(uint32_t maxRoutes);
    void SetSequence(uint32_t seq);
    void SetExpireWindow(Time expire);
    void SetMaxHops(uint8_t maxHops);
    void SetLoopPrevention(LoopPrevention lp);

    static bool SeqnoNewer(uint16_t a, uint16_t b)
    {
        const uint16_t d = static_cast<uint16_t>(a - b);
        return d != 0 && d < 0x8000;
    }
    void SetAdvertiseAllRoutes(bool enable);
    bool GetAdvertiseAllRoutes() const;
    void SetAdvertRoutePolicy(const std::string& policy);
    std::string GetAdvertRoutePolicy() const;
    void SetMetricMode(const std::string& mode);
    std::string GetMetricMode() const;
    void SetCostEncoding(const std::string& mode);
    std::string GetCostEncoding() const;
    void SetMaxRoutesPerDestination(uint32_t maxRoutesPerDestination);
    void SetMaxTotalRoutes(uint32_t maxTotalRoutes);
    void SetSinkNodeId(NodeId id);
    void SetUseDcAwareRouting(bool enable);
    void SetDcFeasibilityThreshold(uint8_t thresholdPct);
    void NotifyDestinationActive(NodeId dest);
    bool IsDestinationActive(NodeId dest) const;
    bool HasAnyRoute(NodeId dest) const;
    bool IsRouteExpired(NodeId dest) const;
    void SetRouteChangeCallback(RouteChangeCallback cb);
    void SetFloodCallback(FloodCallback cb);
    void SetLocalEnergyFractionCallback(LocalEnergyFractionCallback cb);
    void SetMetric(Ptr<DvClRoutingMetric> metric);
    Ptr<DvClRoutingMetric> GetMetric() const;

    bool HasRoute(NodeId dest);              // Non-const: llama GetRoute()
    NodeId LookupNextHop(NodeId dest);       // Non-const: llama GetRoute()
    double GetRouteCost(NodeId dest);        // Non-const: llama GetRoute()
    const RouteEntry* GetRoute(NodeId dest); // Non-const: permite lazy cleanup
    std::size_t GetRouteCount() const;
    std::size_t GetPrimaryRouteCount() const;
    std::size_t GetBackupRouteCount() const;
    std::size_t GetDestinationsWithBackupCount() const;
    uint64_t GetRouteEvictionsTotal() const;
    uint64_t GetBackupPromotionsTotal() const;
    std::vector<RouteEntry> GetPrimaryRoutesSnapshot() const;
    std::vector<RouteEntry> GetBackupRoutesSnapshot() const;

    void UpdateFromDvMsg(const DvMessage& msg, const NeighborLinkInfo& link);
    void FloodDvUpdate();
    void PrintRoutingTable() const;
    void PrintRouteTo(NodeId dest) const;
    void DebugDumpRoutingTable() const;
    void PurgeExpiredRoutes();

    std::vector<RouteAnnouncement> GetBestRoutes(uint32_t maxRoutes) const;
    std::vector<RouteAnnouncement> GetBestRoutesPueyo(uint32_t maxRoutes) const;

    void SetDataToaForSf(uint8_t sf, double toaUs);

  private:
    static constexpr uint8_t kDefaultMaxHops = 10;

    void UpdateRoute(const RouteEntry& candidate);
    RouteEntry* LookupRouteMutable(NodeId dest);
    bool IsExpired(const RouteEntry& entry) const;
    Time GetEffectiveTimeout(const RouteEntry& entry) const;
    void NotifyChange(const RouteEntry& entry, const std::string& action) const;
    DvMessage BuildDvMessage() const;
    uint16_t ScoreToCostX1000(uint16_t scoreX100) const;
    uint16_t ToaHopCostUnits(uint8_t sf) const;
    uint16_t ScoreToToaPathUnits(uint16_t scoreX100) const;
    uint16_t ToaUnitsToScoreX100(uint32_t units) const;
    uint16_t ToaUnitsToCostX1000(uint32_t units) const;
    uint16_t ToaUnitsToAdvertMetric(uint32_t units) const;
    uint16_t AdvertMetricToToaPathUnits(uint16_t metric) const;
    uint16_t MaxAdvertMetricValue() const;
    uint16_t EncodeCompositeMetric(double rawMetric) const;
    uint32_t QuantizeCompositeMetric(double rawMetric) const;
    double DecodeCompositeMetric(uint16_t metric) const;
    double GetComparableMetric(const RouteEntry& entry) const;
    /// Convert received batt_mV to energy fraction [0,1] (0 mV = unknown -> 1.0 = no penalty).
    static double BattMvToEFrac(uint16_t battMv);
    double ComputeThesisLinkCost(double toaUs,
                                 uint8_t sf,
                                 double neighborEFrac,
                                 double rssiDbm = 0.0) const;

    /// The built-in composite metric if that is what is plugged, else nullptr.
    DvClCompositeMetric* BuiltinMetricOrNull() const;
    void SetAttrWToa(double v);
    double GetAttrWToa() const;
    void SetAttrWHop(double v);
    double GetAttrWHop() const;
    void SetAttrWEnergy(double v);
    double GetAttrWEnergy() const;
    void SetAttrEnergyLo(double v);
    double GetAttrEnergyLo() const;
    void SetAttrEnergyHi(double v);
    double GetAttrEnergyHi() const;
    void SetAttrEnergyPow(double v);
    double GetAttrEnergyPow() const;
    void SetAttrEnergyMaxPenalty(double v);
    double GetAttrEnergyMaxPenalty() const;
    double GetLocalEnergyFraction() const;
    bool IsRouteUsable(const RouteEntry& entry, uint8_t hopLimit) const;
    bool IsNextHopDcFeasible(NodeId nextHop) const;
    bool IsCandidateBetter(const RouteEntry& candidate, const RouteEntry& current, bool* tie) const;
    bool SelectWorseRouteToEvict(NodeId* outDest, bool* outBackup) const;
    void EnforceRouteTableLimits();
    bool IsInHoldDown(NodeId dest) const;
    void SetHoldDown(NodeId dest);
    void CleanExpiredHoldDowns();

    NodeId m_nodeId{0};
    Time m_routeTimeout{Seconds(300)};
    Time m_expireWindow{Seconds(180)};
    Time m_holdDownTime{Seconds(30)};
    uint8_t m_maxHops{12};
    LoopPrevention m_loopPrevention{LoopPrevention::LEGACY};
    // Distancia factible por destino: la MEJOR (seqno, metrica) que este nodo
    // haya anunciado nunca. Es el estado que sostiene la libertad de bucles.
    struct FeasibleDist
    {
        uint16_t seqno{0};
        uint16_t metric{0};
    };
    // mutable: la FD se actualiza como efecto lateral de CONSTRUIR el anuncio,
    // y los constructores de anuncios son const. Es la promesa que emitimos, no
    // una cache: por eso se escribe justo donde se decide que anunciar.
    mutable std::map<NodeId, FeasibleDist> m_feasible;
    bool IsFeasible(NodeId dst, uint16_t seqno, uint16_t metric) const;
    void UpdateFeasible(NodeId dst, uint16_t seqno, uint16_t metric) const;
    uint8_t m_initTtl{10};
    uint32_t m_maxRoutes{25};
    uint32_t m_sequence{0};
    double m_pathHopWeight{0.05};
    double m_linkWeight{0.70};
    double m_pathWeight{0.25};
    uint16_t m_routeSwitchMinDeltaX100{5};
    /// Amortiguacion de conmutacion de ruta. Es politica del protocolo, no de
    /// la metrica: si se activa solo en unos modos, la comparacion entre
    /// metricas mezcla formula con pegajosidad de ruta. Ver
    /// DvClRouting::GetTypeId (RouteSwitchHysteresis).
    bool m_routeSwitchHysteresis{true};
    bool m_advertiseAllRoutes{false};
    AdvertRoutePolicy m_advertRoutePolicy{AdvertRoutePolicy::TOP_SCORE};
    double m_dataToaBySf[6]{0, 0, 0, 0, 0, 0};

    MetricMode m_metricMode{MetricMode::COMPOSITE_SCORE};
    CostEncoding m_costEncoding{CostEncoding::COST255};
    double m_compositeCostStep{0.025}; // quantization step for COST255 byte
    Ptr<DvClRoutingMetric> m_metric;
    uint32_t m_maxRoutesPerDestination{1};
    uint32_t m_maxTotalRoutes{1024};
    double m_activeTimeoutFactor{2.0};
    Time m_activeDestWindow{Seconds(600)};
    bool m_hasSink{false};
    NodeId m_sinkNodeId{0};
    bool m_pueyoValidationTrace{false};
    bool m_useBeaconSoC{true};
    uint8_t m_socHysteresisPercent{0};
    std::map<NodeId, uint8_t> m_lastNeighborSoC;
    bool m_useDcAwareRouting{false};
    uint8_t m_dcFeasibilityThreshold{20};
    std::map<NodeId, uint8_t> m_neighborDcRemaining;
    RouteChangeCallback m_routeChangeCallback;
    FloodCallback m_floodCallback;
    LocalEnergyFractionCallback m_localEnergyFractionCallback;
    std::map<NodeId, RouteEntry> m_routes;
    std::map<NodeId, RouteEntry> m_backupRoutes;
    std::map<NodeId, RouteEntry> m_inboundRoutes;
    uint64_t m_routeEvictionsTotal{0};
    uint64_t m_backupPromotionsTotal{0};
    std::map<NodeId, uint32_t> m_lastSeq;
    std::unordered_set<NodeId> m_activeDestinations;
    std::unordered_map<NodeId, Time> m_lastDestActivity;
    std::unordered_map<NodeId, Time> m_holdDownUntil;
    Ptr<UniformRandomVariable> m_rng;
};

} // namespace dvcl
} // namespace ns3

#endif /* DV_CL_ROUTING_H */
