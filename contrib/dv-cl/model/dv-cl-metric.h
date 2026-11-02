/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef DV_CL_METRIC_H
#define DV_CL_METRIC_H

#include "ns3/object.h"

#include <cstdint>

namespace ns3
{
namespace dvcl
{

/**
 * \ingroup dv-cl
 * \brief Inputs a routing metric may use to price one link.
 *
 * Kept deliberately small: link quality is embedded in the SF selection
 * (a bad link needs a higher SF, hence a larger ToA), so neither RSSI
 * nor SNR enter the metric directly.
 */
struct LinkInputs
{
    double toaUs{0.0};          //!< accumulated/announced time-on-air, microseconds
    uint8_t sf{7};              //!< spreading factor the link operates at (7..12)
    double energyFraction{1.0}; //!< next-hop state of charge b in [0,1]; <0 = unknown
    double batteryMv{-1.0};     //!< next-hop terminal voltage, mV; used only when
                                //!< energyFraction is unknown (<0)
    double rssiDbm{0.0};        //!< RSSI at which the receiver hears this neighbour,
                                //!< dBm; a *local* per-link measurement (not on the
                                //!< wire). 0 means unmeasured. Used only by the RSSI
                                //!< baseline metric; the composite metric ignores it
                                //!< (link quality is already embedded in the SF).
};

class DvClRoutingMetric : public Object
{
  public:
    static TypeId GetTypeId();
    ~DvClRoutingMetric() override = default;

    /**
     * \param in link inputs (ToA, SF, next-hop state of charge)
     * \return non-negative incremental link cost; lower is better
     */
    virtual double ComputeLinkCost(const LinkInputs& in) const = 0;
};

class DvClCompositeMetric : public DvClRoutingMetric
{
  public:
    static TypeId GetTypeId();

    double ComputeLinkCost(const LinkInputs& in) const override;

    double NormalizeToa(double toaUs, uint8_t sf) const;

    /**
     * \brief delta * Psi(b): weighted piecewise energy penalty.
     * \param energyFraction next-hop state of charge in [0,1]; <0 = unknown
     *        (unknown chargers price as full battery: no penalty)
     * \return weighted penalty in [0, WEnergy * PsiMax]
     */
    double EnergyPenalty(double energyFraction, double batteryMv = -1.0) const;

  private:
    static constexpr double kMaxToaUs[6] =
        {143360.0, 256512.0, 462848.0, 829440.0, 1810432.0, 3293184.0};

    bool m_toaNormGlobal{true};
    /// Global ceiling, microseconds: airtime of a 222 B payload at SF12 with a
    /// 16-symbol preamble, i.e. the worst frame this radio ever puts on air.
    double m_toaCeilingUs{8462336.0};

    double m_wToa{0.60};            //!< alpha
    double m_wHop{0.15};            //!< beta (constant per hop)
    double m_wEnergy{0.25};         //!< delta
    double m_energyLo{0.20};        //!< bLo
    double m_energyHi{0.50};        //!< bHi
    double m_energyPow{2.0};        //!< p
    double m_energyMaxPenalty{1.0}; //!< PsiMax
};

class DvClRssiMetric : public DvClRoutingMetric
{
  public:
    static TypeId GetTypeId();

    double ComputeLinkCost(const LinkInputs& in) const override;

  private:
    double m_wRssi{1.0};        //!< weight on the normalised weakness term
    double m_hop{0.05};         //!< strictly-positive per-hop term (monotonicity)
    double m_refDbm{-30.0};     //!< RSSI of a perfect link -> weakness 0
    double m_floorDbm{-137.0};  //!< receiver sensitivity -> weakness 1
    double m_unmeasuredWeakness{0.5}; //!< weakness charged when rssi is unmeasured (0 dBm)
};

} // namespace dvcl
} // namespace ns3

#endif /* DV_CL_METRIC_H */
