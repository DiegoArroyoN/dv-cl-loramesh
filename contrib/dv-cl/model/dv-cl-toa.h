/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef DV_CL_TOA_H
#define DV_CL_TOA_H

#include <cstdint>


namespace ns3
{
namespace dvcl
{

bool LowDataRateOptimizationRequired(uint8_t sf, uint32_t bwHz);

uint32_t ComputeToaUs(uint8_t sf,
                      uint32_t bwHz,
                      uint8_t cr,
                      uint32_t payloadBytes,
                      bool implicitHeader = false,
                      bool lowDataRateOpt = false,
                      bool crcOn = true,
                      uint16_t preambleSymbols = 8);

} // namespace dvcl
} // namespace ns3

#endif /* DV_CL_TOA_H */
