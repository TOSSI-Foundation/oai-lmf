/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#ifndef FILE_LMF_TRP_SEEN
#define FILE_LMF_TRP_SEEN

#include <optional>

#include "CoordinateID.h"
#include "lmf_lpp.hpp"
#include "../nrppa/RelativeCartesianLocation.h"

namespace oai::lmf::app {

class Trp {
 public:
  CoordinateID_t relativeCoordinateID                   = {};
  RelativeCartesianLocation_t relativeCartesianLocation = {};
  // TRP Information (TS 38.455 9.2.x), for LPP assistance data
  std::optional<long> pci;
  std::optional<long> arfcn;
  std::optional<lpp::dl_prs> prs;
  std::optional<double> sfn0_unix;  // SFN Initialisation Time (38.455), s since 1970
  // dl-PRS-ID this LMF gave the TRP in the LPP assistance data of the current session (37.355 6.4.3); it is
  // what the UE quotes back in every measurement, so it is how a measurement finds its satellite.
  std::optional<long> dl_prs_id;
};

}  // namespace oai::lmf::app

#endif  // FILE_LMF_TRP_SEEN
