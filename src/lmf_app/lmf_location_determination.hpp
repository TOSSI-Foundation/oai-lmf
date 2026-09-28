/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#ifndef FILE_LMF_LOCATION_DETERMINATION_SEEN
#define FILE_LMF_LOCATION_DETERMINATION_SEEN

#include <array>
#include <future>
#include <optional>
#include <mutex>
#include <vector>
#include <map>
#include <tuple>
#include <set>
#include <variant>
#include <shared_mutex>
#include <cmath>

#include <nlohmann/json.hpp>

#include <pistache/http.h>
#include <pistache/router.h>

#include "lmf_gnb.hpp"
#include "lmf_cause_error.hpp"

#include "NRPPA-PDU.h"
#include "NRPPATransactionID.h"
#include "PositioningInformationResponse.h"
#include "MeasurementResponse.h"
#include "PositioningActivationResponse.h"
#include "SRSConfiguration.h"
#include "ProcedureCode.h"
#include "Measurement-ID.h"
#include "TRPInformationResponse.h"
#include "LocationData.h"

#include "GlobalRanNodeId.h"
#include "TRP-ID.h"
#include "PositioningInformationFailure.h"
#include "PositioningActivationFailure.h"
#include "ULRTOAMeas.h"
#include "TRP-MeasurementResponseList.h"
#include "MeasurementFailure.h"
#include "position_estimation.hpp"
namespace oai::lmf::app {

using NrppaPduShared = std::shared_ptr<NRPPA_PDU_t>;
NrppaPduShared share_nrppa_pdu(NRPPA_PDU_t* ptr);

class LocationDetermination {
 public:
  LocationDetermination(std::string supi);
  virtual ~LocationDetermination();

  using pos_act_succ = NrppaPduShared;
  using pos_act_res  = std::variant<pos_act_succ, CauseError>;
  std::promise<pos_act_res> positioning_activation_response;
  pos_act_res positioning_activation_request();
  void handle_positioning_activation_response(
      NrppaPduShared nrppaPdu, NRPPATransactionID_t const& tId,
      PositioningActivationResponse_t const& positioningActivationResponse);
  void handle_positioning_activation_failure(
      NrppaPduShared nrppa,
      PositioningActivationFailure_t const& positioningActivationFailure);
  bool positioning_deactivation_request();

  using pos_info_succ = std::tuple<NrppaPduShared, SRSConfiguration_t const&>;
  using pos_info_res  = std::variant<pos_info_succ, CauseError>;
  std::promise<pos_info_res> positioning_information_response;
  pos_info_res positioning_information_request();
  void handle_positioning_information_response(
      NrppaPduShared nrppaPdu, NRPPATransactionID_t const& tId,
      PositioningInformationResponse_t const& positioningInformationResponse);
  void handle_positioning_information_failure(
      NrppaPduShared nrppaPdu,
      PositioningInformationFailure_t const& positioningInformationFailure);

  using mmr_succ =
      std::tuple<NrppaPduShared, TRP_MeasurementResponseList_t const&>;
  using mmr_res = std::variant<mmr_succ, CauseError>;
  // One promise per NRPPa transaction, so all gNBs can be measured at once.
  std::mutex m_meas;
  std::map<NRPPATransactionID_t, std::promise<mmr_res>> measurement_responses;
  void send_measurement_request(
      oai::lmf::app::Gnb const& gnb,
      SRSConfiguration_t const& srsConfiguration,
      NRPPATransactionID_t const& tId, long slot_shift = 0);
  // Measures every TRP of every gNB on the same SRS occasion. Returns false
  // (round discarded) if the gNBs reported different SFN/slot time stamps.
  bool measurement_round(
      std::map<oai::lmf::app::GnbId, oai::lmf::app::Gnb> const& gnbs,
      SRSConfiguration_t const& srsConfiguration);
  void handle_measurement_response(
      NrppaPduShared nrppaPdu, NRPPATransactionID_t const& tId,
      MeasurementResponse_t const& measurementResponse);
  void handle_measurement_failure(
      NrppaPduShared nrppaPdu, NRPPATransactionID_t const& tId,
      MeasurementFailure_t const& measurementFailure);

  bool n1_n2_message_transfer(
      NrppaPduShared nrppaPdu, NRPPATransactionID_t const& txnId,
      ProcedureCode_t const& procedureCode);
  bool non_ue_n2_message_transfer(
      NrppaPduShared nrppaPdu, NRPPATransactionID_t const& txnId,
      ProcedureCode_t const& procedureCode,
      std::vector<oai::_3gpp::model::GlobalRanNodeId> const& grnidl,
      SRSConfiguration_t* const srsConfigurationBorrowed = nullptr);

  // mapping between nrppa transaction and transaction type
  // TODO: use individual reponse object as value not ResposeType
  //       to have more than one measurement at same time
  std::map<NRPPATransactionID_t, ProcedureCode_t> nrppa_tId;
  std::mutex m_tId;  // guards nrppa_tId (requests and notifications race)

  // Namf_Communication_N1N2MessageTransfer carrying one LPP PDU (TS 29.518 5.2.2.3.1, n1MessageClass LPP).
  void n1_lpp_message_transfer(std::string const& lpp_pdu);
  std::mutex m_lpp;
  std::optional<std::promise<std::string>> lpp_uplink;  // the uplink PDU being waited for
  long lpp_transaction_number = 0;                      // 37.355 LPP-TransactionID, 0..255
  long lpp_dl_sequence_number = 0;                      // 4.3.2, per direction and session
  std::optional<long> lpp_ul_last_sequence_number;      // 4.3.2 duplicate detection

  void throwHttpError(
      std::string const& title, std::string const& detail,
      Pistache::Http::Code const& code =
          Pistache::Http::Code::Internal_Server_Error);

  nlohmann::json compute_location(
      std::map<oai::lmf::app::GnbId, oai::lmf::app::Gnb> const& gnb);

  std::string supi;
  Measurement_ID_t const measurementId;

  // LPP (TS 37.355) over N1 for this location session, TS 23.273 6.11.1.
  // The LCS correlation ID doubles as the NAS routing identifier the UE echoes back (6.17.1 NOTE 1).
  std::string lcs_correlation_id;
  // Capability Transfer (37.355 5.1.1): RequestCapabilities down, ProvideCapabilities back. Returns a summary;
  // throws on a transfer failure, a timeout, or a reply that breaks 5.1.1/5.1.3.
  nlohmann::json lpp_capability_transfer();
  // One NR Multi-RTT Location Information Transfer (37.355 5.3.1) and, with the gNB Rx-Tx of the latest
  // measurement round, one round-trip time (TS 38.305 8.10). Returns the round as JSON, or throws.
  nlohmann::json lpp_multi_rtt_round(bool ntn);
  // Assistance Data Delivery (37.355 5.2.2): NR-DL-PRS-AssistanceData from the TRP Information of the
  // first TRP that reported a PRS Configuration.
  std::vector<std::tuple<GnbId, long, long>> lpp_provide_assistance_data(std::map<GnbId, Gnb> const& gnbs);
  // gNB Rx-Tx of the last measurement round: (sfn, slot, ns). Single-TRP NTN Multi-RTT uses one.
  std::optional<std::tuple<long, long, double>> last_gnb_rxtx;
  // The same, per gNB: in NTN a neighbour TRP is on another satellite, and when it does measure the UE's SRS
  // its gNB Rx-Tx closes a SECOND round trip (TS 38.305 8.10.3 step 13) instead of only a downlink difference.
  std::map<GnbId, std::tuple<long, long, double>> gnb_rxtx;
  // Which gNB each dl-PRS-ID of this session's assistance data belongs to, so a measurement the UE reports
  // against a dl-PRS-ID can be paired with that gNB's own half of the round trip.
  std::map<long, GnbId> gnb_of_dl_prs_id;
  nlohmann::json multi_rtt_rounds = nlohmann::json::array();
  // An uplink LPP PDU the AMF notified for this session (TS 29.518 5.2.2.3.5).
  void handle_lpp_uplink(
      std::string const& correlation_id, std::string const& pdu);

  // UL-RTOA (k1) per gNB/TRP, one entry per accepted measurement round; all
  // vectors have the same length and index i is the same SRS occasion.
  std::map<oai::lmf::app::GnbId, std::map<TRP_ID_t, std::vector<long>>> result;
  unsigned rounds_rejected = 0;

  template<typename T>
  T wait_for_notification(
      std::string const& kind, NRPPATransactionID_t const& tId,
      std::promise<T>& p, std::chrono::milliseconds const& wait_ms);
};
}  // namespace oai::lmf::app

#endif  // FILE_LMF_LOCATION_DETERMINATION_SEEN
