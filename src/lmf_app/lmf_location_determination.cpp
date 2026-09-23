/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#include "lmf_location_determination.hpp"

// The SRS resource offsets a neighbour TRP's measurement request needs shifted; see shift_srs_offsets().
#include "SRSCarrier-List.h"
#include "SRSCarrier-List-Item.h"
#include "SRSResource-List.h"
#include "SRSResource.h"
#include "ResourceType.h"
#include "ResourceTypePeriodic.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <sstream>
#include <boost/range/adaptor/map.hpp>
#include <optional>

#include "3gpp_29.518.h"
#include "AperiodicSRS.h"
#include "InitiatingMessage.h"
#include "LocationData.h"
#include "N1MessageClass.h"
#include "N1MessageContainer.h"
#include "N1N2MessageTransferReqData.h"
#include "lmf_lpp.hpp"
#include "N2InfoContainer.h"
#include "N2InfoContent.h"
#include "N2InformationClass.h"
#include "N2InformationTransferReqData.h"
#include "NgapIeType.h"
#include "NrppaInformation.h"
#include "ProblemDetails.h"
#include "ProtocolIE-Field.h"
#include "SemipersistentSRS.h"
#include "TRP-MeasurementRequestItem.h"
#include "TRP-MeasurementResponseItem.h"
#include "TRPInformationTypeResponseItem.h"
#include "TRPMeasurementQuantities.h"
#include "TRPMeasurementQuantitiesList-Item.h"
#include "TrpMeasuredResultsValue.h"
#include "TrpMeasurementResultItem.h"
#include "UL-RTOAMeasurement.h"
#include "ULRTOAMeas.h"
#include "GNB-RxTxTimeDiff.h"
#include "GNBRxTxTimeDiffMeas.h"
#include "conversions.hpp"
#include "http_client.hpp"
#include "lmf.h"
#include "lmf_app.hpp"
#include "lmf_cause_error.hpp"
#include "lmf_nrf.hpp"
#include "lmf_sbi_helper.hpp"
#include "logger.hpp"
#include "mime_parser.hpp"
#include "position_estimation.hpp"

using namespace std::string_literals;
using namespace oai::_3gpp::model;
using namespace oai::lmf::app;
using namespace oai::lmf::api;
using namespace oai::_3gpp::model;

extern std::shared_ptr<oai::http::http_client> http_client_inst;

namespace {
std::optional<double> relative_cartesian_unit_to_meters(long const xYZunit) {
  switch (xYZunit) {
    case RelativeCartesianLocation__xYZunit_mm:
      return 0.001;
    case RelativeCartesianLocation__xYZunit_cm:
      return 0.01;
    case RelativeCartesianLocation__xYZunit_dm:
      return 0.1;
    default:
      return std::nullopt;
  }
}

char const* relative_cartesian_unit_name(long const xYZunit) {
  switch (xYZunit) {
    case RelativeCartesianLocation__xYZunit_mm:
      return "mm";
    case RelativeCartesianLocation__xYZunit_cm:
      return "cm";
    case RelativeCartesianLocation__xYZunit_dm:
      return "dm";
    default:
      return "unknown";
  }
}
}  // namespace

// provides for asn container.list.array range based for loops
// for (auto const& xyzIEs : xyzResponse.protocolIEs) {
template<typename T>
auto begin(T const& container) {
  return container.list.array;
}

//------------------------------------------------------------------------------
template<typename T>
auto end(T const& container) {
  return container.list.array + container.list.count;
}

//------------------------------------------------------------------------------
template<
    class result_t   = std::chrono::milliseconds,
    class clock_t    = std::chrono::steady_clock,
    class duration_t = std::chrono::milliseconds>
auto elapsed_ms(std::chrono::time_point<clock_t, duration_t> const& start) {
  return std::chrono::duration_cast<result_t>(clock_t::now() - start).count();
}

//------------------------------------------------------------------------------
std::shared_ptr<NRPPA_PDU_t> oai::lmf::app::share_nrppa_pdu(NRPPA_PDU_t* ptr) {
  return {
      ptr, [](NRPPA_PDU_t* ptr) { ASN_STRUCT_FREE(asn_DEF_NRPPA_PDU, ptr); }};
}

//------------------------------------------------------------------------------
LocationDetermination::LocationDetermination(std::string supi)
    : supi{supi}, measurementId{lmf_app_inst->measurement_id_gen.get_uid()} {}

//------------------------------------------------------------------------------
LocationDetermination::~LocationDetermination() {
  lmf_app_inst->measurement_id_gen.free_uid(this->measurementId);
}

//------------------------------------------------------------------------------
bool LocationDetermination::n1_n2_message_transfer(
    NrppaPduShared nrppaPdu, NRPPATransactionID_t const& txnId,
    ProcedureCode_t const& procedureCode) {
  Logger::lmf_app().info("n1_n2_message_transfer");
  // xer_fprint(stdout, &asn_DEF_NRPPA_PDU, nrppaPdu.get());

  asn_encode_to_new_buffer_result_t nrppaPduEnc = asn_encode_to_new_buffer(
      0, ATS_ALIGNED_CANONICAL_PER, &asn_DEF_NRPPA_PDU, nrppaPdu.get());

  if (nrppaPduEnc.result.encoded == -1) {
    Logger::lmf_app().error(
        "Could not encode (at %s)\n", nrppaPduEnc.result.failed_type ?
                                          nrppaPduEnc.result.failed_type->name :
                                          "unknown");
    auto const& title  = "asn nrppa encode failed"s;
    auto const& field  = nrppaPduEnc.result.failed_type ?
                             nrppaPduEnc.result.failed_type->name :
                             "unknown";
    auto const& detail = "Could not encode at; "s + field;
    throwHttpError(title, detail);
  }
  // gc free(nrppaPduEnc.buffer)
  std::unique_ptr<void, decltype(&std::free)> gc{
      nrppaPduEnc.buffer, &std::free};

  std::string amf_uri  = {};
  std::string response = {};
  lmf_sbi_helper::get_amf_comm_n1n2_message_transfer_uri(
      lmf_cfg.amf_addr, this->supi, amf_uri);
  Logger::lmf_app().debug("AMF's URI %s", amf_uri.c_str());

  std::string nrppaMsgStr(
      (char*) nrppaPduEnc.buffer, nrppaPduEnc.result.encoded);
  std::string nrppaMsgHex = {};
  oai::utils::conv::convert_string_2_hex(nrppaMsgStr, nrppaMsgHex);

  RefToBinaryData ngapData = {};
  ngapData.setContentId(N2_NRPPa_CONTENT_ID);

  NgapIeType ngapIeType = {};
  ngapIeType.setEnumValue(NgapIeType_anyOf::eNgapIeType_anyOf::NRPPA_PDU);

  N2InfoContent n2InfoContent = {};
  n2InfoContent.setNgapIeType(ngapIeType);
  n2InfoContent.setNgapData(ngapData);

  NrppaInformation nrppaInformation = {};
  nrppaInformation.setNfId(lmf_nrf_inst->lmf_nf_profile.get_nf_instance_id());
  nrppaInformation.setNrppaPdu(n2InfoContent);

  N2InformationClass n2InformationClass = {};
  n2InformationClass.setEnumValue(
      N2InformationClass_anyOf::eN2InformationClass_anyOf::NRPPA);
  N2InfoContainer n2InfoContainer = {};
  n2InfoContainer.setN2InformationClass(n2InformationClass);
  n2InfoContainer.setNrppaInfo(nrppaInformation);

  N1N2MessageTransferReqData n1n2MessageTransferReqData = {};
  n1n2MessageTransferReqData.setN2InfoContainer(n2InfoContainer);

  nlohmann::json n1n2MessageTransferReq_json;
  to_json(n1n2MessageTransferReq_json, n1n2MessageTransferReqData);

  std::string body      = {};
  std::string json_part = {};
  json_part             = n1n2MessageTransferReq_json.dump();

  mime_parser::create_multipart_related_content(
      body, json_part, CURL_MIME_BOUNDARY, nrppaMsgHex,
      multipart_related_content_part_e::NGAP);

  if (auto const& [iter, inserted] =
          [&] {
            std::scoped_lock lk{this->m_tId};
            return this->nrppa_tId.try_emplace(txnId, procedureCode);
          }();
      !inserted) {
    throwHttpError(
        "n1_n2_message_transfer"s,
        "nrppa id "s + std::to_string(txnId) + " reuse"s);
  }

  // Send HTTP request
  oai::http::request http_request =
      http_client_inst->prepare_multipart_request(amf_uri, body);
  auto http_response = http_client_inst->send_http_request(
      oai::common::sbi::method_e::POST, http_request);
  response = http_response.body;

  Logger::lmf_app().info("Response from AMF: %s", response);

  auto const& rspData_json = nlohmann::json::parse(response);
  if (!rspData_json.contains("cause") ||
      rspData_json["cause"] !=
          n1_n2_message_transfer_cause_e2str[N1_N2_TRANSFER_INITIATED]) {
    {
      std::scoped_lock lk{this->m_tId};
      this->nrppa_tId.erase(txnId);
    }
    lmf_app_inst->nrppa_tid_gen.free_uid(txnId);

    auto const& title = "n1n2message transfer failed"s;
    auto const& cause =
        rspData_json.contains("cause") ?
            n1_n2_message_transfer_cause_e2str[rspData_json["cause"]] :
            "no cause"s;
    auto const& detail = "supi: '"s + this->supi + "': cause: "s + cause;
    throwHttpError(title, detail);
  }

  return true;
}

//------------------------------------------------------------------------------
bool LocationDetermination::non_ue_n2_message_transfer(
    NrppaPduShared nrppaPdu, NRPPATransactionID_t const& txnId,
    ProcedureCode_t const& procedureCode,
    std::vector<GlobalRanNodeId> const& globalRanNodeList,
    SRSConfiguration_t* const ueSrsConfigurationShared) {
  Logger::lmf_app().info("non_ue_n2_message_transfer");
  // xer_fprint(stdout, &asn_DEF_NRPPA_PDU, nrppaPdu.get());

  asn_encode_to_new_buffer_result_t nrppaPduEnc = asn_encode_to_new_buffer(
      0, ATS_ALIGNED_CANONICAL_PER, &asn_DEF_NRPPA_PDU, nrppaPdu.get());
  if (ueSrsConfigurationShared != nullptr) {
    // don't free, it's from positioning information request
    *ueSrsConfigurationShared = {};
  }

  if (nrppaPduEnc.result.encoded == -1) {
    Logger::lmf_app().error(
        "Could not encode (at %s)\n", nrppaPduEnc.result.failed_type ?
                                          nrppaPduEnc.result.failed_type->name :
                                          "unknown");
    auto const& title  = "asn nrppa encode failed"s;
    auto const& field  = nrppaPduEnc.result.failed_type ?
                             nrppaPduEnc.result.failed_type->name :
                             "unknown";
    auto const& detail = "Could not encode at; "s + field;
    throwHttpError(title, detail);
  }
  std::unique_ptr<void, decltype(&std::free)> gc{
      nrppaPduEnc.buffer, &std::free};

  std::string amf_uri  = {};
  std::string method   = "POST";
  std::string response = {};
  lmf_sbi_helper::get_amf_comm_non_ue_n1n2_message_transfer_uri(
      lmf_cfg.amf_addr, amf_uri);

  Logger::lmf_app().debug("AMF's URI %s", amf_uri.c_str());

  std::string nrppaMsgStr(
      (char*) nrppaPduEnc.buffer, nrppaPduEnc.result.encoded);
  std::string nrppaMsgHex = {};
  oai::utils::conv::convert_string_2_hex(nrppaMsgStr, nrppaMsgHex);

  RefToBinaryData ngapData = {};
  ngapData.setContentId(N2_NRPPa_CONTENT_ID);

  NgapIeType ngapIeType = {};
  ngapIeType.setEnumValue(NgapIeType_anyOf::eNgapIeType_anyOf::NRPPA_PDU);

  N2InfoContent n2InfoContent = {};
  n2InfoContent.setNgapIeType(ngapIeType);
  n2InfoContent.setNgapData(ngapData);

  NrppaInformation nrppaInformation = {};
  nrppaInformation.setNfId(lmf_nrf_inst->lmf_nf_profile.get_nf_instance_id());
  nrppaInformation.setNrppaPdu(n2InfoContent);

  N2InformationClass n2InformationClass = {};
  n2InformationClass.setEnumValue(
      N2InformationClass_anyOf::eN2InformationClass_anyOf::NRPPA);
  N2InfoContainer n2InfoContainer = {};
  n2InfoContainer.setN2InformationClass(n2InformationClass);
  n2InfoContainer.setNrppaInfo(nrppaInformation);

  N2InformationTransferReqData n2InformationTransferReqData;
  n2InformationTransferReqData.setN2Information(n2InfoContainer);
  if (globalRanNodeList.size() > 0) {
    Logger::lmf_app().debug(
        "non_ue_n2_message_transfer: globalRanNodeList set, send to %d gNBs",
        globalRanNodeList.size());
    n2InformationTransferReqData.setGlobalRanNodeList(globalRanNodeList);
  } else {
    Logger::lmf_app().debug(
        "non_ue_n2_message_transfer: globalRanNodeList not set, send to all "
        "gNBs using ratSelector");
    RatSelector ratSelector;
    ratSelector.setEnumValue(RatSelector_anyOf::eRatSelector_anyOf::NR);
    n2InformationTransferReqData.setRatSelector(ratSelector);
  }

  nlohmann::json n2InformationTransferReqData_json;
  to_json(n2InformationTransferReqData_json, n2InformationTransferReqData);

  std::string body      = {};
  std::string json_part = {};
  json_part             = n2InformationTransferReqData_json.dump();

  mime_parser::create_multipart_related_content(
      body, json_part, CURL_MIME_BOUNDARY, nrppaMsgHex,
      multipart_related_content_part_e::NGAP);

  // avoid
  // [error] extract_nrppaTxnId2Supi: unknown nrppa txn id:0
  // prepare receiving n2 notification before sending request,
  // because if the amf/gnb is fast the nrppa response
  // can be received before NON_UE_N2_TRANSFER_INITIATED
  if (auto const& [iter, inserted] =
          [&] {
            std::scoped_lock lk{this->m_tId};
            return this->nrppa_tId.try_emplace(txnId, procedureCode);
          }();
      !inserted) {
    throwHttpError(
        "non-ue n2 message transfer: "s,
        "nrppa id "s + std::to_string(txnId) + " reuse"s);
  }
  lmf_app_inst->insert_nrppaTxnId2supi(txnId, this->supi);

  // Send HTTP request
  oai::http::request http_request =
      http_client_inst->prepare_multipart_request(amf_uri, body);
  auto http_response = http_client_inst->send_http_request(
      oai::common::sbi::method_e::POST, http_request);
  response = http_response.body;

  Logger::lmf_app().info("Response from AMF: %s", response);

  // N2InformationTransferRspData;
  // N2InformationTransferError
  // N2InformationTransferResult

  auto const& rspData_json = nlohmann::json::parse(response);
  if ((rspData_json.contains("cause") &&
       (rspData_json["cause"] != "NON_UE_N2_TRANSFER_INITIATED")) ||
      (rspData_json.contains("result") &&
       (rspData_json["result"] != "N2_INFO_TRANSFER_INITIATED"))) {
    {
      std::scoped_lock lk{this->m_tId};
      this->nrppa_tId.erase(txnId);
    }
    lmf_app_inst->nrppa_tid_gen.free_uid(txnId);

    auto const& title = "non-ue-n2-message transfer failed"s;
    auto const& cause =
        rspData_json.contains("cause") ?
            non_ue_n2_message_transfer_cause_e2str[rspData_json["cause"]] :
            "no cause"s;
    auto const& detail = "supi: '"s + this->supi + "': cause: "s + cause;
    throwHttpError(title, detail);
  }

  return true;
}

//------------------------------------------------------------------------------
template<typename T>
T LocationDetermination::wait_for_notification(
    std::string const& kind, NRPPATransactionID_t const& tId,
    std::promise<T>& p, std::chrono::milliseconds const& wait_ms) {
  Logger::lmf_app().info(
      "waiting %dms for %s notification for supi %s, tId: %d", wait_ms.count(),
      kind, this->supi, tId);

  auto const& start = std::chrono::steady_clock::now();
  auto f            = p.get_future();
  switch (auto const& rc = f.wait_for(wait_ms); rc) {
    case std::future_status::timeout: {
      this->throwHttpError(
          kind + " notification timeout"s,
          "waited "s + std::to_string(wait_ms.count()) + "ms"s);
    } break;

    case std::future_status::ready: {
      Logger::lmf_app().info(
          kind + " notifiaction received for supi: %s waiting %dms"s,
          this->supi, elapsed_ms(start));
    } break;

    default:
      this->throwHttpError(
          kind,
          "unhandled future_status: "s + std::to_string(static_cast<int>(rc)));
  }
  return f.get();
}

//------------------------------------------------------------------------------
LocationDetermination::pos_info_res
LocationDetermination::positioning_information_request() {
  auto const& tId = lmf_app_inst->nrppa_tid_gen.get_uid();
  Logger::lmf_app().info("positioning_information_request: tId: %d", tId);
  auto initiatingMessage =
      (InitiatingMessage_t*) malloc(sizeof(InitiatingMessage_t));
  *initiatingMessage = InitiatingMessage_t{
      .procedureCode      = ProcedureCode_id_positioningInformationExchange,
      .criticality        = Criticality_reject,
      .nrppatransactionID = tId,
      .value =
          {.present =
               InitiatingMessage__value_PR_PositioningInformationRequest},
  };
  auto ies = &initiatingMessage->value.choice.PositioningInformationRequest
                  .protocolIEs.list;

  auto requestedSRSTransmissionCharacteristics =
      (PositioningInformationRequest_IEs_t*) malloc(
          sizeof(PositioningInformationRequest_IEs_t));
  *requestedSRSTransmissionCharacteristics = PositioningInformationRequest_IEs_t{
      .id          = ProtocolIE_ID_id_RequestedSRSTransmissionCharacteristics,
      .criticality = Criticality_ignore,
      .value =
          {
              .present =
                  PositioningInformationRequest_IEs__value_PR_RequestedSRSTransmissionCharacteristics,
              .choice =
                  {
                      .RequestedSRSTransmissionCharacteristics =
                          {
                              .resourceType =
                                  RequestedSRSTransmissionCharacteristics__resourceType_aperiodic,
                              .bandwidth =
                                  {
                                      .present = BandwidthSRS_PR_fR1,
                                      .choice =
                                          {.fR1 = BandwidthSRS__fR1_mHz100},
                                  },
                          },
                  },
          },
  };
  ASN_SEQUENCE_ADD(ies, requestedSRSTransmissionCharacteristics);

  auto nrppaPdu = (NRPPA_PDU_t*) malloc(sizeof(NRPPA_PDU_t));
  *nrppaPdu     = NRPPA_PDU_t{
      .present = NRPPA_PDU_PR_initiatingMessage,
      .choice  = {.initiatingMessage = initiatingMessage},
  };

  this->positioning_information_response = {};
  this->n1_n2_message_transfer(
      share_nrppa_pdu(nrppaPdu), tId,
      ProcedureCode_id_positioningInformationExchange);
  return this->wait_for_notification(
      "positioning information", tId, this->positioning_information_response,
      lmf_cfg.positioning_wait_ms);
}

//------------------------------------------------------------------------------
// The UE's SRS configuration describes when it transmits in the SERVING cell's frame grid. A neighbour TRP
// counts frames on its own, so the same transmission falls slot_shift slots later there (TS 38.455 SRSResource,
// ResourceTypePeriodic.offset). Without the shift the neighbour looks in the wrong slot and measures nothing.
//
// The structure is SHARED with every other gNB's request - the IE holds a shallow copy, so writing through it
// writes the caller's own SRSConfiguration. It therefore has to be put back exactly as it was found, which
// the caller does after the transfer; leaving it shifted made the serving gNB's request drift by slot_shift
// every round, so from the second round on it asked the serving cell to measure an SRS the UE does not send.
// The encode inside non_ue_n2_message_transfer() is synchronous, so nothing else observes it in between.
static void shift_srs_offsets(SRSConfiguration_t& cfg, long slot_shift) {
  static long const periodicity_slots[] = {1, 2, 4, 5, 8, 10, 16, 20, 32, 40, 64, 80, 160, 320, 640, 1280, 2560};
  for (int c = 0; c < cfg.sRSCarrier_List.list.count; ++c) {
    auto* res_list = cfg.sRSCarrier_List.list.array[c]->activeULBWP.sRSConfig.sRSResource_List;
    if (res_list == nullptr) continue;
    for (int r = 0; r < res_list->list.count; ++r) {
      auto& type = res_list->list.array[r]->resourceType;
      if (type.present != ResourceType_PR_periodic || type.choice.periodic == nullptr) continue;
      long const p = type.choice.periodic->periodicity;
      if (p < 0 || p >= (long)(sizeof(periodicity_slots) / sizeof(periodicity_slots[0]))) continue;
      long const slots = periodicity_slots[p];
      type.choice.periodic->offset = ((type.choice.periodic->offset + slot_shift) % slots + slots) % slots;
    }
  }
}

void LocationDetermination::send_measurement_request(
    Gnb const& gnb, SRSConfiguration_t const& srsConfigurationUE,
    NRPPATransactionID_t const& tId, long slot_shift) {
  auto const& globalRanNodeList = std::vector{gnb.ncgi};
  auto const& trpIdRng          = boost::adaptors::keys(gnb.trp);
  auto const& trpIds            = std::set(trpIdRng.begin(), trpIdRng.end());

  Logger::lmf_app().info("measurement request: tId: %d", tId);

  auto initiatingMessage =
      (InitiatingMessage_t*) malloc(sizeof(InitiatingMessage_t));
  *initiatingMessage = InitiatingMessage_t{
      .procedureCode      = ProcedureCode_id_Measurement,
      .criticality        = Criticality_reject,
      .nrppatransactionID = tId,
      .value = {.present = InitiatingMessage__value_PR_MeasurementRequest},
  };
  auto ies =
      &initiatingMessage->value.choice.MeasurementRequest.protocolIEs.list;

  auto lmfMeasurementIdIe =
      (MeasurementRequest_IEs_t*) malloc(sizeof(MeasurementRequest_IEs_t));
  *lmfMeasurementIdIe = MeasurementRequest_IEs_t{
      .id          = ProtocolIE_ID_id_LMF_Measurement_ID,
      .criticality = Criticality_reject,
      .value =
          {
              .present = MeasurementRequest_IEs__value_PR_Measurement_ID,
              .choice  = {.Measurement_ID = this->measurementId},
          },
  };
  ASN_SEQUENCE_ADD(ies, lmfMeasurementIdIe);

  auto trpMeasurementRequestListIe =
      (MeasurementRequest_IEs_t*) malloc(sizeof(MeasurementRequest_IEs_t));
  *trpMeasurementRequestListIe = {
      .id          = ProtocolIE_ID_id_TRP_MeasurementRequestList,
      .criticality = Criticality_reject,
      .value =
          {
              .present =
                  MeasurementRequest_IEs__value_PR_TRP_MeasurementRequestList,
          },
  };
  auto trpMeasurementRequestList = &trpMeasurementRequestListIe->value.choice
                                        .TRP_MeasurementRequestList.list;
  for (auto const& trpId : trpIds) {
    auto trpMeasurementRequestItem = (TRP_MeasurementRequestItem_t*) malloc(
        sizeof(TRP_MeasurementRequestItem_t));
    *trpMeasurementRequestItem = TRP_MeasurementRequestItem_t{
        .tRP_ID = trpId,
    };
    ASN_SEQUENCE_ADD(trpMeasurementRequestList, trpMeasurementRequestItem);
  }
  ASN_SEQUENCE_ADD(ies, trpMeasurementRequestListIe);

  auto reportCharacteristics =
      (MeasurementRequest_IEs_t*) malloc(sizeof(MeasurementRequest_IEs_t));
  *reportCharacteristics = MeasurementRequest_IEs_t{
      .id          = ProtocolIE_ID_id_ReportCharacteristics,
      .criticality = Criticality_reject,
      .value =
          {
              .present = MeasurementRequest_IEs__value_PR_ReportCharacteristics,
              .choice =
                  {
                      .ReportCharacteristics = ReportCharacteristics_onDemand,
                  },
          },
  };
  ASN_SEQUENCE_ADD(ies, reportCharacteristics);

  auto srsConfigurationIE =
      (MeasurementRequest_IEs_t*) malloc(sizeof(MeasurementRequest_IEs_t));
  *srsConfigurationIE = MeasurementRequest_IEs_t{
      .id          = ProtocolIE_ID_id_SRSConfiguration,
      .criticality = Criticality_ignore,
      .value =
          {
              .present = MeasurementRequest_IEs__value_PR_SRSConfiguration,
              .choice =
                  {
                      .SRSConfiguration = srsConfigurationUE,
                  },
          },
  };
  auto ueSrsConfigurationShared =
      &srsConfigurationIE->value.choice.SRSConfiguration;
  if (slot_shift != 0) {
    shift_srs_offsets(*ueSrsConfigurationShared, slot_shift);
    Logger::lmf_app().info(
        "measurement request: gnbId 0x%x counts frames %ld slots from the serving cell, SRS offsets shifted",
        gnb.id, slot_shift);
  }
  ASN_SEQUENCE_ADD(ies, srsConfigurationIE);

  // TRP Measurement Quantities is mandatory (TS 38.455, 9.1.4.1) and was
  // missing. Ask for UL-RTOA and state the reporting granularity explicitly:
  // compute_location() below assumes k1, and a gNB may reject the request (or
  // assert, as OCUDU does) when the factor is absent.
  constexpr long kTimingReportingGranularityFactor = 1;  // k1
  auto trpMeasurementQuantitiesIe =
      (MeasurementRequest_IEs_t*) calloc(1, sizeof(MeasurementRequest_IEs_t));
  trpMeasurementQuantitiesIe->id          = ProtocolIE_ID_id_TRPMeasurementQuantities;
  trpMeasurementQuantitiesIe->criticality = Criticality_reject;
  trpMeasurementQuantitiesIe->value.present =
      MeasurementRequest_IEs__value_PR_TRPMeasurementQuantities;

  // UL-RTOA feeds the solver below. gNB Rx-Tx is the gNB half of Multi-RTT
  // (RTT = UE Rx-Tx + gNB Rx-Tx), which TS 38.305 8.10.1 uses for NTN network
  // verification of UE location at a single TRP over several time instances;
  // it is only logged for now. Both at k1.
  for (auto const type : {TRPMeasurementType_uL_RTOA,
                          TRPMeasurementType_gNB_RxTxTimeDiff}) {
    auto trpMeasurementQuantitiesItem =
        (TRPMeasurementQuantitiesList_Item_t*) calloc(
            1, sizeof(TRPMeasurementQuantitiesList_Item_t));
    trpMeasurementQuantitiesItem->tRPMeasurementQuantities_Item = type;
    trpMeasurementQuantitiesItem->timingReportingGranularityFactor =
        (long*) calloc(1, sizeof(long));
    *trpMeasurementQuantitiesItem->timingReportingGranularityFactor =
        kTimingReportingGranularityFactor;
    ASN_SEQUENCE_ADD(
        &trpMeasurementQuantitiesIe->value.choice.TRPMeasurementQuantities.list,
        trpMeasurementQuantitiesItem);
  }
  ASN_SEQUENCE_ADD(ies, trpMeasurementQuantitiesIe);

  auto nrppaPdu = (NRPPA_PDU_t*) malloc(sizeof(NRPPA_PDU_t));
  *nrppaPdu     = NRPPA_PDU_t{
      .present = NRPPA_PDU_PR_initiatingMessage,
      .choice  = {.initiatingMessage = initiatingMessage},
  };

  this->non_ue_n2_message_transfer(
      share_nrppa_pdu(nrppaPdu), tId, ProcedureCode_id_Measurement,
      globalRanNodeList, ueSrsConfigurationShared);
  // Put the shared offsets back. non_ue_n2_message_transfer() empties the IE's copy so the PDU can be freed
  // without taking the caller's lists with it, so the restore has to go through the original.
  if (slot_shift != 0) {
    shift_srs_offsets(const_cast<SRSConfiguration_t&>(srsConfigurationUE), -slot_shift);
  }
  // MEASUREMENT RESPONSE ( 9.1.4.2 NRPPa TS 38.455 ) arrives via
  // handle_measurement_response() and completes measurement_responses[tId].
}

//------------------------------------------------------------------------------
bool LocationDetermination::measurement_round(
    std::map<GnbId, Gnb> const& gnbs, SRSConfiguration_t const& srsConfigurationUE) {
  // Register every transaction before sending anything: a fast gNB may answer
  // before the AMF has acknowledged the transfer.
  std::vector<std::tuple<Gnb const*, NRPPATransactionID_t, std::future<mmr_res>>>
      pending;
  for (auto const& [gnbId, gnb] : gnbs) {
    auto const tId = lmf_app_inst->nrppa_tid_gen.get_uid();
    std::scoped_lock lk{this->m_meas};
    pending.emplace_back(
        &gnb, tId, this->measurement_responses[tId].get_future());
  }
  auto const cleanup = [&] {
    for (auto const& [gnb, tId, fut] : pending) {
      bool waiting;  // still registered => no answer was handled
      {
        std::scoped_lock lk{this->m_meas};
        waiting = this->measurement_responses.erase(tId) > 0;
      }
      if (!waiting) continue;  // handler already released tId
      bool sent;
      {
        std::scoped_lock lk{this->m_tId};
        sent = this->nrppa_tId.erase(tId) > 0;
      }
      if (sent) {
        try {
          lmf_app_inst->erase_nrppaTxnId2Supi(tId);
        } catch (...) {
        }
      }
      lmf_app_inst->nrppa_tid_gen.free_uid(tId);
    }
  };

  try {
    // The serving gNB - the one whose TRP the assistance data made the reference - and how far each other
    // gNB's frame grid sits from it. 15 kHz here, so one slot per millisecond.
    auto const sfn0_of = [&gnbs](GnbId g) -> std::optional<double> {
      auto const gi = gnbs.find(g);
      if (gi == gnbs.end()) return std::nullopt;
      for (auto const& [trpId, trp] : gi->second.trp)
        if (trp.sfn0_unix) return trp.sfn0_unix;
      return std::nullopt;
    };
    auto const ref = this->gnb_of_dl_prs_id.find(0);
    std::optional<double> const ref_sfn0 =
        ref == this->gnb_of_dl_prs_id.end() ? std::optional<double>{} : sfn0_of(ref->second);

    // Back-to-back, so every gNB measures the same (next) SRS occasion.
    std::map<GnbId, long> shift_of;
    for (auto const& [gnb, tId, fut] : pending) {
      long shift = 0;
      if (ref_sfn0 && ref != this->gnb_of_dl_prs_id.end() && gnb->id != ref->second) {
        if (auto const s = sfn0_of(gnb->id)) shift = std::lround((*ref_sfn0 - *s) * 1e3);
      }
      shift_of[gnb->id] = shift;
      Logger::lmf_app().info(
          "measurement request: gnbId: 0x%x tId: %d", gnb->id, tId);
      this->send_measurement_request(*gnb, srsConfigurationUE, tId, shift);
    }

    auto const deadline =
        std::chrono::steady_clock::now() + lmf_cfg.measurement_wait_ms;
    // (sfn, slot) of every TRP measurement; one value if all saw the same SRS.
    std::set<std::pair<long, long>> occasions;
    std::map<GnbId, std::map<TRP_ID_t, long>> round;
    for (auto& [gnb, tId, fut] : pending) {
      // A gNB that does not answer, or answers with a failure, is skipped rather than fatal. In NTN a
      // neighbour TRP is on another satellite: the UE's uplink is pre-compensated for the serving satellite
      // and arrives at the neighbour milliseconds away from its own alignment, so the neighbour legitimately
      // has no uplink measurement to give while still being the TRP whose downlink the UE measured. TS 38.305
      // 8.10.3 step 13 pairs "each gNB for which corresponding UL and DL measurements were provided", not all
      // of them. A stack that is wholly dead is still caught: the round is rejected below if nothing answered,
      // and the session throws if no round is ever accepted.
      if (fut.wait_until(deadline) != std::future_status::ready) {
        Logger::lmf_app().warn(
            "measurement: no answer from gnbId 0x%x within %ldms, that TRP sits this round out",
            gnb->id, (long)lmf_cfg.measurement_wait_ms.count());
        continue;
      }
      auto const res = fut.get();
      if (std::holds_alternative<CauseError>(res)) {
        Logger::lmf_app().warn("measurement: gnbId 0x%x: %s, that TRP sits this round out", gnb->id,
                               std::get<CauseError>(res).msg().c_str());
        continue;
      }
      auto const& trpList = std::get<1>(std::get<mmr_succ>(res));
      for (auto const& trpMeasurement : trpList) {
        for (auto const& m : trpMeasurement->measurementResult) {
          if (m->measuredResultsValue.present ==
              TrpMeasuredResultsValue_PR_gNB_RxTxTimeDiff) {
            // Own log line, deliberately not matching the UL-RTOA one, so
            // scripts that parse "trpId: N k1: N" are not polluted by it.
            // Same k1 scale as UL-RTOA: 0 ns is 492512, steps of 2*Tc.
            auto const& rxtx =
                m->measuredResultsValue.choice.gNB_RxTxTimeDiff->rxTxTimeDiff;
            if (rxtx.present == GNBRxTxTimeDiffMeas_PR_k1) {
              auto const& ts = m->timeStamp;
              // Stored on the REFERENCE gNB's frame grid, like the occasions above, so that pairing a
              // neighbour's half with the UE's report (which counts frames on the serving cell) compares
              // like with like. The serving gNB's own shift is 0, so its values are untouched.
              long const g_slot =
                  ts.slotIndex.present == TimeStampSlotIndex_PR_sCS_15 ? ts.slotIndex.choice.sCS_15 : -1;
              long const g_sub =
                  ((long(ts.systemFrameNumber) * 10 + g_slot - shift_of[gnb->id]) % 10240 + 10240) % 10240;
              this->gnb_rxtx[gnb->id] = std::make_tuple(
                  g_sub / 10, g_slot < 0 ? -1 : g_sub % 10,
                  (rxtx.choice.k1 - 492512) * 1e9 / (4096.0 * 480000 / 2));
              this->last_gnb_rxtx = std::make_tuple(
                  long(ts.systemFrameNumber),
                  long(ts.slotIndex.present == TimeStampSlotIndex_PR_sCS_15 ? ts.slotIndex.choice.sCS_15 : -1),
                  (rxtx.choice.k1 - 492512) * 1e9 / (4096.0 * 480000 / 2));
              Logger::lmf_app().info(
                  "measurement: gnbId: 0x%x trpId: %d gnbRxTx k1: %ld "
                  "(%.1f ns) sfn: %ld",
                  gnb->id, trpMeasurement->tRP_ID, rxtx.choice.k1,
                  (rxtx.choice.k1 - 492512) * 1e9 / (4096.0 * 480000 / 2),
                  m->timeStamp.systemFrameNumber);
            }
            continue;
          }
          if (m->measuredResultsValue.present !=
              TrpMeasuredResultsValue_PR_uL_RTOA) {
            continue;
          }
          auto const& meas =
              m->measuredResultsValue.choice.uL_RTOA->uLRTOAmeas;
          if (meas.present != ULRTOAMeas_PR_k1) {
            Logger::lmf_app().warn(
                "measurement: gnbId: 0x%x trpId: %d: expected k1, got k%d",
                gnb->id, trpMeasurement->tRP_ID, meas.present - 1);
            continue;
          }
          auto const& ts = m->timeStamp;
          long const slot =
              ts.slotIndex.present == TimeStampSlotIndex_PR_sCS_15  ? ts.slotIndex.choice.sCS_15 :
              ts.slotIndex.present == TimeStampSlotIndex_PR_sCS_30  ? ts.slotIndex.choice.sCS_30 :
              ts.slotIndex.present == TimeStampSlotIndex_PR_sCS_60  ? ts.slotIndex.choice.sCS_60 :
              ts.slotIndex.present == TimeStampSlotIndex_PR_sCS_120 ? ts.slotIndex.choice.sCS_120 :
                                                                      -1;
          // ONE instant, not one (SFN, slot). Each gNB timestamps in its own frame grid, and two gNBs that
          // started seconds apart number the same instant differently - so comparing (SFN, slot) across them
          // rejected every round with "TRPs measured 2 different SRS occasions". Put each back on the
          // reference gNB's grid first, with the same shift its request carried.
          long const sub = ((ts.systemFrameNumber * 10 + slot - shift_of[gnb->id]) % 10240 + 10240) % 10240;
          occasions.emplace(sub / 10, sub % 10);
          round[gnb->id][trpMeasurement->tRP_ID] = meas.choice.k1;
          Logger::lmf_app().info(
              "measurement: gnbId: 0x%x trpId: %d k1: %ld sfn: %ld slot: %ld",
              gnb->id, trpMeasurement->tRP_ID, meas.choice.k1,
              ts.systemFrameNumber, slot);
        }
      }
    }
    cleanup();

    // At least one TRP must have answered, and those that did must have measured one SRS occasion.
    for (auto const& [gnbId, gnb] : gnbs)
      for (auto const& [trpId, trp] : gnb.trp)
        if (round.count(gnbId) == 0 || round.at(gnbId).count(trpId) == 0)
          Logger::lmf_app().info("measurement: no UL-RTOA from gnbId: 0x%x trpId: %d this round", gnbId, trpId);
    if (round.empty()) {
      Logger::lmf_app().warn("measurement round discarded: no TRP measured the UE's SRS");
      ++this->rounds_rejected;
      return false;
    }
    if (occasions.size() != 1) {
      Logger::lmf_app().warn(
          "measurement round discarded: TRPs measured %zu different SRS "
          "occasions",
          occasions.size());
      ++this->rounds_rejected;
      return false;
    }
    for (auto const& [gnbId, trps] : round) {
      for (auto const& [trpId, k1] : trps) {
        this->result[gnbId][trpId].push_back(k1);
      }
    }
    return true;
  } catch (...) {
    cleanup();
    throw;
  }
}

//------------------------------------------------------------------------------
void LocationDetermination::handle_measurement_response(
    NrppaPduShared nrppaPdu, NRPPATransactionID_t const& tId,
    MeasurementResponse_t const& measurementResponse) {
  Logger::lmf_app().info("handle measurement response: tId: %d", tId);
  std::scoped_lock lk{this->m_meas};
  auto const it = this->measurement_responses.find(tId);
  if (it == this->measurement_responses.end()) {
    Logger::lmf_app().warn("measurement response: no waiter for tId: %d", tId);
    return;
  }
  for (auto const& ie : measurementResponse.protocolIEs) {
    if (ie->id == ProtocolIE_ID_id_TRP_MeasurementResponseList &&
        ie->value.present ==
            MeasurementResponse_IEs__value_PR_TRP_MeasurementResponseList) {
      it->second.set_value(std::make_tuple(
          nrppaPdu, std::cref(ie->value.choice.TRP_MeasurementResponseList)));
      this->measurement_responses.erase(it);
      return;
    }
  }
  // No TRP measurement list in the response: report it as a failure.
  it->second.set_value(CauseError{});
  this->measurement_responses.erase(it);
}

//------------------------------------------------------------------------------
void LocationDetermination::handle_measurement_failure(
    NrppaPduShared nrppaPdu, NRPPATransactionID_t const& tId,
    MeasurementFailure_t const& measurementFailure) {
  std::scoped_lock lk{this->m_meas};
  auto const it = this->measurement_responses.find(tId);
  if (it == this->measurement_responses.end()) {
    Logger::lmf_app().warn("measurement failure: no waiter for tId: %d", tId);
    return;
  }
  it->second.set_value(CauseError::parse(
      measurementFailure, MeasurementFailure_IEs__value_PR_Cause));
  this->measurement_responses.erase(it);
}

//------------------------------------------------------------------------------
void LocationDetermination::handle_positioning_information_response(
    NrppaPduShared nrppaPdu, NRPPATransactionID_t const& tId,
    PositioningInformationResponse_t const& positioningInformationResponse) {
  Logger::lmf_app().info("handle positioning information response");
  std::optional<pos_info_res> res;

  for (auto const& positioningInformationIE :
       positioningInformationResponse.protocolIEs) {
    if (positioningInformationIE->id == ProtocolIE_ID_id_SRSConfiguration &&
        positioningInformationIE->value.present ==
            PositioningInformationResponse_IEs__value_PR_SRSConfiguration) {
      auto const& srsCfg =
          positioningInformationIE->value.choice.SRSConfiguration;
      res.emplace(std::make_tuple(nrppaPdu, std::cref(srsCfg)));
    }
  }
  if (!res.has_value()) {
    try {
      throwHttpError(
          "handle_positioning_information_response: srsConfiguration missing",
          "srsConfiguration needed for non-ue measurement request");
    } catch (...) {
      this->positioning_information_response.set_exception(
          std::current_exception());
      throw;
    }
  }
  this->positioning_information_response.set_value(res.value());
}

//------------------------------------------------------------------------------
void LocationDetermination::handle_positioning_information_failure(
    NrppaPduShared nrppaPdu,
    PositioningInformationFailure_t const& positioningInformationFailure) {
  auto err = CauseError::parse(
      positioningInformationFailure,
      PositioningInformationFailure_IEs__value_PR_Cause);
  this->positioning_information_response.set_value(err);
}

//------------------------------------------------------------------------------
// 9.1.1.17 POSITIONING ACTIVATION REQUEST
LocationDetermination::pos_act_res
LocationDetermination::positioning_activation_request() {
  auto const& tId = lmf_app_inst->nrppa_tid_gen.get_uid();
  Logger::lmf_app().info("positioning_activation_request: tId: %d", tId);
  auto initiatingMessage =
      (InitiatingMessage_t*) malloc(sizeof(InitiatingMessage_t));
  *initiatingMessage = InitiatingMessage_t{
      .procedureCode      = ProcedureCode_id_positioningActivation,
      .criticality        = Criticality_reject,
      .nrppatransactionID = tId,
      .value =
          {.present = InitiatingMessage__value_PR_PositioningActivationRequest},
  };
  auto ies = &initiatingMessage->value.choice.PositioningActivationRequest
                  .protocolIEs.list;

  // >Aperiodic
  auto aperiodicSRS = (AperiodicSRS_t*) malloc(sizeof(AperiodicSRS_t));
  *aperiodicSRS     = AperiodicSRS_t{
      .aperiodic = AperiodicSRS__aperiodic_true,
  };
  // CHOICE SRS type
  auto aperiodicSRS_ie = (PositioningActivationRequestIEs_t*) malloc(
      sizeof(PositioningActivationRequestIEs_t));
  *aperiodicSRS_ie = PositioningActivationRequestIEs_t{
      .id          = ProtocolIE_ID_id_SRSType,
      .criticality = Criticality_reject,
      .value =
          {
              .present = PositioningActivationRequestIEs__value_PR_SRSType,
              .choice =
                  {
                      .SRSType =
                          {
                              .present = SRSType_PR_aperiodicSRS,
                              .choice =
                                  {
                                      .aperiodicSRS = aperiodicSRS,
                                  },
                          },
                  },
          },
  };
  ASN_SEQUENCE_ADD(ies, aperiodicSRS_ie);
#if 0
  // >Semi-persistent
  auto semipersistentSRS =
      (SemipersistentSRS_t*) malloc(sizeof(SemipersistentSRS_t));
  *semipersistentSRS = SemipersistentSRS_t{
      .sRSResourceSetID = 1,
  };
  // CHOICE SRS type
  auto semipersistentSRS_ie = (PositioningActivationRequestIEs_t*) malloc(
      sizeof(PositioningActivationRequestIEs_t));
  *semipersistentSRS_ie = PositioningActivationRequestIEs_t{
      .id          = ProtocolIE_ID_id_SRSType,
      .criticality = Criticality_reject,
      .value =
          {
              .present = PositioningActivationRequestIEs__value_PR_SRSType,
              .choice =
                  {
                      .SRSType =
                          {
                              .present = SRSType_PR_semipersistentSRS,
                              .choice =
                                  {
                                      .semipersistentSRS = semipersistentSRS,
                                  },
                          },
                  },
          },
  };
  ASN_SEQUENCE_ADD(ies, semipersistentSRS_ie);
#endif
  auto nrppaPdu = (NRPPA_PDU_t*) malloc(sizeof(NRPPA_PDU_t));
  *nrppaPdu     = NRPPA_PDU_t{
      .present = NRPPA_PDU_PR_initiatingMessage,
      .choice  = {.initiatingMessage = initiatingMessage},
  };
  this->positioning_activation_response = {};
  this->n1_n2_message_transfer(
      share_nrppa_pdu(nrppaPdu), tId, ProcedureCode_id_positioningActivation);
  return this->wait_for_notification(
      "positionong activation", tId, this->positioning_activation_response,
      lmf_cfg.positioning_wait_ms);
}

//------------------------------------------------------------------------------
// 9.1.1.20 POSITIONING DEACTIVATION
bool LocationDetermination::positioning_deactivation_request() {
  auto const& tId = lmf_app_inst->nrppa_tid_gen.get_uid();

  auto initiatingMessage =
      (InitiatingMessage_t*) malloc(sizeof(InitiatingMessage_t));
  *initiatingMessage = InitiatingMessage_t{
      .procedureCode      = ProcedureCode_id_positioningDeactivation,
      .criticality        = Criticality_reject,
      .nrppatransactionID = tId,
      .value = {.present = InitiatingMessage__value_PR_PositioningDeactivation},
  };
  auto ies =
      &initiatingMessage->value.choice.PositioningDeactivation.protocolIEs.list;

  // >Release ALL
  auto positioningDeactivationIe = (PositioningDeactivationIEs_t*) malloc(
      sizeof(PositioningDeactivationIEs_t));
  *positioningDeactivationIe = PositioningDeactivationIEs_t{
      .id          = ProtocolIE_ID_id_AbortTransmission,
      .criticality = Criticality_ignore,
      .value =
          {
              .present = PositioningDeactivationIEs__value_PR_AbortTransmission,
              .choice =
                  {
                      .AbortTransmission =
                          {
                              .present = AbortTransmission_PR_releaseALL,
                              .choice =
                                  {
                                      .releaseALL = true,  // meaningless
                                  },
                          },
                  },
          },
  };
  ASN_SEQUENCE_ADD(ies, positioningDeactivationIe);

  auto nrppaPdu = (NRPPA_PDU_t*) malloc(sizeof(NRPPA_PDU_t));
  *nrppaPdu     = NRPPA_PDU_t{
      .present = NRPPA_PDU_PR_initiatingMessage,
      .choice  = {.initiatingMessage = initiatingMessage},
  };

  this->n1_n2_message_transfer(
      share_nrppa_pdu(nrppaPdu), tId, ProcedureCode_id_positioningDeactivation);
  // no success/failure notifiaction defined, nothing to wait for
  {
    std::scoped_lock lk{this->m_tId};
    this->nrppa_tId.erase(tId);
  }

  return true;
}

//------------------------------------------------------------------------------
void LocationDetermination::handle_positioning_activation_response(
    NrppaPduShared nrppaPdu, NRPPATransactionID_t const& tId,
    PositioningActivationResponse_t const& positioningActivationResponse) {
  Logger::lmf_app().info("handle positioning activation response");
  this->positioning_activation_response.set_value({nrppaPdu});
}

//------------------------------------------------------------------------------
void LocationDetermination::handle_positioning_activation_failure(
    NrppaPduShared nrppa,
    PositioningActivationFailure_t const& positioningActivationFailure) {
  auto err = CauseError::parse(
      positioningActivationFailure,
      PositioningActivationFailureIEs__value_PR_Cause);
  this->positioning_activation_response.set_value(err);
}

//------------------------------------------------------------------------------
void LocationDetermination::throwHttpError(
    std::string const& title, std::string const& detail,
    Pistache::Http::Code const& code) {
  oai::lmf::api::lmf_sbi_helper::throwHttpError(
      title, detail, this->supi, code);
}

//------------------------------------------------------------------------------
// Per-TRP fixed delay (fronthaul fibre, RU group delay), e.g.
// LMF_TRP_DELAY_NS="411:1=12.5,412:1=-3.0" (gnbId:trpId=ns). Subtracted from the
// TRP's time of arrival. Calibrate by placing the UE at a known point.
static std::map<std::pair<uint64_t, long>, double> const& trp_delay_ns() {
  static auto const delays = [] {
    std::map<std::pair<uint64_t, long>, double> m;
    char const* env = std::getenv("LMF_TRP_DELAY_NS");
    std::stringstream ss(env ? env : "");
    std::string item;
    while (std::getline(ss, item, ',')) {
      uint64_t gnb = 0;
      long trp     = 0;
      double ns    = 0;
      if (std::sscanf(item.c_str(), " %lu:%ld=%lf", &gnb, &trp, &ns) == 3) {
        m[{gnb, trp}] = ns;
        Logger::lmf_app().info(
            "trp delay calibration: gnbId: %lu trpId: %ld delay: %.3f ns", gnb,
            trp, ns);
      } else {
        Logger::lmf_app().warn("LMF_TRP_DELAY_NS: ignoring '%s'", item.c_str());
      }
    }
    return m;
  }();
  return delays;
}

nlohmann::json LocationDetermination::compute_location(
    std::map<oai::lmf::app::GnbId, oai::lmf::app::Gnb> const& gnbs) {
  std::vector<double> toas;
  std::vector<std::vector<double>> toa_rounds;  // [trp][round], ns
  std::vector<std::array<double, 3>> trp_pos;
  nlohmann::json trps_json = nlohmann::json::array();
  uint32_t Tc_inv   = 4096 * 480000;
  uint16_t K        = 1;
  uint32_t T_inv    = Tc_inv / (1 << K);
  uint32_t T_ns_inv = 1e9;

  for (auto const& [gnbId, trp] : this->result) {
    if (gnbs.count(gnbId) == 0) {
      Logger::lmf_app().warn("unknown gnbId: %d", gnbId);
      continue;
    }
    auto const gnb = gnbs.at(gnbId);
    for (auto const& [trpId, k1Rounds] : trp) {
      if (gnb.trp.count(trpId) == 0) {
        Logger::lmf_app().warn(
            "no such trpId: %d attached to gnbId: %d", trpId, gnbId);
        continue;
      }
      auto const& trp       = gnb.trp.at(trpId);
      auto const unit_scale = relative_cartesian_unit_to_meters(
          trp.relativeCartesianLocation.xYZunit);
      auto const unit =
          relative_cartesian_unit_name(trp.relativeCartesianLocation.xYZunit);

      if (!unit_scale.has_value()) {
        Logger::lmf_app().warn(
            "unsupported trp relative cartesian unit: %ld for gnbId: %llu, "
            "trpId: %ld",
            trp.relativeCartesianLocation.xYZunit,
            static_cast<unsigned long long>(gnbId), trpId);
        continue;
      }

      auto const meters_per_unit = unit_scale.value();
      auto const position_m      = std::array<double, 3>{
          static_cast<double>(trp.relativeCartesianLocation.xvalue) *
              meters_per_unit,
          static_cast<double>(trp.relativeCartesianLocation.yvalue) *
              meters_per_unit,
          static_cast<double>(trp.relativeCartesianLocation.zvalue) *
              meters_per_unit};
      trp_pos.push_back(position_m);

      auto const delay_it = trp_delay_ns().find({gnbId, trpId});
      double const delay_ns =
          delay_it == trp_delay_ns().end() ? 0.0 : delay_it->second;
      std::vector<double> toa_ns;
      for (auto const v : k1Rounds) {
        toa_ns.push_back(
            (static_cast<double>(v) - 492512) * T_ns_inv / T_inv - delay_ns);
      }
      toa_rounds.push_back(toa_ns);
      trps_json.push_back(
          {{"gnbId", gnbId},
           {"trpId", trpId},
           {"posM", position_m},
           {"ulRtoaK1", k1Rounds},
           {"delayCalNs", delay_ns},
           {"toaNs", toa_ns}});
      Logger::lmf_app().debug(
          "gnbId: %llu, trpId: %ld, trpPosMeters(x: %.3f, y: %.3f, z: %.3f), "
          "%zu rounds (%s)",
          static_cast<unsigned long long>(gnbId), trpId, position_m[0],
          position_m[1], position_m[2], k1Rounds.size(), unit);
    }
  }

  // Median over rounds of each TRP's ToA difference to the first TRP. Round r
  // is the same SRS occasion for every TRP, so the UE's own timing drift
  // between occasions cancels. Express it as ToAs relative to TRP 0.
  size_t const nof_rounds = toa_rounds.empty() ? 0 : toa_rounds[0].size();
  nlohmann::json tdoa_rounds_json = nlohmann::json::array();
  if (nof_rounds > 0) {
    toas.push_back(0.0);
    for (size_t i = 1; i < toa_rounds.size(); ++i) {
      std::vector<double> d;
      for (size_t r = 0; r < nof_rounds; ++r) {
        d.push_back(toa_rounds[i][r] - toa_rounds[0][r]);
      }
      tdoa_rounds_json.push_back(d);
      std::sort(d.begin(), d.end());
      auto const n = d.size();
      toas.push_back(n % 2 ? d[n / 2] : (d[n / 2 - 1] + d[n / 2]) / 2.0);
    }
  }

  if (toas.empty() || toas.size() != trp_pos.size()) {
    Logger::lmf_app().error(
        "location: got %zu ToA values for %zu TRPs", toas.size(),
        trp_pos.size());
    return nlohmann::json{
        {"error", "Not enough TRP measurements to compute a location"},
        {"trps", trps_json}};
  }

  // Reference ToA index (assuming the first TRP as the reference)
  int ref_toa_idx = 0;
  std::vector<double> tdoa_ns(toas.size() - 1);
  int idx = 0;

  // Calculate TDoA values relative to the reference TRP
  for (size_t i = 0; i < toas.size(); ++i) {
    if (i == ref_toa_idx) continue;  // Skip the reference TRP
    tdoa_ns[idx] = (toas[i] - toas[ref_toa_idx]);
    ++idx;
  }

  // Debug output for TDoA values
  std::cout << "[pos_est] TDoA Values:" << std::endl;
  for (const auto& tau : tdoa_ns) {
    std::cout << "TDoA: " << tau << " nsec" << std::endl;
  }

  // Speed of light in meters per nanosecond
  const double SPEED_OF_LIGHT_NS = 0.3;

  // Two TRPs give one TDoA, i.e. a hyperbola. Assume the UE is on the segment
  // between the two antennas (corridor / walk test) and intersect it with that.
  // ponytail: 1D only; the general 2D solver below needs a third TRP.
  if (trp_pos.size() == 2) {
    auto const& a = trp_pos[0];
    auto const& b = trp_pos[1];
    double const L =
        std::hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);  // metres
    // dd = d_B - d_A, and d_A + d_B = L on the segment.
    double const dd = tdoa_ns[0] * SPEED_OF_LIGHT_NS;
    if (L < 1.0) {
      return nlohmann::json{
          {"error", "The two TRPs must be at least 1 m apart (check "
                    "geo_coordinates of both gNBs)"},
          {"trps", trps_json},
          {"tdoaNs", tdoa_ns}};
    }
    double const dA_raw = (L - dd) / 2.0;
    double const dA     = std::clamp(dA_raw, 0.0, L);
    double const f      = dA / L;
    nlohmann::json j;
    j["mode"]              = "baseline-1d";
    j["baselineM"]         = L;
    j["tdoaNs"]            = tdoa_ns;
    j["distDiffM"]         = dd;
    j["distFromFirstTrpM"] = dA;
    j["outsideBaseline"]   = dA_raw != dA;
    j["localLocationEstimate"]["shape"]      = "POINT";
    j["localLocationEstimate"]["point"]["x"] = a[0] + f * (b[0] - a[0]);
    j["localLocationEstimate"]["point"]["y"] = a[1] + f * (b[1] - a[1]);
    j["localLocationEstimate"]["point"]["z"] = a[2] + f * (b[2] - a[2]);
    j["trps"]                                = trps_json;
    j["rounds"]                              = nof_rounds;
    j["roundsRejected"]                      = this->rounds_rejected;
    j["tdoaNsRounds"]                        = tdoa_rounds_json;
    Logger::lmf_app().info(
        "location (baseline-1d): L: %.2fm tdoa: %.3fns dd: %.2fm -> %.2fm from "
        "first TRP%s",
        L, tdoa_ns[0], dd, dA, dA_raw != dA ? " (clamped)" : "");
    return j;
  }

  // A 2D fix needs at least 3 TRPs, i.e. 2 time differences.
  if (trp_pos.size() < 3 || tdoa_ns.size() < 2) {
    Logger::lmf_app().error(
        "location: need at least 3 TRPs with a measurement, got %zu TRPs and "
        "%zu TDoA values",
        trp_pos.size(), tdoa_ns.size());
    return nlohmann::json{
        {"error", "Not enough TRP measurements to compute a location"},
        {"trps", trps_json}};
  }

  // Convert TDoA values from nanoseconds to meters. One value per TDoA, not a
  // fixed 8-TRP assumption.
  std::vector<double> dd_estimated(tdoa_ns.size());
  for (size_t i = 0; i < tdoa_ns.size(); i++) {
    dd_estimated[i] = tdoa_ns[i] * SPEED_OF_LIGHT_NS;
  }

  // Debug output for dd_estimated values
  std::cout << "[pos_est] dd_estimated Values:" << std::endl;
  for (const auto& dd : dd_estimated) {
    std::cout << "dd_estimated: " << dd << " meters" << std::endl;
  }
  // Convert trp_pos to a C-style array
  double trp_pos_array[trp_pos.size()][3];
  for (size_t i = 0; i < trp_pos.size(); ++i) {
    for (size_t j = 0; j < 3; ++j) {
      trp_pos_array[i][j] = trp_pos[i][j];
    }
  }

  // Estimated position array
  double pos_est[2]     = {0.0, 0.0};
  int dd_estimated_size = static_cast<int>(dd_estimated.size());
  // Perform LLS to estimate position
  try {
    lls_estimation(
        trp_pos_array, trp_pos.size(), dd_estimated.data(), dd_estimated_size,
        pos_est);
    std::cout << "[pos_est] Estimated Position: x = " << pos_est[0]
              << ", y = " << pos_est[1] << std::endl;
  } catch (const std::exception& e) {
    Logger::lmf_app().error("Error in LLS estimation: %s", e.what());
    return nlohmann::json{
        {"error", "Failed to compute location due to LLS estimation error"}};
  }
  SupportedGADShapes supportedGADShapes;
  supportedGADShapes.setEnumValue(
      SupportedGADShapes_anyOf::eSupportedGADShapes_anyOf::POINT);

  UncertaintyEllipse uncertaintyEllipse;
  uncertaintyEllipse.setSemiMajor(0.0);
  uncertaintyEllipse.setSemiMinor(0.0);
  uncertaintyEllipse.setOrientationMajor(180);

  oai::_3gpp::model::GeographicalCoordinates geographicalCoordinates;
  geographicalCoordinates.setLat(0.0);
  geographicalCoordinates.setLon(0.0);

  GeographicArea geographicArea;
  geographicArea.setShape(supportedGADShapes);
  geographicArea.setPoint(geographicalCoordinates);
  geographicArea.setUncertaintyEllipse(uncertaintyEllipse);
  geographicArea.setConfidence(100);

  LocationData locationData;
  locationData.setLocationEstimate(geographicArea);

  nlohmann::json j;
  j["localLocationEstimate"]["shape"]                           = "POINT";
  j["localLocationEstimate"]["localOrigin"]["coordinateId"]     = "string";
  j["localLocationEstimate"]["localOrigin"]["point"]["lon"]     = 180;
  j["localLocationEstimate"]["localOrigin"]["point"]["lat"]     = 90;
  j["localLocationEstimate"]["point"]["x"]                      = pos_est[0];
  j["localLocationEstimate"]["point"]["y"]                      = pos_est[1];
  j["localLocationEstimate"]["point"]["z"]                      = 1.5;
  j["localLocationEstimate"]["uncertaintyEllipse"]["semiMajor"] = 0;
  j["localLocationEstimate"]["uncertaintyEllipse"]["semiMinor"] = 0;
  j["localLocationEstimate"]["uncertaintyEllipse"]["orientationMajor"] = 180;
  j["localLocationEstimate"]["confidence"]                             = 100;
  j["mode"]                                                            = "lls-2d";
  j["tdoaNs"]                                                          = tdoa_ns;
  j["trps"]                                                            = trps_json;
  j["rounds"]                                                          = nof_rounds;
  j["roundsRejected"]                                                  = this->rounds_rejected;
  j["tdoaNsRounds"]                                                    = tdoa_rounds_json;

  return j;  // locationData;
}

//------------------------------------------------------------------------------
// LPP over N1 (TS 23.273 6.11.1, TS 29.518 5.2.2.3.1 / 5.2.2.3.5, TS 37.355)
//------------------------------------------------------------------------------
void LocationDetermination::n1_lpp_message_transfer(std::string const& lpp_pdu) {
  std::string amf_uri = {};
  lmf_sbi_helper::get_amf_comm_n1n2_message_transfer_uri(
      lmf_cfg.amf_addr, this->supi, amf_uri);

  // n1MessageContainer {n1MessageClass LPP, n1MessageContent -> the binary part, nfId = this LMF}; nfId is
  // mandatory for LPP (TS 29.518 6.1.6.2.17) and must equal the subscription's, which is how the AMF
  // routes the UE's answer back (TS 24.501 5.4.5.2.3 c).
  N1MessageClass n1MessageClass = {};
  n1MessageClass.setEnumValue(N1MessageClass_anyOf::eN1MessageClass_anyOf::LPP);
  RefToBinaryData n1MessageContent = {};
  n1MessageContent.setContentId(N1_LPP_CONTENT_ID);
  N1MessageContainer n1MessageContainer = {};
  n1MessageContainer.setN1MessageClass(n1MessageClass);
  n1MessageContainer.setN1MessageContent(n1MessageContent);
  n1MessageContainer.setNfId(lmf_nrf_inst->lmf_instance_id);

  N1N2MessageTransferReqData req = {};
  req.setN1MessageContainer(n1MessageContainer);
  // The Session ID of the transfer is the LCS correlation ID (TS 23.273 6.11.1 step 1); the AMF puts it in
  // the DL NAS TRANSPORT's Additional information (TS 24.501 5.4.5.3.2 c).
  req.setLcsCorrelationId(this->lcs_correlation_id);

  nlohmann::json req_json;
  to_json(req_json, req);
  std::string lpp_hex = {};
  oai::utils::conv::convert_string_2_hex(lpp_pdu, lpp_hex);
  std::string body = {};
  mime_parser::create_multipart_related_content(
      body, req_json.dump(), CURL_MIME_BOUNDARY, lpp_hex,
      multipart_related_content_part_e::NAS);

  oai::http::request http_request =
      http_client_inst->prepare_multipart_request(amf_uri, body);
  auto http_response = http_client_inst->send_http_request(
      oai::common::sbi::method_e::POST, http_request);
  Logger::lmf_app().info(
      "LPP N1N2MessageTransfer (correlation %s): HTTP %d %s",
      this->lcs_correlation_id, (int) http_response.status_code,
      http_response.body);

  // 200 {cause N1_N2_TRANSFER_INITIATED} when transferred (TS 29.518 5.2.2.3.1.1); 403
  // UE_WITHOUT_N1_LPP_SUPPORT, 400 and the rest are failures.
  nlohmann::json rsp = nlohmann::json::parse(http_response.body, nullptr, false);
  if (http_response.status_code != oai::common::sbi::http_status_code::OK ||
      rsp.is_discarded() || !rsp.contains("cause") ||
      rsp["cause"] !=
          n1_n2_message_transfer_cause_e2str[N1_N2_TRANSFER_INITIATED]) {
    throwHttpError(
        "LPP N1N2MessageTransfer failed"s,
        "HTTP "s + std::to_string((int) http_response.status_code) + " "s +
            http_response.body);
  }
}

//------------------------------------------------------------------------------
nlohmann::json LocationDetermination::lpp_capability_transfer() {
  long transaction = 0;
  long sequence    = 0;
  std::future<std::string> uplink;
  {
    std::scoped_lock lk{m_lpp};
    transaction = lpp_transaction_number;
    lpp_transaction_number = (lpp_transaction_number + 1) % 256;
    sequence = lpp_dl_sequence_number;
    lpp_dl_sequence_number = (lpp_dl_sequence_number + 1) % 256;
    lpp_uplink.emplace();
    uplink = lpp_uplink->get_future();
  }

  auto const pdu = lpp::encode_request_capabilities(transaction, sequence);
  if (pdu.empty()) throwHttpError("LPP"s, "RequestCapabilities encode failed"s);
  Logger::lmf_app().info(
      "LPP RequestCapabilities -> %s: transaction %ld, sequence %ld",
      this->supi, transaction, sequence);
  this->n1_lpp_message_transfer(pdu);

  if (uplink.wait_for(lmf_cfg.positioning_wait_ms) != std::future_status::ready) {
    std::scoped_lock lk{m_lpp};
    lpp_uplink.reset();
    throwHttpError(
        "LPP"s, "no ProvideCapabilities within "s +
                    std::to_string(lmf_cfg.positioning_wait_ms.count()) + "ms"s);
  }
  std::string error;
  auto const reply = lpp::decode(uplink.get(), error);
  if (!reply) throwHttpError("LPP"s, "uplink undecodable: "s + error);
  Logger::lmf_app().info("LPP uplink from %s:\n%s", this->supi, reply->xer);

  // 37.355 5.1.1 step 2 / 5.1.3: a ProvideCapabilities, same LPP-TransactionID, endTransaction TRUE.
  if (reply->body != "provideCapabilities")
    throwHttpError("LPP"s, "expected provideCapabilities, got "s + reply->body);
  if (reply->transaction_initiator != 0 /* locationServer */ ||
      reply->transaction_number != transaction)
    throwHttpError("LPP"s, "ProvideCapabilities does not carry our transaction ID"s);
  if (!reply->end_transaction)
    throwHttpError("LPP"s, "ProvideCapabilities without endTransaction TRUE"s);

  return {
      {"transactionNumber", transaction},
      {"nrMultiRttCapable", reply->nr_multi_rtt_capable},
      {"nrNtnMeasAndReport", reply->nr_ntn_meas_and_report},
      {"lcsCorrelationId", this->lcs_correlation_id}};
}

//------------------------------------------------------------------------------
std::vector<std::tuple<GnbId, long, long>> LocationDetermination::lpp_provide_assistance_data(
    std::map<GnbId, Gnb> const& gnbs) {
  // Every TRP that reported a PRS Configuration, the first as the reference TRP. In NTN each satellite carries
  // its own TRP (TS 38.305 5.4.2), so a UE under two satellites gets two, and the measurements it returns for
  // them are what locates it at one instant instead of over a pass.
  // ponytail: the reference is the first TRP in gnbId order, which in this testbed is the serving cell. Pick
  // it by the gNB that answers the NRPPa measurements if a deployment ever orders them differently.
  std::vector<lpp::dl_prs_assistance> trps;
  std::vector<std::pair<GnbId, long>> ids;
  for (auto const& [gnbId, gnb] : gnbs) {
    for (auto const& [trpId, trp] : gnb.trp) {
      if (!trp.prs) continue;
      // dl-PRS-ID is this LMF's name for the TRP in this session (37.355 6.4.3).
      // nr-ARFCN is that of the TRP's CD-SSB (37.355 6.4.3), while the TRP Information NR ARFCN is the carrier's
      // Point A (38.473 9.3.1.x, OCUDU reports the UL one): not the same frequency, so it is left out and the
      // dl-PRS-ID and PCI identify the TRP.
      lpp::dl_prs_assistance a{static_cast<long>(trps.size()), trp.pci, std::nullopt, 0, 0, *trp.prs};
      trps.push_back(a);
      ids.emplace_back(gnbId, trpId);
    }
  }
  if (trps.empty()) return {};
  // nr-DL-PRS-SFN0-Offset: how far each neighbour's SFN 0 sits behind the reference TRP's, in frames and
  // subframes (37.355 6.4.3). Both come from the TRPs' SFN Initialisation Times (TS 38.455 9.2.x).
  auto const sfn0_of = [&gnbs](GnbId g, long t) -> std::optional<double> {
    auto const gi = gnbs.find(g);
    if (gi == gnbs.end()) return std::nullopt;
    auto const ti = gi->second.trp.find(t);
    return ti == gi->second.trp.end() ? std::nullopt : ti->second.sfn0_unix;
  };
  auto const ref_sfn0 = sfn0_of(ids[0].first, ids[0].second);
  for (size_t i = 1; i < trps.size(); ++i) {
    auto const s = sfn0_of(ids[i].first, ids[i].second);
    if (!ref_sfn0 || !s) continue;
    // 37.355 6.4.3: "the time offset of the SFN#0 slot#0 for the given TRP with respect to the SFN#0 slot#0 of
    // the assistance data reference TRP", i.e. t_j - t_ref, over the 10.24 s SFN cycle. It is what tells the
    // UE - which counts frames on the serving cell's grid - where in its own timeline this TRP's PRS occasions
    // fall. Without it the UE would look for them in the serving cell's slot and find nothing.
    long const ms       = static_cast<long>(std::llround((*s - *ref_sfn0) * 1e3));
    long const positive = (ms % 10240 + 10240) % 10240;
    trps[i].sfn_offset      = positive / 10;
    trps[i].subframe_offset = positive % 10;
  }

  long transaction = 0, sequence = 0;
  {
    std::scoped_lock lk{m_lpp};
    transaction            = lpp_transaction_number;
    lpp_transaction_number = (lpp_transaction_number + 1) % 256;
    sequence               = lpp_dl_sequence_number;
    lpp_dl_sequence_number = (lpp_dl_sequence_number + 1) % 256;
  }
  auto const pdu = lpp::encode_provide_assistance_data(transaction, sequence, trps);
  if (pdu.empty()) {
    Logger::lmf_app().warn(
        "LPP assistance data: %zu TRP(s) not expressible as one frequency layer (scs %ld, period %ld)",
        trps.size(), trps[0].prs.scs, trps[0].prs.period);
    return {};
  }
  this->n1_lpp_message_transfer(pdu);
  std::vector<std::tuple<GnbId, long, long>> assigned;
  this->gnb_of_dl_prs_id.clear();
  for (size_t i = 0; i != trps.size(); ++i) {
    auto const& a = trps[i];
    assigned.emplace_back(ids[i].first, ids[i].second, a.dl_prs_id);
    this->gnb_of_dl_prs_id[a.dl_prs_id] = ids[i].first;
    Logger::lmf_app().info(
        "LPP ProvideAssistanceData -> %s: dl-PRS-ID %ld = gnbId 0x%x trpId %d%s, PCI %ld, ARFCN %ld, point A %ld, "
        "start PRB %ld, bw %ld, comb idx %ld, period idx %ld offset %ld, symbols idx %ld from %ld, seq ID %ld, "
        "SFN0 offset %ld frames %ld subframes",
        this->supi, a.dl_prs_id, ids[i].first, ids[i].second, i ? "" : " (reference)", a.pci.value_or(-1),
        a.arfcn.value_or(-1), a.prs.point_a, a.prs.start_prb, a.prs.bandwidth, a.prs.comb, a.prs.period,
        a.prs.set_slot_offset, a.prs.nof_symbols, a.prs.symbol_offset, a.prs.sequence_id, a.sfn_offset,
        a.subframe_offset);
  }
  return assigned;
}

//------------------------------------------------------------------------------
nlohmann::json LocationDetermination::lpp_multi_rtt_round(bool ntn) {
  long transaction = 0;
  long sequence    = 0;
  std::future<std::string> uplink;
  {
    std::scoped_lock lk{m_lpp};
    transaction            = lpp_transaction_number;
    lpp_transaction_number = (lpp_transaction_number + 1) % 256;
    sequence               = lpp_dl_sequence_number;
    lpp_dl_sequence_number = (lpp_dl_sequence_number + 1) % 256;
    lpp_uplink.emplace();
    uplink = lpp_uplink->get_future();
  }
  // k0 (1 Tc) asks for full resolution; the UE may report coarser (37.355 6.5.12.5).
  auto const pdu = lpp::encode_request_location_information(transaction, sequence, 0, ntn);
  if (pdu.empty()) throwHttpError("LPP"s, "RequestLocationInformation encode failed"s);
  this->n1_lpp_message_transfer(pdu);
  if (uplink.wait_for(lmf_cfg.positioning_wait_ms) != std::future_status::ready) {
    std::scoped_lock lk{m_lpp};
    lpp_uplink.reset();
    throwHttpError("LPP"s, "no ProvideLocationInformation within "s +
                               std::to_string(lmf_cfg.positioning_wait_ms.count()) + "ms"s);
  }
  std::string error;
  auto const reply = lpp::decode(uplink.get(), error);
  if (!reply) throwHttpError("LPP"s, "uplink undecodable: "s + error);
  // 37.355 5.3.1 step 2: ProvideLocationInformation, same transaction, endTransaction TRUE.
  if (reply->body != "provideLocationInformation")
    throwHttpError("LPP"s, "expected provideLocationInformation, got "s + reply->body);
  if (reply->transaction_initiator != 0 || reply->transaction_number != transaction ||
      !reply->end_transaction)
    throwHttpError("LPP"s, "ProvideLocationInformation not closing our transaction"s);
  if (reply->multi_rtt_error)
    throwHttpError("LPP"s, "UE: nr-Multi-RTT-Error target device cause "s +
                               std::to_string(*reply->multi_rtt_error));
  if (reply->location_error)
    throwHttpError("LPP"s, "UE: locationError cause "s + std::to_string(*reply->location_error));
  if (reply->multi_rtt.empty()) throwHttpError("LPP"s, "no NR Multi-RTT measurement"s);
  // The reference TRP is the one the LMF named first in the assistance data, and the one the gNB Rx-Tx below
  // belongs to. The rest are neighbour TRPs - in NTN, the other satellites over this UE.
  auto const& u = reply->multi_rtt.front();
  if (ntn && !u.subframe_offset)
    throwHttpError("LPP"s, "NTN measurements requested but not provided"s);

  // TS 38.305 8.10: RTT = UE Rx-Tx + gNB Rx-Tx. In NTN the UE Rx-Tx proper is within +-0.5 ms of the UL
  // subframe closest to the DL one, so the UE's whole timing is offset x 1 ms + Rx-Tx (TS 38.215 5.1.46).
  constexpr double tc_per_us = 480000.0 * 4096 / 1e6;
  double const ue_us = (u.subframe_offset.value_or(0) * 1000.0 * tc_per_us + u.rxtx_tc) / tc_per_us;
  nlohmann::json j = {
      {"ueRxTxUs", ue_us},
      {"ueRxTxK", u.k},
      {"ueRxTxReported", u.reported},
      {"subframeOffset", u.subframe_offset.value_or(0)},
      {"dlTimingDrift01ppm", u.dl_drift_01ppm.value_or(0)},
      {"ueSfn", u.sfn},
      {"ueSlot", u.slot},
      {"timingQualityM", u.timing_quality_m},
      {"pci", u.pci.value_or(-1)},
      {"dlPrsId", u.dl_prs_id}};
  // Every measured TRP, the reference included, in the order the UE reported them. Their downlink arrivals
  // differ by their transmit times plus their ranges, and the LMF knows the transmit times (each TRP's SFN
  // Initialisation Time), so each one is a range of its own once the reference round trip has fixed the
  // UE's clock - see lmf_app::ntn_record.
  nlohmann::json trps = nlohmann::json::array();
  for (auto const& t : reply->multi_rtt) {
    double const t_us = (t.subframe_offset.value_or(0) * 1000.0 * tc_per_us + t.rxtx_tc) / tc_per_us;
    nlohmann::json e = {{"dlPrsId", t.dl_prs_id},
                        {"ueRxTxUs", t_us},
                        {"subframeOffset", t.subframe_offset.value_or(0)},
                        {"dlTimingDrift01ppm", t.dl_drift_01ppm.value_or(0)},
                        {"sfn", t.sfn},
                        {"slot", t.slot},
                        {"timingQualityM", t.timing_quality_m},
                        {"pci", t.pci.value_or(-1)}};
    // TS 38.305 8.10.3 step 13: where this TRP's own gNB also measured the UE's SRS on the same frame, the two
    // halves close a round trip of their own and that TRP gets an independent range, not one differenced
    // against the reference. In NTN a neighbour satellite usually cannot - the UE's uplink is pre-compensated
    // for the serving satellite - so this is present only when it managed to.
    auto const g = this->gnb_of_dl_prs_id.find(t.dl_prs_id);
    if (g != this->gnb_of_dl_prs_id.end()) {
      auto const r = this->gnb_rxtx.find(g->second);
      if (r != this->gnb_rxtx.end() && std::get<0>(r->second) == t.sfn) {
        e["gnbRxTxUs"] = std::get<2>(r->second) / 1e3;
        e["gnbSlot"]   = std::get<1>(r->second);
        e["rttUs"]     = t_us + std::get<2>(r->second) / 1e3;
      }
    }
    trps.push_back(e);
  }
  j["trps"] = trps;
  // The REFERENCE TRP's half, not whichever gNB answered last. With two gNBs the map is walked in gnbId
  // order, so last_gnb_rxtx ended up holding the NEIGHBOUR's - in its own frame grid, and in NTN usually a
  // zero measurement, because a neighbour satellite cannot hear an uplink pre-compensated for another one.
  // Pairing the UE's half against that never matched on SFN, so every round came back unpaired and the
  // solver got nothing.
  auto const ref_gnb = this->gnb_of_dl_prs_id.find(0);
  auto const ref_rxtx = ref_gnb == this->gnb_of_dl_prs_id.end() ? this->gnb_rxtx.end()
                                                                : this->gnb_rxtx.find(ref_gnb->second);
  if (ref_rxtx == this->gnb_rxtx.end() && !this->last_gnb_rxtx)
    throwHttpError("Multi-RTT"s, "no gNB Rx-Tx to pair with"s);
  auto const [gsfn, gslot, gnb_ns] =
      ref_rxtx != this->gnb_rxtx.end() ? ref_rxtx->second : *this->last_gnb_rxtx;
  j["gnbRxTxUs"] = gnb_ns / 1e3;
  j["gnbSfn"]    = gsfn;
  j["gnbSlot"]   = gslot;
  // One frame, PRS and SRS a few slots apart: the pair describes one instant to within the delay drift over
  // those slots (46 us/s x 5 ms = 0.23 us at the LEO horizon). Anything else is not one round trip.
  if (u.sfn != gsfn) {
    Logger::lmf_app().warn(
        "Multi-RTT: UE Rx-Tx from sfn %ld, gNB Rx-Tx from sfn %ld - not one instant, not paired",
        u.sfn, gsfn);
    j["paired"] = false;
    return j;
  }
  double const rtt_us   = ue_us + gnb_ns / 1e3;
  double const range_km = rtt_us * 1e-6 * 299792458.0 / 2 / 1e3;
  j["paired"]  = true;
  j["rttUs"]   = rtt_us;
  j["rangeKm"] = range_km;
  Logger::lmf_app().info(
      "Multi-RTT: UE Rx-Tx %.3f us (offset %ld ms %+.1f ns, k%d, drift %ld x0.1 ppm) + gNB Rx-Tx %+.3f us "
      "at sfn %ld (PRS slot %ld, SRS slot %ld) = RTT %.3f us -> %.3f km",
      ue_us, u.subframe_offset.value_or(0), u.rxtx_tc / tc_per_us * 1e3, u.k,
      u.dl_drift_01ppm.value_or(0), gnb_ns / 1e3, gsfn, u.slot, gslot, rtt_us, range_km);
  this->multi_rtt_rounds.push_back(j);
  return j;
}

//------------------------------------------------------------------------------
void LocationDetermination::handle_lpp_uplink(
    std::string const& correlation_id, std::string const& pdu) {
  if (correlation_id != this->lcs_correlation_id) {
    throwHttpError(
        "LPP"s,
        "uplink for correlation ID "s + correlation_id + ", session is "s +
            this->lcs_correlation_id,
        Pistache::Http::Code::Forbidden);
  }
  std::string error;
  auto const msg = lpp::decode(pdu, error);
  if (!msg) {
    // Still deliver: the waiting procedure reports the decode failure with context.
    Logger::lmf_app().warn("LPP uplink from %s undecodable: %s", this->supi, error);
  }
  std::scoped_lock lk{m_lpp};
  // 37.355 4.3.2: a message carrying the same sequence number as the last one received in this session is
  // a duplicate and shall be discarded.
  if (msg && msg->sequence_number) {
    if (lpp_ul_last_sequence_number == msg->sequence_number) {
      Logger::lmf_app().info(
          "LPP uplink from %s: duplicate sequence number %ld discarded",
          this->supi, *msg->sequence_number);
      return;
    }
    lpp_ul_last_sequence_number = msg->sequence_number;
  }
  if (!lpp_uplink) {
    Logger::lmf_app().warn("LPP uplink from %s with no request pending", this->supi);
    return;
  }
  lpp_uplink->set_value(pdu);
  lpp_uplink.reset();
}
