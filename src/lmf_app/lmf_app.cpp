/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#include "lmf_app.hpp"

#include <unistd.h>

#include <boost/format.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/irange.hpp>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <iostream>
#include <iterator>
#include <optional>
#include <string>
#include <thread>

#include "3gpp_29.500.h"
#include "3gpp_29.518.h"
#include "conversions.hpp"
#include "lmf_nrf.hpp"
#include "logger.hpp"
#include "mime_parser.hpp"
// model
#include "GlobalRanNodeId.h"
#include "LocationData.h"
#include "N1MessageClass.h"
#include "N1MessageContainer.h"
#include "N1N2MessageTransferReqData.h"
#include "N1N2MessageTransferRspData.h"
#include "N2InformationNotification.h"
#include "N2InformationTransferReqData.h"
#include "ProblemDetails.h"
#include "RefToBinaryData.h"
#include "UeN1N2InfoSubscriptionCreateData.h"
#include "UeN1N2InfoSubscriptionCreatedData.h"
// nrppa
#include "InitiatingMessage.h"
#include "ProtocolIE-Field.h"
#include "SuccessfulOutcome.h"
#include "NG-RANAccessPointPosition.h"
#include "TRP-MeasurementResponseItem.h"
#include "TRPInformationTypeResponseItem.h"
#include "PRSConfiguration.h"
#include "PRSResourceSet-Item.h"
#include "PRSResource-Item.h"
#include "TRPInformationTypeListTRPReq.h"
#include "TRPInformationTypeItem.h"
#include "TRPPositionDirect.h"
#include "TRPPositionDirectAccuracy.h"
#include "NGRANHighAccuracyAccessPointPosition.h"
#include "TRPItem.h"
#include "TrpMeasuredResultsValue.h"
#include "TrpMeasurementResultItem.h"
#include "UL-RTOAMeasurement.h"
#include "ULRTOAMeas.h"
#include "UnsuccessfulOutcome.h"
// do not include model GeographicalCoordinates.h
#include "../nrppa/GeographicalCoordinates.h"
#include "CoordinateID.h"
#include "TRPPositionDefinitionType.h"
#include "TRPPositionReferenced.h"
#include "lmf_sbi_helper.hpp"

using namespace std;
using namespace oai::lmf::app;
using namespace oai::_3gpp::model;
using namespace oai::lmf::config;
using namespace std::chrono_literals;

lmf_nrf* lmf_nrf_inst = nullptr;

// provides for asn container.list.array range based for loops
// for (auto const& xyzIE : xyzResponse.protocolIEs) {
template<typename T>
auto begin(T const& container) {
  return container.list.array;
}

template<typename T>
auto end(T const& container) {
  return container.list.array + container.list.count;
}

//------------------------------------------------------------------------------
static unsigned env_uint(char const* name, unsigned fallback) {
  char const* v = std::getenv(name);
  return (v && *v) ? static_cast<unsigned>(std::strtoul(v, nullptr, 10)) :
                     fallback;
}

lmf_app::lmf_app(const std::string& config_file, lmf_event& ev)
    : event_sub(ev) {}

//------------------------------------------------------------------------------
lmf_app::~lmf_app() {
  if (lmf_nrf_inst) {
    delete lmf_nrf_inst;
    lmf_nrf_inst = nullptr;
  }
  Logger::lmf_app().debug("Delete LMF_APP instance...");
}

//------------------------------------------------------------------------------
bool lmf_app::start() {
  Logger::lmf_app().startup("Starting...");
  // Create NRF instance and register to NRF if needed
  if (lmf_cfg.register_nrf) {
    try {
      lmf_nrf_inst = new lmf_nrf(event_sub);
      Logger::lmf_app().info("NRF TASK Created ");
      // Register to NRF
      lmf_nrf_inst->register_to_nrf();
    } catch (std::exception& e) {
      Logger::lmf_app().error("Cannot create NRF TASK: %s", e.what());
      return false;
    }
  }
  Logger::lmf_app().startup("Started");

  // Periodic positioning without an external client: LMF_AUTO_SUPI="imsi-001010000000003[,imsi-...]"  UEs
  // to locate LMF_AUTO_INTERVAL_S=5                            pause between fixes Each result is logged
  // ("auto position") and appended as one JSON line to LMF_AUTO_OUTPUT (default /openair-
  // lmf/positions/positions.jsonl).
  if (char const* supis = std::getenv("LMF_AUTO_SUPI"); supis && *supis) {
    auto_running = true;
    auto_thread  = std::thread([this, list = std::string(supis)] {
      unsigned const interval_s = env_uint("LMF_AUTO_INTERVAL_S", 5);
      char const* out_env       = std::getenv("LMF_AUTO_OUTPUT");
      std::string const out     = (out_env && *out_env) ?
                                      out_env :
                                      "/openair-lmf/positions/positions.jsonl";
      Logger::lmf_app().info(
          "auto positioning: supi(s) %s every %us -> %s", list, interval_s,
          out);
      while (auto_running) {
        std::stringstream ss(list);
        for (std::string supi; std::getline(ss, supi, ',') && auto_running;) {
          nlohmann::json j;
          Pistache::Http::Code code = {};
          try {
            InputData input;
            input.setSupi(supi);
            this->handle_determine_location(input, j, code);
          } catch (std::exception const& e) {
            j = nlohmann::json{{"error", e.what()}};
          }
          // Same cleanup as the HTTP handler does on failure.
          this->release_all_n1n2subscriptions();
          this->release_non_ue_subscription();

          j["supi"] = supi;
          j["time"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
          Logger::lmf_app().info("auto position: %s", j.dump());
          if (std::ofstream f{out, std::ios::app}; f) f << j.dump() << "\n";
        }
        for (unsigned i = 0; i < interval_s * 10 && auto_running; ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    });
  }
  return true;
}

//------------------------------------------------------------------------------
void lmf_app::stop() {
  auto_running = false;
  if (auto_thread.joinable()) auto_thread.join();
  if (lmf_nrf_inst and lmf_cfg.register_nrf) {
    lmf_nrf_inst->deregister_to_nrf();
    delete lmf_nrf_inst;
    lmf_nrf_inst = nullptr;
  }
}

//------------------------------------------------------------------------------
void lmf_app::trp_information_request(
    std::shared_ptr<LocationDetermination> const& ctx,
    NRPPATransactionID_t const& nrppatransactionID) {
  Logger::lmf_app().info("TRP information request");

  auto initiatingMessage =
      (InitiatingMessage_t*) malloc(sizeof(InitiatingMessage_t));
  *initiatingMessage = InitiatingMessage_t{
      .procedureCode      = ProcedureCode_id_tRPInformationExchange,
      .criticality        = Criticality_reject,
      .nrppatransactionID = nrppatransactionID,
      .value =
          {
              .present = InitiatingMessage__value_PR_TRPInformationRequest,
              .choice =
                  {
                      .TRPInformationRequest = TRPInformationRequest_t{},
                  },
          },
  };

  // TRP Information Type List is mandatory (TS 38.455 9.1.2.1); ask for what
  // handle_trp_information_response() consumes.
  auto trpInformationTypeListIe = (TRPInformationRequest_IEs_t*) calloc(
      1, sizeof(TRPInformationRequest_IEs_t));
  trpInformationTypeListIe->id = ProtocolIE_ID_id_TRPInformationTypeListTRPReq;
  trpInformationTypeListIe->criticality = Criticality_reject;
  trpInformationTypeListIe->value.present =
      TRPInformationRequest_IEs__value_PR_TRPInformationTypeListTRPReq;
  for (auto const type :
       {TRPInformationTypeItem_nrPCI, TRPInformationTypeItem_nG_RAN_CGI,
        TRPInformationTypeItem_arfcn, TRPInformationTypeItem_geoCoord,
        // for NR-DL-PRS-AssistanceData, 37.355 6.4.3 (TS 38.305 8.10.2.1: the LMF obtains it from the gNB)
        TRPInformationTypeItem_pRSConfig,
        // SFN Initialisation Time: turns measurement (SFN, slot) timestamps into absolute time, needed to know
        // where the satellite was for each round trip (NTN, TS 38.305 8.10)
        TRPInformationTypeItem_sFNInitTime}) {
    auto item = (TRPInformationTypeItemTRPReq_t*) calloc(
        1, sizeof(TRPInformationTypeItemTRPReq_t));
    item->id          = ProtocolIE_ID_id_TRPInformationTypeItem;
    item->criticality = Criticality_reject;
    item->value.present =
        TRPInformationTypeItemTRPReq__value_PR_TRPInformationTypeItem;
    item->value.choice.TRPInformationTypeItem = type;
    ASN_SEQUENCE_ADD(
        &trpInformationTypeListIe->value.choice.TRPInformationTypeListTRPReq
             .list,
        item);
  }
  ASN_SEQUENCE_ADD(
      &initiatingMessage->value.choice.TRPInformationRequest.protocolIEs.list,
      trpInformationTypeListIe);

  auto nrppaPdu = (NRPPA_PDU_t*) malloc(sizeof(NRPPA_PDU_t));
  *nrppaPdu     = NRPPA_PDU_t{
      .present = NRPPA_PDU_PR_initiatingMessage,
      .choice =
          {
              .initiatingMessage = initiatingMessage,
          },
  };

  ctx->non_ue_n2_message_transfer(
      share_nrppa_pdu(nrppaPdu), nrppatransactionID,
      ProcedureCode_id_tRPInformationExchange, {}, nullptr);
}

//------------------------------------------------------------------------------
void lmf_app::trp_information(
    std::shared_ptr<LocationDetermination> const& ctx) {
  std::unique_lock lk{this->cv_m_gnb};

  // Always re-query: a gNB (e.g.
  this->gnb.clear();

  auto const& tId = this->nrppa_tid_gen.get_uid();
  this->trp_information_request(ctx, tId);

  // lmf_cfg.determine_num_gnb -> return false to wait until trp_info_wait_ms
  this->trp_info_err.clear();
  auto const& pred = [&gnb = this->gnb, &err = this->trp_info_err] {
    if (lmf_cfg.determine_num_gnb) return false;  // wait for timeout
    return (gnb.size() + err.size()) ==
           lmf_cfg.num_gnb;  // stop if cfg'd gnb's recvd
  };
  Logger::lmf_app().debug(
      "trp information request: wait %dms for %s gnb responses",
      lmf_cfg.trp_info_wait_ms.count(),
      lmf_cfg.determine_num_gnb ? "until timeout" :
                                  std::to_string(lmf_cfg.num_gnb));
  auto const& rc = this->cv_gnb.wait_for(lk, lmf_cfg.trp_info_wait_ms, pred);
  {
    std::scoped_lock lk{ctx->m_tId};
    ctx->nrppa_tId.erase(tId);
  }
  this->nrppa_tid_gen.free_uid(tId);
  this->erase_nrppaTxnId2Supi(tId);
  if (this->trp_info_err.size() > 0) {
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "trp information failure",
        "gnb err count: "s + std::to_string(this->trp_info_err.size()));
  }
  if (!lmf_cfg.determine_num_gnb &&
      (!rc || this->gnb.size() < lmf_cfg.num_gnb)) {
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "trp information request",
        "timeout after "s + std::to_string(lmf_cfg.trp_info_wait_ms.count()) +
            "ms waiting for "s + std::to_string(lmf_cfg.num_gnb) +
            " gnb responses, but received: "s +
            std::to_string(this->gnb.size()));
  }
  Logger::lmf_app().debug(
      "trp information request: received %d gnb responses", this->gnb.size());

  if (this->gnb.size() == 0) {
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "trp information request"s, "no gnbs available"s);
  }
  if (this->numTrps() == 0) {
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "trp information request"s, "no trp's available"s);
  }
}

//------------------------------------------------------------------------------
void lmf_app::handle_determine_location(
    const InputData& inputData, nlohmann::json& json_data,
    Pistache::Http::Code& code) {
  auto const& supi = inputData.getSupi();

  auto const& ctx = this->create_lmf_context(supi);
  if (!ctx) {
    auto const& err =
        "Could not create context for supi '"s + supi + "': already exist"s;
    Logger::lmf_app().warn(err);
    ProblemDetails problemDetails;
    problemDetails.setCause("INTERNAL_SERVER_ERROR");
    problemDetails.setStatus(
        oai::common::sbi::http_status_code::INTERNAL_SERVER_ERROR);
    problemDetails.setDetail(err);

    json_data = problemDetails;
    code      = Pistache::Http::Code(problemDetails.getStatus());

    return;
  }

  // LCS correlation ID of this location session: the AMF's, when it sent one in
  // Nlmf_Location_DetermineLocation (TS 23.273 6.1.1 step 7, InputData.correlationID in TS 29.572);
  // otherwise this LMF assigns one, from its own "lmf-" range so the two kinds stay distinguishable (TS
  // 24.501 5.4.5.3.2 NOTE 2, TS 23.273 6.3.1 NOTE 11).
  ctx->lcs_correlation_id =
      inputData.correlationIDIsSet() && !inputData.getCorrelationID().empty() ?
          inputData.getCorrelationID() :
          "lmf-"s + std::to_string(++this->lmf_correlation_counter);

  this->create_non_ue_subscription();
  this->trp_information(ctx);
  this->create_n1n2subscription(supi);

  // LPP Capability Transfer (TS 37.355 5.1.1) over N1. Phase 1 of the Multi-RTT work: it proves the LPP path
  // end to end and must not take the working NRPPa positioning below down with it, so a failure is logged.
  bool multi_rtt = false, ntn = false;
  try {
    auto const lpp = ctx->lpp_capability_transfer();
    Logger::lmf_app().info("LPP capability transfer: %s", lpp.dump());
    multi_rtt = lpp.value("nrMultiRttCapable", false);
    ntn       = lpp.value("nrNtnMeasAndReport", false);
    // TS 38.305 8.10.2.1 step 3: the DL-PRS the UE is to measure, before it is asked to measure.
    if (multi_rtt) {
      auto const assigned = ctx->lpp_provide_assistance_data(this->gnb);
      if (assigned.empty()) {
        Logger::lmf_app().warn("LPP assistance data: no TRP reported a PRS Configuration");
      } else {
        std::scoped_lock lk{this->cv_m_gnb};
        for (auto const& [gnbId, trpId, id] : assigned) {  // Gnb has no default constructor: find, never [].
          auto const g = this->gnb.find(gnbId);
          if (g == this->gnb.end()) continue;
          auto const t = g->second.trp.find(trpId);
          if (t != g->second.trp.end()) t->second.dl_prs_id = id;
        }
      }
    }
  } catch (std::exception const& e) {
    Logger::lmf_app().warn("LPP capability transfer failed: %s", e.what());
  }

  auto const& res = ctx->positioning_information_request();
  if (std::holds_alternative<CauseError>(res)) {
    auto const& err = std::get<CauseError>(res);
    ctx->throwHttpError("positioning infromation request failure", err.msg());
  }
  auto const& [nrppaPduPIR, ueSrsConfiguration] =
      std::get<LocationDetermination::pos_info_succ>(res);
  // nrppaPduPIR contain position information
  // POSITIONING INFORMATION RESPONSE ( 9.1.1.11 NRPPa TS 38.455 )
  // xer_fprint(stdout, &asn_DEF_NRPPA_PDU, nrppaPduPIR.get());

  // 5. NRPPa Request UE SRS activation
  // 9.1.1.17 POSITIONING ACTIVATION REQUEST
  // Skipped: activation only starts semi-persistent/aperiodic SRS. When the
  // gNB measures the UE's periodic SRS there is nothing to activate, and gNBs
  // that do not implement the F1AP Positioning Activation procedure (e.g.
  // OCUDU, whose DU has no handler for it) let the request time out, which
  // would abort this whole procedure. Same reason the deactivation call below
  // is left out.
  // ctx->positioning_activation_request();

  // Measure all gNBs together, several times; compute_location() takes the
  // median. LMF_MEAS_ROUNDS (default 5) rounds are kept; rounds where the
  // gNBs did not measure the same SRS occasion are dropped and retried.
  unsigned const rounds_wanted = env_uint("LMF_MEAS_ROUNDS", 5);
  unsigned accepted            = 0;
  for (unsigned attempt = 0;
       accepted < rounds_wanted && attempt < 3 * rounds_wanted; ++attempt) {
    if (ctx->measurement_round(this->gnb, ueSrsConfiguration)) {
      ++accepted;
      // NR Multi-RTT (TS 38.305 8.10): the UE's half of the round trip for this round, over LPP. Its
      // failures are logged, never allowed to break the NRPPa positioning around it.
      if (multi_rtt) {
        try {
          auto round = ctx->lpp_multi_rtt_round(ntn);
          if (round.value("paired", false)) this->ntn_record(supi, round);
        } catch (std::exception const& e) {
          Logger::lmf_app().warn("Multi-RTT round failed: %s", e.what());
        }
      }
    }
  }
  if (accepted == 0) {
    ctx->throwHttpError(
        "measurement"s, "no round where all TRPs measured the same SRS "
                        "occasion (rejected: "s +
                            std::to_string(ctx->rounds_rejected) + ")"s);
  }

  // In positioning_deactivation()
  // openairinterface5g/openair2/LAYER2/NR_MAC_gNB/mac_rrc_dl_handler.c:792
  // Not Implemented
  // ctx->positioning_deactivation_request();

  code                = Pistache::Http::Code::Ok;
  // NTN: the UE location from the round trips to one satellite at different instants (TS 38.305 8.10), solved
  // over this UE's history before the terrestrial solver below, which needs several TRPs and throws without.
  nlohmann::json ntn_fix;
  if (multi_rtt) ntn_fix = this->ntn_solve(supi);
  // The terrestrial solver needs several TRPs at known, distinct, FIXED positions.
  nlohmann::json locData;
  auto const ntn_status = ntn_fix.is_null() ? std::string{} : ntn_fix.value("status", std::string{});
  try {
    locData = ctx->compute_location(this->gnb);
  } catch (...) {
    if (ntn_status != "ok" && ntn_status != "ambiguous") throw;
    Logger::lmf_app().info("terrestrial solver not applicable (TRPs on satellites); reporting the NTN fix");
    locData = nlohmann::json::object();
  }
  if (!ntn_fix.is_null()) locData["ntnFix"] = ntn_fix;
  if (!ctx->multi_rtt_rounds.empty()) locData["multiRtt"] = ctx->multi_rtt_rounds;
  // using hard coded location data as adeel requiered
  // can not use rel16 LocationData here, incompatible with rel17 values
  // LocationData locationData{locData};
  json_data = locData;  // locationData;

  this->del_supi_2_context(supi);

  return;
}

//------------------------------------------------------------------------------
// One paired Multi-RTT round of this UE as (absolute time, service-link range, satellite position).
void lmf_app::ntn_record(std::string const& supi, nlohmann::json& round) {
  // The TRP this round was measured at: the gNB Rx-Tx carries its gNB, and its SFN Initialisation Time is what
  // turns the (SFN, slot) timestamp into an absolute instant.
  std::optional<double> sfn0;
  uint64_t gnb_of_round = round.value("gnbId", uint64_t{0});
  long trp_of_round     = round.value("trpId", 1L);
  // Every TRP the UE was given assistance data for, by dl-PRS-ID: the gNB and TRP it is, which is how a
  // measurement the UE reports against a dl-PRS-ID finds its satellite.
  struct trp_ref { uint64_t gnb; long trp; };
  std::map<long, trp_ref> by_dl_prs_id;
  {
    std::scoped_lock lk{this->cv_m_gnb};
    for (auto const& [gnbId, g] : this->gnb)
      for (auto const& [trpId, trp] : g.trp) {
        if (!sfn0 && trp.sfn0_unix && (gnb_of_round == 0 || gnbId == gnb_of_round)) {
          sfn0          = trp.sfn0_unix;
          gnb_of_round  = gnbId;
          trp_of_round  = trpId;
        }
        if (trp.dl_prs_id) by_dl_prs_id.emplace(*trp.dl_prs_id, trp_ref{gnbId, trpId});
      }
  }
  if (!sfn0) {
    Logger::lmf_app().warn("NTN Multi-RTT: no SFN Initialisation Time from the TRP, round not timed");
    return;
  }
  // Absolute time of the SRS the gNB Rx-Tx was measured on: SFN0 + (SFN, slot), on the 10.24 s SFN cycle that
  // ends last before now (the response arrives after the measurement). 15 kHz: one slot per ms.
  double const now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  double const base = *sfn0 + (round["gnbSfn"].get<long>() * 10 + round["gnbSlot"].get<long>()) * 1e-3;
  double t = base + std::floor((now - base) / 10.24) * 10.24;
  if (t > now) t -= 10.24;
  // Which satellite carried that TRP at that instant: with a satellite switch the same cell changes satellite
  // mid-session, so this is a function of time (TS 38.305 5.4: the association comes from OAM).
  std::string error;
  auto const sat = ntn::satellite_for(gnb_of_round, trp_of_round, t, error);
  if (!sat) {
    Logger::lmf_app().warn("NTN Multi-RTT: %s", error);
    return;
  }
  double rtt = round["rttUs"].get<double>();

  std::scoped_lock lk{m_ntn};
  auto& h = ntn_history[supi];
  if (!h.empty() && (t < h.back().t_unix || t - h.back().t_unix > 120)) {
    Logger::lmf_app().info("NTN Multi-RTT %s: history restarted (%.1f s after the last round)", supi, t - h.back().t_unix);
    h.clear();
    ntn_last_rtt_rate.erase(supi);
  }
  // The two halves describe different uplink subframes: the gNB Rx-Tx the SRS's, the UE Rx-Tx - measured
  // on DL subframe i - the UL subframe i + offset it transmits then (TS 38.215 5.1.30, 5.1.46).
  double const gap_s =
      (round["ueSlot"].get<long>() + round["subframeOffset"].get<long>() - round["gnbSlot"].get<long>()) * 1e-3;
  if (auto r = ntn_last_rtt_rate.find(supi); r != ntn_last_rtt_rate.end()) rtt -= r->second * gap_s;
  // The rate at which the round trip moves, from this satellite's own previous round - the history may also
  // hold neighbour satellites' ranges, which are a different geometry and would give a meaningless rate.
  auto const prev = std::find_if(h.rbegin(), h.rend(), [&](auto const& r) { return r.sat_id == sat->id; });
  if (prev != h.rend() && t > prev->t_unix) {
    double const prev_rtt = 2 * prev->range_m / 299792458.0 * 1e6 + sat->common_delay_us + sat->rtt_calibration_ns / 1e3;
    ntn_last_rtt_rate[supi] = (rtt - prev_rtt) / (t - prev->t_unix);
  }
  double const range = (rtt - sat->common_delay_us - sat->rtt_calibration_ns / 1e3) * 1e-6 * 299792458.0 / 2;
  h.push_back({t, range, ntn::ecef_at(*sat, t), sat->id});
  while (!h.empty() && t - h.front().t_unix > 900) h.erase(h.begin());  // one pass
  Logger::lmf_app().info(
      "NTN Multi-RTT round %s: t %.6f (SFN0 %.6f + sfn %ld slot %ld), RTT at SRS %.3f us -> range %.3f m, "
      "satellite %d at %.1f %.1f %.1f",
      supi, t, *sfn0, round["gnbSfn"].get<long>(), round["gnbSlot"].get<long>(), rtt, range, sat->id,
      h.back().sat[0], h.back().sat[1], h.back().sat[2]);
  round["tUnix"]        = t;
  round["rangeKmAtSrs"] = range / 1e3;
  round["satelliteId"]  = sat->id;

  // Neighbour TRPs - in NTN, the other satellites over this UE (TS 38.305 5.4.2, 5.4.4).
  if (!round.contains("trps") || round["trps"].size() < 2) return;
  auto const& ts = round["trps"];
  double const r_ref_us  = ts[0]["ueRxTxUs"].get<double>();
  long const   sub_ref   = ts[0]["sfn"].get<long>() * 10 + ts[0]["slot"].get<long>();
  for (size_t i = 1; i < ts.size(); ++i) {
    long const id = ts[i]["dlPrsId"].get<long>();
    auto const it = by_dl_prs_id.find(id);
    if (it == by_dl_prs_id.end()) {
      Logger::lmf_app().warn("NTN Multi-RTT: measurement for dl-PRS-ID %ld, which this session never assisted", id);
      continue;
    }
    long gap_ms = (ts[i]["sfn"].get<long>() * 10 + ts[i]["slot"].get<long>()) - sub_ref;
    gap_ms = (gap_ms % 10240 + 10240) % 10240;          // the SFN cycle wraps
    if (gap_ms > 5120) gap_ms -= 10240;
    // Two TRPs' occasions can sit up to one DL-PRS period apart on the UE's grid (their SFN0 offset), no
    // further.
    constexpr long max_gap_ms = 320;
    if (gap_ms > max_gap_ms || gap_ms < -max_gap_ms) {
      Logger::lmf_app().warn(
          "NTN Multi-RTT: dl-PRS-ID %ld measured %+ld ms from the reference TRP - not one instant, discarded",
          id, gap_ms);
      continue;
    }
    double const t_j = t + gap_ms * 1e-3;
    std::string err_j;
    auto const sat_j = ntn::satellite_for(it->second.gnb, it->second.trp, t_j, err_j);
    if (!sat_j) {
      Logger::lmf_app().warn("NTN Multi-RTT: dl-PRS-ID %ld: %s", id, err_j);
      continue;
    }
    // A round trip of this TRP's own is better than a difference against the reference: it does not inherit
    // the reference's error, and it needs no assumption about the two downlinks sharing a transmit instant.
    bool const   own_rtt = ts[i].contains("rttUs");
    double const range_j =
        own_rtt ? (ts[i]["rttUs"].get<double>() - sat_j->common_delay_us - sat_j->rtt_calibration_ns / 1e3) *
                      1e-6 * 299792458.0 / 2
                : range + (ts[i]["ueRxTxUs"].get<double>() - r_ref_us) * 1e-6 * 299792458.0;
    // A range that is not a range: beyond the geometry any satellite over this UE can have.
    if (range_j < 100e3 || range_j > 4000e3) {
      Logger::lmf_app().warn(
          "NTN Multi-RTT: dl-PRS-ID %ld gives range %.1f km (reference %.1f km) - discarded", id, range_j / 1e3,
          range / 1e3);
      continue;
    }
    auto const pos_j = ntn::ecef_at(*sat_j, t_j);
    h.push_back({t_j, range_j, pos_j, sat_j->id});
    Logger::lmf_app().info(
        "NTN Multi-RTT round %s: t %.6f neighbour dl-PRS-ID %ld (gnbId 0x%x trpId %d), %s -> range %.3f m, "
        "satellite %d at %.1f %.1f %.1f",
        supi, t_j, id, it->second.gnb, it->second.trp,
        own_rtt ? ("own round trip " + std::to_string(ts[i]["rttUs"].get<double>()) + " us").c_str()
                : ("UE Rx-Tx " + std::to_string(ts[i]["ueRxTxUs"].get<double>()) + " us vs reference " +
                   std::to_string(r_ref_us) + " us (" + std::to_string(gap_ms) + " ms)").c_str(),
        range_j, sat_j->id, pos_j[0], pos_j[1], pos_j[2]);
    round["trps"][i]["tUnix"]       = t_j;
    round["trps"][i]["rangeKm"]     = range_j / 1e3;
    round["trps"][i]["satelliteId"] = sat_j->id;
  }
  // The history is appended out of order when a neighbour's occasion precedes the reference's; the solver and
  // the pruning below both read it as a time series.
  std::sort(h.begin(), h.end(), [](auto const& a, auto const& b) { return a.t_unix < b.t_unix; });
}

//------------------------------------------------------------------------------
nlohmann::json lmf_app::ntn_solve(std::string const& supi) {
  std::vector<ntn::range_meas> m;
  {
    std::scoped_lock lk{m_ntn};
    m = ntn_history[supi];
  }
  double const span = m.empty() ? 0 : m.back().t_unix - m.front().t_unix;
  // One satellite has to be given time: its ranges only separate the UE's position as the pass matures. Two
  // satellites at once are already two ranges from two directions, so three of them place the UE at once.
  std::set<int> sats;
  for (auto const& r : m) sats.insert(r.sat_id);
  bool const enough = sats.size() > 1 ? (m.size() >= 3) : (m.size() >= 5 && span >= 20);
  if (!enough) {
    Logger::lmf_app().info("NTN Multi-RTT fix %s: not yet (%zu rounds from %zu satellite(s) over %.0f s)", supi,
                           m.size(), sats.size(), span);
    return {{"status", "insufficient"}, {"rounds", m.size()}, {"spanS", span}, {"satellites", sats.size()}};
  }
  auto const fixes = ntn::solve(m);
  if (fixes.empty()) {
    Logger::lmf_app().warn("NTN Multi-RTT fix %s: no convergence (%zu rounds over %.0f s)", supi, m.size(), span);
    return {{"status", "noConvergence"}, {"rounds", m.size()}, {"spanS", span}};
  }
  auto const& f = fixes.front();
  nlohmann::json alt = nlohmann::json::array();
  std::string alts;
  for (size_t i = 1; i < fixes.size(); i++) {
    alt.push_back({{"latDeg", fixes[i].lat_deg}, {"lonDeg", fixes[i].lon_deg}, {"rmsM", fixes[i].rms_m}});
    alts += " | alt " + std::to_string(fixes[i].lat_deg) + "," + std::to_string(fixes[i].lon_deg) + " rms " +
            std::to_string(int(fixes[i].rms_m)) + " m";
  }
  // One pass can fit a second point nearly as well (before the satellite passes the UE, the range profile does
  // not yet tell which side along the track it is on): then the fix is not unique and is reported as such.
  bool const ambiguous = fixes.size() > 1 && fixes[1].rms_m < 2 * f.rms_m;
  Logger::lmf_app().info(
      "NTN Multi-RTT fix %s: lat %.6f lon %.6f, 1-sigma %.0f x %.0f m (major axis %.0f deg from north), "
      "range rms %.1f m, %d rounds over %.0f s%s%s",
      supi, f.lat_deg, f.lon_deg, f.semi_major_m, f.semi_minor_m, f.orient_deg, f.rms_m, f.n, f.span_s,
      ambiguous ? " AMBIGUOUS" : "", alts);
  return {{"status", ambiguous ? "ambiguous" : "ok"},
          {"satellites", sats.size()},
          {"latDeg", f.lat_deg},
          {"lonDeg", f.lon_deg},
          {"ecefM", f.ecef},
          {"uncertaintySemiMajorM", f.semi_major_m},
          {"uncertaintySemiMinorM", f.semi_minor_m},
          {"orientationMajorAxisDeg", f.orient_deg},
          {"rangeRmsM", f.rms_m},
          {"rounds", f.n},
          {"spanS", f.span_s},
          {"alternatives", alt}};
}

//------------------------------------------------------------------------------
bool lmf_app::_is_supi_2_context(const std::string& supi) const {
  return (supi2ctx.count(supi) > 0) && (supi2ctx.at(supi) != nullptr);
}

//------------------------------------------------------------------------------
bool lmf_app::is_supi_2_context(const string& supi) const {
  std::shared_lock lock(m_supi2ctx);
  return _is_supi_2_context(supi);
}

std::shared_ptr<LocationDetermination> lmf_app::create_lmf_context(
    const string& supi) {
  std::unique_lock lock(m_supi2ctx);

  if (this->_is_supi_2_context(supi)) {
    Logger::lmf_app().warn(
        "create_lmf_context: %s: already exist, remove", supi);
    this->supi2ctx.erase(supi);
  }
  return supi2ctx[supi] = std::make_shared<LocationDetermination>(supi);
}

//------------------------------------------------------------------------------
std::shared_ptr<LocationDetermination> lmf_app::supi_2_context(
    const std::string& supi) const {
  std::shared_lock lock(m_supi2ctx);

  if (!_is_supi_2_context(supi)) {
    return {nullptr};
  }
  return supi2ctx.at(supi);
}

//------------------------------------------------------------------------------
void lmf_app::set_supi_2_context(
    const string& supi, const std::shared_ptr<LocationDetermination>& lc) {
  std::unique_lock lock(m_supi2ctx);
  supi2ctx[supi] = lc;
}

//------------------------------------------------------------------------------
void lmf_app::del_supi_2_context(const string& supi) {
  std::unique_lock lock(m_supi2ctx);
  supi2ctx.erase(supi);
}

//------------------------------------------------------------------------------
void lmf_app::create_n1n2subscription(const std::string& supi) {
  std::unique_lock lock(m_supi2n1n2subs);

  auto const& [iter, inserted] = supi2n1n2subs.try_emplace(supi, supi);
  auto const& subscription     = iter->second;
  Logger::lmf_app().info(
      "n1n2info %s for supi: %s id: %s"s,
      inserted ? "subscription created"s : "already subscribed"s,
      subscription.supi, subscription.id);
}

//------------------------------------------------------------------------------
void oai::lmf::app::lmf_app::release_n1n2subscription(const std::string& supi) {
  std::unique_lock lock(this->m_supi2n1n2subs);

  this->supi2n1n2subs.erase(supi);
}

//------------------------------------------------------------------------------
void oai::lmf::app::lmf_app::release_all_n1n2subscriptions() {
  std::unique_lock lock(m_supi2n1n2subs);

  this->supi2n1n2subs.clear();
}

//------------------------------------------------------------------------------
void lmf_app::create_non_ue_subscription() {
  std::scoped_lock lock(this->m_non_ue_subs);

  if (!this->nonUeN2MessageSubscription) {
    this->nonUeN2MessageSubscription =
        std::make_unique<NonUeN2MessageSubscription>();
    Logger::lmf_app().info("non-ue subscription created");
  } else {
    Logger::lmf_app().debug(
        "non-ue subscription not created: already subscribed");
  }
}

//------------------------------------------------------------------------------
void oai::lmf::app::lmf_app::release_non_ue_subscription() {
  std::scoped_lock lock(this->m_non_ue_subs);

  if (this->nonUeN2MessageSubscription) {
    this->nonUeN2MessageSubscription.reset();
    Logger::lmf_app().info("non-ue subscription deleted");
  } else {
    Logger::lmf_app().debug("non-ue subscription not deleted: already deleted");
  }
}

//------------------------------------------------------------------------------
NRPPATransactionID_t getNrppaTxnId(NrppaPduShared nrppa) {
  switch (nrppa->present) {
    case NRPPA_PDU_PR_initiatingMessage:
      return nrppa->choice.initiatingMessage->nrppatransactionID;
    case NRPPA_PDU_PR_successfulOutcome:
      return nrppa->choice.successfulOutcome->nrppatransactionID;
    case NRPPA_PDU_PR_unsuccessfulOutcome:
      return nrppa->choice.unsuccessfulOutcome->nrppatransactionID;
    default:
      oai::lmf::api::lmf_sbi_helper::throwHttpError(
          "getNrppaTxnId"s, "malformed nrppa message"s);
  }
  return 0;
}

//------------------------------------------------------------------------------
// check 1:1 relationship between procedureCode and value.present
template<typename T, typename U>
static void checkPC(T const& present, U const& expected) {
  if (present->procedureCode != expected) {
    auto const &title  = "handle_nrppa_notification: invalid procedue code"s,
               &ps     = "present: "s + std::to_string(present->procedureCode),
               &es     = "expected: "s + std::to_string(expected),
               &detail = ps + ": "s + es;
    oai::lmf::api::lmf_sbi_helper::throwHttpError(title, detail);
  }
}

//------------------------------------------------------------------------------
template<typename T, typename U, typename V>
static U const& getPR(U const& choice, T const& value, V const& expected) {
  if (value.present != expected) {
    auto const &title  = "handle_nrppa_notification: invalid message"s,
               &ps     = "present: "s + std::to_string(value.present),
               &es     = "expected: "s + std::to_string(expected),
               &detail = ps + ": "s + es;
    oai::lmf::api::lmf_sbi_helper::throwHttpError(title, detail);
  }
  return choice;
}

//------------------------------------------------------------------------------
void lmf_app::handle_trp_information_response(
    NrppaPduShared nrppa, NRPPATransactionID_t const& tId,
    TRPInformationResponse_t const& trpInformation) {
  Logger::lmf_app().debug("trp information received");
  std::scoped_lock lk{this->cv_m_gnb};

  for (auto const& trpInformationIE : trpInformation.protocolIEs) {
    if (trpInformationIE->id == ProtocolIE_ID_id_TRPInformationListTRPResp) {
      auto const& value              = trpInformationIE->value;
      auto const& trpInformationList = getPR(
          value.choice.TRPInformationListTRPResp, value,
          TRPInformationResponse_IEs__value_PR_TRPInformationListTRPResp);
      for (auto const& trpInformationListMember : trpInformationList) {
        auto const& trpId = trpInformationListMember->tRPInformation.tRP_ID;
        std::optional<GnbId> gnbId;
        Trp trp;

        for (auto const& trpInformationItem :
             trpInformationListMember->tRPInformation.tRPInformationTypeResponseList) {
          switch (trpInformationItem->present) {
            case TRPInformationTypeResponseItem_PR_cGI_NR: {
              auto const& ngRanCgi      = trpInformationItem->choice.cGI_NR;
              auto const& plmnnIdentity = ngRanCgi->pLMN_Identity;
              if (plmnnIdentity.size != 3) {
                oai::lmf::api::lmf_sbi_helper::throwHttpError(
                    "trp information response",
                    "plmnnIdentity.size != 3: "s +
                        std::to_string(plmnnIdentity.size));
              }

              {
                auto const& ngRanCell = ngRanCgi->nRcellIdentifier;
                if (ngRanCell.size != 5 || ngRanCell.bits_unused != 4) {
                  oai::lmf::api::lmf_sbi_helper::throwHttpError(
                      "trp information response",
                      "ngRanCell.size != 5: "s +
                          std::to_string(ngRanCell.size) +
                          " || ngRanCell.bits_unused != 4: " +
                          std::to_string(ngRanCell.bits_unused));
                }
                uint64_t nci = 0;
                for (auto i = 0, s = 32; i < 5; ++i, s -= 8) {
                  nci |= static_cast<uint64_t>(ngRanCell.buf[i]) << s;
                }
                nci >>= ngRanCell.bits_unused;
                auto const& cellIdBitCnt = 36 - lmf_cfg.gnb_id_bits_count;
                gnbId.emplace(nci >> cellIdBitCnt);

                if (this->gnb.count(gnbId.value()) == 0) {
                  static auto constexpr d1 = [](auto const& v) constexpr {
                    return v & 0xf;
                  };
                  static auto constexpr d2 = [](auto const& v) constexpr {
                    return v >> 4;
                  };
                  auto const& pb  = plmnnIdentity.buf;
                  auto const& mcc = (boost::format("%0d%0d%0d") % d1(pb[0]) %
                                     d2(pb[0]) % d1(pb[1]))
                                        .str();
                  auto const& mncd3 = d2(pb[1]);
                  auto const& mnc2  = (mncd3 == 0xf);
                  auto const& mnc =
                      mnc2 ? (boost::format("%0d%0d") % d1(pb[2]) % d2(pb[2]))
                                 .str() :
                             (boost::format("%0d%0d%0d") % d1(pb[2]) %
                              d2(pb[2]) % mncd3)
                                 .str();
                  if (mcc.size() > 3) {
                    oai::lmf::api::lmf_sbi_helper::throwHttpError(
                        "trp information response", "invalid mcc: "s + mcc);
                  }
                  if (mnc.size() > (mnc2 ? 2 : 3)) {
                    oai::lmf::api::lmf_sbi_helper::throwHttpError(
                        "trp information response", "invalid mnc: "s + mnc);
                  }

                  PlmnId plmnId;
                  plmnId.setMcc(mcc);
                  plmnId.setMnc(mnc);

                  auto const& gnbValue =
                      (boost::format(
                           lmf_cfg.gnb_id_bits_count <= 24 ? "%06x" : "%08x") %
                       gnbId.value())
                          .str();
                  GNbId gNbId;
                  gNbId.setGNBValue(gnbValue);
                  gNbId.setBitLength(lmf_cfg.gnb_id_bits_count);

                  GlobalRanNodeId globalRanNodeId;
                  globalRanNodeId.setPlmnId(plmnId);
                  globalRanNodeId.setGNbId(gNbId);

                  Logger::lmf_app().info(
                      "trp information: adding gnb with id: 0x%x mcc: %s mnc: "
                      "%s",
                      gnbId.value(), mcc, mnc);

                  if (auto const& [iter, inserted] = this->gnb.try_emplace(
                          gnbId.value(), gnbId.value(), globalRanNodeId);
                      !inserted) {
                    oai::lmf::api::lmf_sbi_helper::throwHttpError(
                        "trp information response",
                        "gnbId: "s + std::to_string(gnbId.value()) +
                            " already inserted"s);
                  }
                }
              }

            } break;

            case TRPInformationTypeResponseItem_PR_pCI_NR:
              trp.pci = trpInformationItem->choice.pCI_NR;
              break;
            case TRPInformationTypeResponseItem_PR_sFNInitialisationTime: {
              auto const& b = trpInformationItem->choice.sFNInitialisationTime;
              if (b.size != 8) break;
              uint64_t v = 0;
              for (int i = 0; i < 8; i++) v = (v << 8) | b.buf[i];
              trp.sfn0_unix = ntn::relative_time_1900_to_unix(v);
            } break;
            case TRPInformationTypeResponseItem_PR_aRFCN:
              trp.arfcn = trpInformationItem->choice.aRFCN;
              break;
            case TRPInformationTypeResponseItem_PR_pRSConfiguration: {
              auto const& sets = trpInformationItem->choice.pRSConfiguration->pRSResourceSet_List.list;
              if (sets.count < 1 || sets.array[0]->pRSResource_List.list.count < 1) break;
              if (sets.count > 1 || sets.array[0]->pRSResource_List.list.count > 1)
                Logger::lmf_app().warn(
                    "trp information: %d PRS resource sets, using the first set's first resource", sets.count);
              auto const* s = sets.array[0];
              auto const* r = s->pRSResource_List.list.array[0];
              trp.prs = lpp::dl_prs{
                  s->pRSResourceSetID, s->subcarrierSpacing, s->pRSbandwidth, s->startPRB,
                  s->pointA, s->combSize, s->cPType, s->resourceSetPeriodicity, s->resourceSetSlotOffset,
                  s->resourceRepetitionFactor, s->resourceTimeGap, s->resourceNumberofSymbols,
                  s->pRSResourceTransmitPower, r->pRSResourceID, r->sequenceID, r->rEOffset,
                  r->resourceSlotOffset, r->resourceSymbolOffset};
            } break;

            case TRPInformationTypeResponseItem_PR_geographicalCoordinates: {
              auto const& geographicalCoordinates =
                  trpInformationItem->choice.geographicalCoordinates;
              auto const& trpPositionDefinitionType =
                  geographicalCoordinates->tRPPositionDefinitionType;
              switch (trpPositionDefinitionType.present) {
                case TRPPositionDefinitionType_PR_referenced: {
                  auto const& referenced =
                      trpPositionDefinitionType.choice.referenced;
                  auto const& referencePoint = referenced->referencePoint;
                  auto const& referencePointType =
                      referenced->referencePointType;

                  switch (referencePoint.present) {
                    case ReferencePoint_PR_relativeCoordinateID: {
                      trp.relativeCoordinateID =
                          referencePoint.choice.relativeCoordinateID;
                    } break;

                    default:
                      Logger::lmf_app().warn(
                          "trp information: unhandled ReferencePoint_PR: %d",
                          referencePoint.present);
                  }

                  switch (referencePointType.present) {
                    case TRPReferencePointType_PR_tRPPositionRelativeCartesian: {
                      trp.relativeCartesianLocation =
                          *referencePointType.choice
                               .tRPPositionRelativeCartesian;
                    } break;

                    default:
                      Logger::lmf_app().warn(
                          "trp information: unhandled "
                          "TRPReferencePointType_PR: %d",
                          referencePointType.present);
                  }

                } break;

                case TRPPositionDefinitionType_PR_direct: {
                  // A gNB may report an absolute antenna position instead of a
                  // relative one (OCUDU does). Convert it to the local
                  // cartesian frame the solver works in.
                  auto const& accuracy =
                      trpPositionDefinitionType.choice.direct->accuracy;
                  double latDeg = 0, lonDeg = 0, altM = 0;
                  if (accuracy.present ==
                      TRPPositionDirectAccuracy_PR_tRPPosition) {
                    auto const& pos = accuracy.choice.tRPPosition;
                    // TS 23.032 6.1: N <= 2^23 * |lat| / 90, N <= 2^24 * lon/360
                    // (~1.2 m latitude steps).
                    latDeg = (pos->latitudeSign ==
                                      NG_RANAccessPointPosition__latitudeSign_south ?
                                  -1.0 :
                                  1.0) *
                             static_cast<double>(pos->latitude) * 90.0 /
                             (1 << 23);
                    lonDeg = static_cast<double>(pos->longitude) * 360.0 /
                             (1L << 24);
                    altM = (pos->directionOfAltitude ==
                                    NG_RANAccessPointPosition__directionOfAltitude_depth ?
                                -1.0 :
                                1.0) *
                           static_cast<double>(pos->altitude);
                  } else if (
                      accuracy.present ==
                      TRPPositionDirectAccuracy_PR_tRPHAposition) {
                    auto const& pos = accuracy.choice.tRPHAposition;
                    // TS 23.032 6.1a/6.3a: lat = N*90/2^31, lon = N*180/2^31,
                    // alt = N/128 m (~5 mm steps).
                    latDeg = static_cast<double>(pos->latitude) * 90.0 /
                             2147483648.0;
                    lonDeg = static_cast<double>(pos->longitude) * 180.0 /
                             2147483648.0;
                    altM = static_cast<double>(pos->altitude) / 128.0;
                  } else {
                    Logger::lmf_app().warn(
                        "trp information: unhandled "
                        "TRPPositionDirectAccuracy_PR: %d",
                        accuracy.present);
                    break;
                  }

                  // The first absolute position seen anchors the local frame;
                  // every other TRP is expressed as an offset from it
                  // (equirectangular approximation - exact enough over a site).
                  static bool originSet   = false;
                  static double originLat = 0.0;
                  static double originLon = 0.0;
                  static double originAlt = 0.0;
                  if (!originSet) {
                    originLat = latDeg;
                    originLon = lonDeg;
                    originAlt = altM;
                    originSet = true;
                  }

                  constexpr double kMetresPerDegLat = 111132.95;
                  constexpr double kMetresPerDegLon = 111319.49;
                  double const eastM = (lonDeg - originLon) *
                                       kMetresPerDegLon *
                                       std::cos(originLat * M_PI / 180.0);
                  double const northM = (latDeg - originLat) * kMetresPerDegLat;

                  trp.relativeCartesianLocation.xYZunit =
                      RelativeCartesianLocation__xYZunit_cm;
                  trp.relativeCartesianLocation.xvalue =
                      std::lround(eastM * 100.0);
                  trp.relativeCartesianLocation.yvalue =
                      std::lround(northM * 100.0);
                  trp.relativeCartesianLocation.zvalue =
                      std::lround((altM - originAlt) * 100.0);

                  Logger::lmf_app().info(
                      "trp information: direct position lat: %.7f lon: %.7f "
                      "alt: %.1fm -> local x: %ldcm y: %ldcm z: %ldcm",
                      latDeg, lonDeg, altM,
                      trp.relativeCartesianLocation.xvalue,
                      trp.relativeCartesianLocation.yvalue,
                      trp.relativeCartesianLocation.zvalue);
                } break;

                default:
                  Logger::lmf_app().warn(
                      "trp information: unhandled "
                      "TRPPositionDefinitionType_PR: %d",
                      trpPositionDefinitionType.present);
              }
            } break;

            default:
              Logger::lmf_app().warn(
                  "trp information: unhandled TRPInformationTypeResponseItem_PR: %d",
                  trpInformationItem->present);
          }
        }

        if (gnbId.has_value()) {
          if (this->gnb.at(gnbId.value()).trp.count(trpId) == 0) {
            Logger::lmf_app().info(
                "trp information: adding to gnbId: 0x%x trpId: %d coordID: %d "
                "x: %d y: %d z: %d",
                gnbId.value(), trpId, trp.relativeCoordinateID,
                trp.relativeCartesianLocation.xvalue,
                trp.relativeCartesianLocation.yvalue,
                trp.relativeCartesianLocation.zvalue);
            if (auto const& [iter, inserted] =
                    this->gnb.at(gnbId.value()).trp.try_emplace(trpId, trp);
                !inserted) {
              oai::lmf::api::lmf_sbi_helper::throwHttpError(
                  "trp information response",
                  "trpId: "s + std::to_string(trpId) + " already inserted"s);
            }
          } else {
            // Known TRP answering again (every location session asks): what changes with a gNB restart - its SFN
            // timing and PRS - is refreshed; the position is not.
            auto& known     = this->gnb.at(gnbId.value()).trp.at(trpId);
            known.pci       = trp.pci;
            known.arfcn     = trp.arfcn;
            known.prs       = trp.prs;
            known.sfn0_unix = trp.sfn0_unix;
          }
        } else {
          oai::lmf::api::lmf_sbi_helper::throwHttpError(
              "trp information", "no gnbId");
        }
      }
    }
  }
  this->cv_gnb.notify_one();
}

//------------------------------------------------------------------------------
bool lmf_app::handle_non_ue_n2info_nrppa_notification(NrppaPduShared nrppa) {
  auto const& nrppaTxnId = getNrppaTxnId(nrppa);
  auto const& supi       = this->get_nrppaTxnId2Supi(nrppaTxnId);

  return this->handle_n2info_nrppa_notification(supi, nrppa);
}

//------------------------------------------------------------------------------
// TODO: replace bool retval with exception
// shoult not fail
void lmf_app::handle_n1_lpp_notification(
    std::string const& supi, std::string const& correlation_id,
    std::string const& lpp_pdu) {
  auto ctx = this->supi_2_context(supi);
  if (!ctx) {
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "N1MessageNotify (LPP)"s, "no location session for "s + supi, "",
        Pistache::Http::Code::Not_Found);
  }
  ctx->handle_lpp_uplink(correlation_id, lpp_pdu);
}

//------------------------------------------------------------------------------
bool lmf_app::handle_n2info_nrppa_notification(
    std::string supi, NrppaPduShared nrppa) {
  auto ctx = this->supi_2_context(supi);
  if (!ctx) {
    Logger::lmf_server().error("N2InfoNotify: unknown supi: %s", supi);
    return false;
  }

  auto const& tId = getNrppaTxnId(nrppa);

  // TODO
  if (nrppa->present == NRPPA_PDU_PR_initiatingMessage) {
    // don't forget tId handling ctx->nrppa_tId.erase(tId);
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "handle_n2info_nrppa_notification",
        "NRPPA_PDU_PR_initiatingMessage not implemented");
  }

  ProcedureCode_t procedureCode;
  {
    std::scoped_lock lk{ctx->m_tId};
    if (ctx->nrppa_tId.count(tId) != 1) {
      oai::lmf::api::lmf_sbi_helper::throwHttpError(
          "handle_n2info_nrppa_notification"s,
          "unknown nrppa transaction id: "s + std::to_string(tId));
    }
    procedureCode = ctx->nrppa_tId.at(tId);
    // with multiple gnb expect multiple trp infos, erase after timeout
    if (procedureCode != ProcedureCode_id_tRPInformationExchange) {
      ctx->nrppa_tId.erase(tId);          // not for incomming/initiating!
      this->nrppa_tid_gen.free_uid(tId);  // for reuse
    }
  }
  // is non-ue but not a broadcast like trp-info
  // TODO: introduce non-ue "was broadcast" switch
  if (procedureCode == ProcedureCode_id_Measurement) {
    this->erase_nrppaTxnId2Supi(tId);
  }

  if (nrppa->present == NRPPA_PDU_PR_unsuccessfulOutcome) {
    auto const& unsuccessfulOutcome = getPR(
        nrppa->choice.unsuccessfulOutcome, *nrppa,
        NRPPA_PDU_PR_unsuccessfulOutcome);
    checkPC(unsuccessfulOutcome, procedureCode);
    switch (procedureCode) {
      case ProcedureCode_id_tRPInformationExchange: {
        std::scoped_lock lk{this->cv_m_gnb};

        auto const& value                 = unsuccessfulOutcome->value;
        auto const& trpInformationFailure = getPR(
            value.choice.TRPInformationFailure, value,
            UnsuccessfulOutcome__value_PR_TRPInformationFailure);
        auto err = CauseError::parse(
            trpInformationFailure, TRPInformationFailure_IEs__value_PR_Cause);
        this->trp_info_err.push_back(err);
        this->cv_gnb.notify_one();
        return true;
      }; break;

      case ProcedureCode_id_positioningInformationExchange: {
        auto const& value                         = unsuccessfulOutcome->value;
        auto const& positioningInformationFailure = getPR(
            value.choice.PositioningInformationFailure, value,
            UnsuccessfulOutcome__value_PR_PositioningInformationFailure);
        ctx->handle_positioning_information_failure(
            nrppa, positioningInformationFailure);
        return true;
      }; break;

      case ProcedureCode_id_positioningActivation: {
        auto const& value                        = unsuccessfulOutcome->value;
        auto const& positioningActivationFailure = getPR(
            value.choice.PositioningActivationFailure, value,
            UnsuccessfulOutcome__value_PR_PositioningActivationFailure);
        ctx->handle_positioning_activation_failure(
            nrppa, positioningActivationFailure);
        return true;
      }; break;

      case ProcedureCode_id_Measurement: {
        auto const& value              = unsuccessfulOutcome->value;
        auto const& measurementFailure = getPR(
            value.choice.MeasurementFailure, value,
            UnsuccessfulOutcome__value_PR_MeasurementFailure);
        ctx->handle_measurement_failure(nrppa, tId, measurementFailure);
        return true;
      }; break;

      default:
        oai::lmf::api::lmf_sbi_helper::throwHttpError(
            "handle_nrppa_notification"s,
            "unsuccessfulOutcome: unhandled procedure code: %d"s +
                std::to_string(procedureCode));
    }
  }

  auto const& successfulOutcome = getPR(
      nrppa->choice.successfulOutcome, *nrppa, NRPPA_PDU_PR_successfulOutcome);
  checkPC(successfulOutcome, procedureCode);
  switch (procedureCode) {
    case ProcedureCode_id_tRPInformationExchange: {
      auto const& value                  = successfulOutcome->value;
      auto const& trpInformationResponse = getPR(
          value.choice.TRPInformationResponse, value,
          SuccessfulOutcome__value_PR_TRPInformationResponse);
      this->handle_trp_information_response(nrppa, tId, trpInformationResponse);
      return true;
    } break;

    case ProcedureCode_id_positioningInformationExchange: {
      auto const& value                          = successfulOutcome->value;
      auto const& positioningInformationResponse = getPR(
          value.choice.PositioningInformationResponse, value,
          SuccessfulOutcome__value_PR_PositioningInformationResponse);
      ctx->handle_positioning_information_response(
          nrppa, tId, positioningInformationResponse);
      return true;
    } break;

    case ProcedureCode_id_Measurement: {
      auto const& value               = successfulOutcome->value;
      auto const& measurementResponse = getPR(
          value.choice.MeasurementResponse, value,
          SuccessfulOutcome__value_PR_MeasurementResponse);
      ctx->handle_measurement_response(nrppa, tId, measurementResponse);
      return true;
    } break;

    case ProcedureCode_id_positioningActivation: {
      auto const& value                         = successfulOutcome->value;
      auto const& positioningActivationResponse = getPR(
          value.choice.PositioningActivationResponse, value,
          SuccessfulOutcome__value_PR_PositioningActivationResponse);
      ctx->handle_positioning_activation_response(
          nrppa, tId, positioningActivationResponse);
      return true;
    } break;

    default:
      oai::lmf::api::lmf_sbi_helper::throwHttpError(
          "handle_nrppa_notification"s,
          "successfulOutcome: unhandled procedure code: %d"s +
              std::to_string(procedureCode));
  }

  auto titel  = "n2info nrppa notifiaction pdu error"s;
  auto detail = "unhandled nrppa  pdu: " + std::to_string(nrppa->present);
  oai::lmf::api::lmf_sbi_helper::throwHttpError(titel, detail);

  return false;
}

//------------------------------------------------------------------------------
NrppaPduShared lmf_app::parse_n2_info_container_nrppa(
    N2InformationNotification const& n2InformationNotification,
    mime_part const& nrppa_part) {
  if (!n2InformationNotification.n2InfoContainerIsSet()) {
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "parse_n2_info_container_nrppa", "N2InfoContainer not present");
  }

  auto const& n2InfoContainer = n2InformationNotification.getN2InfoContainer();
  auto const& eN2InformationClass =
      n2InfoContainer.getN2InformationClass().getEnumValue();

  // Check N2 Information Class
  if (eN2InformationClass !=
      N2InformationClass_anyOf::eN2InformationClass_anyOf::NRPPA) {
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "parse_n2_info_container_nrppa",
        "N2 Information Class not NRPPA: " +
            std::to_string(static_cast<int>(eN2InformationClass)));
  }

  if (!n2InfoContainer.nrppaInfoIsSet()) {
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "parse_n2_info_container_nrppa", "nrppaInfo not present");
  }
  auto const& nrppaInfo = n2InfoContainer.getNrppaInfo();

  if (nrppaInfo.getNfId() != lmf_nrf_inst->lmf_instance_id) {
    Logger::lmf_server().warn(
        "nfId != '%s': '%s'", lmf_nrf_inst->lmf_instance_id,
        nrppaInfo.getNfId());
  }

  auto const& nrppaPdu = nrppaInfo.getNrppaPdu();
  if (!nrppaPdu.ngapIeTypeIsSet()) {
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "parse_n2_info_container_nrppa", "ngapIeType not present");
  }

  auto const& eNgapIeType = nrppaPdu.getNgapIeType().getEnumValue();
  if (eNgapIeType != NgapIeType_anyOf::eNgapIeType_anyOf::NRPPA_PDU) {
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "parse_n2_info_container_nrppa",
        "ngapIeType not NRPPA_PDU: " +
            std::to_string(static_cast<int>(eNgapIeType)));
  }
  auto const& ngapData = nrppaPdu.getNgapData();
  Logger::lmf_app().debug(
      "parse_n2_info_container_nrppa: content-id: " + ngapData.getContentId());
  if (nrppa_part.content_type != "application/vnd.3gpp.ngap") {
    Logger::lmf_server().warn(
        "content-type != 'application/vnd.3gpp.ngap': '%s'",
        nrppa_part.content_type);
  }

  auto const& nrppa_bin = nrppa_part.body;
  NRPPA_PDU_t* nrppa    = nullptr;
  auto const& rc        = asn_decode(
      NULL, ATS_ALIGNED_CANONICAL_PER, &asn_DEF_NRPPA_PDU, (void**) &nrppa,
      nrppa_bin.c_str(), nrppa_bin.length());
  if (rc.code != RC_OK) {
    ASN_STRUCT_FREE(asn_DEF_NRPPA_PDU, nrppa);
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "parse_n2_info_container_nrppa",
        "asn_decode failed: " + std::to_string(rc.code));
  }
  // xer_fprint(stdout, &asn_DEF_NRPPA_PDU, nrppa);
  Logger::lmf_server().debug("asn_decode ok, consumed: %d", rc.consumed);

  return share_nrppa_pdu(nrppa);
}

//------------------------------------------------------------------------------
void oai::lmf::app::lmf_app::insert_nrppaTxnId2supi(
    NRPPATransactionID_t const& nrppaTxnId, std::string const& supi) {
  std::unique_lock lock{this->m_nrppaTxnId2supi};
  auto const& [iter, insered] =
      this->nrppaTxnId2supi.try_emplace(nrppaTxnId, supi);
  if (!insered) {
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "insert_nrppaTxnId2supi",
        "nrppa id "s + std::to_string(nrppaTxnId) + " reuse"s);
  }
}

//------------------------------------------------------------------------------
std::string oai::lmf::app::lmf_app::extract_nrppaTxnId2Supi(
    NRPPATransactionID_t const& nrppaTxnId) {
  std::unique_lock lock{this->m_nrppaTxnId2supi};
  auto const& nh = this->nrppaTxnId2supi.extract(nrppaTxnId);
  if (nh.empty()) {
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "extract_nrppaTxnId2Supi",
        "unknown nrppa txn id:"s + std::to_string(nrppaTxnId));
  }
  return nh.mapped();
}

std::string oai::lmf::app::lmf_app::get_nrppaTxnId2Supi(
    NRPPATransactionID_t const& nrppaTxnId) {
  std::unique_lock lock{this->m_nrppaTxnId2supi};
  auto const& it = this->nrppaTxnId2supi.find(nrppaTxnId);
  if (it == std::end(this->nrppaTxnId2supi)) {
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "get_nrppaTxnId2Supi",
        "unknown nrppa txn id:"s + std::to_string(nrppaTxnId));
  }
  return it->second;
}

void oai::lmf::app::lmf_app::erase_nrppaTxnId2Supi(
    NRPPATransactionID_t const& nrppaTxnId) {
  std::unique_lock lock{this->m_nrppaTxnId2supi};
  auto const& it = this->nrppaTxnId2supi.find(nrppaTxnId);
  if (it == std::end(this->nrppaTxnId2supi)) {
    oai::lmf::api::lmf_sbi_helper::throwHttpError(
        "erase_nrppaTxnId2Supi",
        "unknown nrppa txn id:"s + std::to_string(nrppaTxnId));
  }
  this->nrppaTxnId2supi.erase(it);
}
