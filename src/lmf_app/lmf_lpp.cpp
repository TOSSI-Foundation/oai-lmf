#include "lmf_lpp.hpp"

#include <cstdlib>
#include <memory>
#include <type_traits>

extern "C" {
#include "LPP_LPP-Message.h"
#include "LPP_LPP-MessageBody.h"
#include "LPP_LPP-TransactionID.h"
#include "LPP_Acknowledgement.h"
#include "LPP_RequestCapabilities.h"
#include "LPP_RequestCapabilities-r9-IEs.h"
#include "LPP_NR-Multi-RTT-RequestCapabilities-r16.h"
#include "LPP_ProvideCapabilities.h"
#include "LPP_ProvideCapabilities-r9-IEs.h"
#include "LPP_NR-Multi-RTT-ProvideCapabilities-r16.h"
#include "LPP_Multi-RTT-MeasCapabilityPerBand-r17.h"
#include "LPP_RequestLocationInformation.h"
#include "LPP_RequestLocationInformation-r9-IEs.h"
#include "LPP_CommonIEsRequestLocationInformation.h"
#include "LPP_NR-Multi-RTT-RequestLocationInformation-r16.h"
#include "LPP_ProvideLocationInformation.h"
#include "LPP_ProvideLocationInformation-r9-IEs.h"
#include "LPP_CommonIEsProvideLocationInformation.h"
#include "LPP_LocationError.h"
#include "LPP_NR-Multi-RTT-ProvideLocationInformation-r16.h"
#include "LPP_NR-Multi-RTT-SignalMeasurementInformation-r16.h"
#include "LPP_NR-Multi-RTT-MeasElement-r16.h"
#include "LPP_NR-NTN-UE-RxTxMeasurements-r18.h"
#include "LPP_NR-Multi-RTT-Error-r16.h"
#include "LPP_NR-Multi-RTT-TargetDeviceErrorCauses-r16.h"
#include "LPP_ProvideAssistanceData.h"
#include "LPP_ProvideAssistanceData-r9-IEs.h"
#include "LPP_NR-Multi-RTT-ProvideAssistanceData-r16.h"
#include "LPP_NR-DL-PRS-AssistanceData-r16.h"
#include "LPP_NR-DL-PRS-AssistanceDataPerFreq-r16.h"
#include "LPP_NR-DL-PRS-AssistanceDataPerTRP-r16.h"
#include "LPP_NR-DL-PRS-ResourceSet-r16.h"
#include "LPP_NR-DL-PRS-Resource-r16.h"
}

namespace oai::lmf::app::lpp {

namespace {

struct lpp_free {
  void operator()(LPP_LPP_Message_t* m) const {
    ASN_STRUCT_FREE(asn_DEF_LPP_LPP_Message, m);
  }
};
using lpp_ptr = std::unique_ptr<LPP_LPP_Message_t, lpp_free>;

template<typename T>
T* alloc() {
  return static_cast<T*>(calloc(1, sizeof(T)));
}

// Allocate a zeroed object of whatever type a (pointer) field points to; the anonymous nested C types
// asn1c generates are awkward to name from C++.
template<typename Ptr>
void alloc_into(Ptr& field) {
  field = alloc<std::remove_pointer_t<Ptr>>();
}

std::string to_xer(const LPP_LPP_Message_t* msg) {
  auto enc = asn_encode_to_new_buffer(
      nullptr, ATS_BASIC_XER, &asn_DEF_LPP_LPP_Message, msg);
  if (enc.buffer == nullptr) return "<xer encode failed>";
  std::string xer(static_cast<char*>(enc.buffer), enc.result.encoded);
  free(enc.buffer);
  return xer;
}

const char* body_name(const LPP_LPP_Message_t* msg) {
  if (msg->lpp_MessageBody == nullptr) return "none";
  if (msg->lpp_MessageBody->present != LPP_LPP_MessageBody_PR_c1 ||
      msg->lpp_MessageBody->choice.c1 == nullptr)
    return "messageClassExtension";
  switch (msg->lpp_MessageBody->choice.c1->present) {
    case LPP_LPP_MessageBody__c1_PR_requestCapabilities:
      return "requestCapabilities";
    case LPP_LPP_MessageBody__c1_PR_provideCapabilities:
      return "provideCapabilities";
    case LPP_LPP_MessageBody__c1_PR_requestAssistanceData:
      return "requestAssistanceData";
    case LPP_LPP_MessageBody__c1_PR_provideAssistanceData:
      return "provideAssistanceData";
    case LPP_LPP_MessageBody__c1_PR_requestLocationInformation:
      return "requestLocationInformation";
    case LPP_LPP_MessageBody__c1_PR_provideLocationInformation:
      return "provideLocationInformation";
    case LPP_LPP_MessageBody__c1_PR_abort:
      return "abort";
    case LPP_LPP_MessageBody__c1_PR_error:
      return "error";
    default:
      return "spare";
  }
}

std::string encode(LPP_LPP_Message_t* msg) {
  auto enc = asn_encode_to_new_buffer(
      nullptr, ATS_UNALIGNED_BASIC_PER, &asn_DEF_LPP_LPP_Message, msg);
  if (enc.buffer == nullptr) return {};
  std::string pdu(static_cast<char*>(enc.buffer), enc.result.encoded);
  free(enc.buffer);
  return pdu;
}

// Inverse of TS 38.133 10.1.25.3.1: value v at granularity k covers [(v-1) 2^k - 985024, v 2^k - 985024) Tc;
// take the middle.
double rxtx_tc(long v, int k) {
  double const step = double(1L << k);
  return (v - 1) * step - 985024 + step / 2;
}

void decode_multi_rtt(const LPP_ProvideLocationInformation_r9_IEs_t* ies, message& m) {
  if (ies->commonIEsProvideLocationInformation &&
      ies->commonIEsProvideLocationInformation->locationError)
    m.location_error =
        ies->commonIEsProvideLocationInformation->locationError->locationfailurecause;
  if (!ies->ext2 || !ies->ext2->nr_Multi_RTT_ProvideLocationInformation_r16) return;
  auto const* pli = ies->ext2->nr_Multi_RTT_ProvideLocationInformation_r16;
  if (pli->nr_Multi_RTT_Error_r16 &&
      pli->nr_Multi_RTT_Error_r16->present ==
          LPP_NR_Multi_RTT_Error_r16_PR_targetDeviceErrorCauses_r16)
    m.multi_rtt_error =
        pli->nr_Multi_RTT_Error_r16->choice.targetDeviceErrorCauses_r16->cause_r16;
  auto const* smi = pli->nr_Multi_RTT_SignalMeasurementInformation_r16;
  if (!smi) return;
  static constexpr long nta_tc[] = {25600, 0, 39936, 13792};  // nTA1..nTA4
  for (int i = 0; i < smi->nr_Multi_RTT_MeasList_r16.list.count; i++) {
    auto const* e = smi->nr_Multi_RTT_MeasList_r16.list.array[i];
    multi_rtt_meas r;
    r.dl_prs_id = e->dl_PRS_ID_r16;
    if (e->nr_PhysCellID_r16) r.pci = *e->nr_PhysCellID_r16;
    if (e->nr_ARFCN_r16) r.arfcn = *e->nr_ARFCN_r16;
    auto const pr = e->nr_UE_RxTxTimeDiff_r16.present;
    if (pr < LPP_NR_Multi_RTT_MeasElement_r16__nr_UE_RxTxTimeDiff_r16_PR_k0_r16 ||
        pr > LPP_NR_Multi_RTT_MeasElement_r16__nr_UE_RxTxTimeDiff_r16_PR_k5_r16)
      continue;  // kMinus6..kMinus1 (Rel-18 finer granularity) not requested by this LMF
    r.k        = pr - LPP_NR_Multi_RTT_MeasElement_r16__nr_UE_RxTxTimeDiff_r16_PR_k0_r16;
    r.reported = e->nr_UE_RxTxTimeDiff_r16.choice.k0_r16;  // every kN member is a long at the same place
    r.rxtx_tc  = rxtx_tc(r.reported, r.k);
    r.sfn      = e->nr_TimeStamp_r16.nr_SFN_r16;
    r.slot     = e->nr_TimeStamp_r16.nr_Slot_r16.choice.scs15_r16;  // union of longs
    static constexpr double res_m[] = {0.1, 1, 10, 30};
    r.timing_quality_m = long(
        e->nr_TimingQuality_r16.timingQualityValue_r16 *
        res_m[e->nr_TimingQuality_r16.timingQualityResolution_r16 & 3]);
    if (e->ext2 && e->ext2->nr_NTN_UE_RxTxMeasurements_r18) {
      r.subframe_offset =
          e->ext2->nr_NTN_UE_RxTxMeasurements_r18->nr_NTN_UE_RxTxTimeDiffSubframeOffset_r18;
      r.dl_drift_01ppm =
          e->ext2->nr_NTN_UE_RxTxMeasurements_r18->nr_NTN_DL_TimingDrift_r18;
    }
    if (smi->nr_NTA_Offset_r16 && *smi->nr_NTA_Offset_r16 >= 0 && *smi->nr_NTA_Offset_r16 < 4)
      r.nta_offset_tc = nta_tc[*smi->nr_NTA_Offset_r16];
    m.multi_rtt.push_back(r);
  }
}

}  // namespace

std::string encode_request_location_information(
    long transaction_number, long sequence_number, int k, bool ntn) {
  lpp_ptr msg{alloc<LPP_LPP_Message_t>()};
  msg->transactionID                    = alloc<LPP_LPP_TransactionID_t>();
  msg->transactionID->initiator         = LPP_Initiator_locationServer;
  msg->transactionID->transactionNumber = transaction_number;
  msg->endTransaction                   = 0;  // the target ends it (5.3.1 step 2)
  msg->sequenceNumber                   = alloc<LPP_SequenceNumber_t>();
  *msg->sequenceNumber                  = sequence_number;

  msg->lpp_MessageBody          = alloc<LPP_LPP_MessageBody_t>();
  msg->lpp_MessageBody->present = LPP_LPP_MessageBody_PR_c1;
  alloc_into(msg->lpp_MessageBody->choice.c1);
  auto* c1    = msg->lpp_MessageBody->choice.c1;
  c1->present = LPP_LPP_MessageBody__c1_PR_requestLocationInformation;
  auto* req   = alloc<LPP_RequestLocationInformation_t>();
  c1->choice.requestLocationInformation = req;
  req->criticalExtensions.present =
      LPP_RequestLocationInformation__criticalExtensions_PR_c1;
  alloc_into(req->criticalExtensions.choice.c1);
  auto* rc1    = req->criticalExtensions.choice.c1;
  rc1->present = LPP_RequestLocationInformation__criticalExtensions__c1_PR_requestLocationInformation_r9;
  auto* ies    = alloc<LPP_RequestLocationInformation_r9_IEs_t>();
  rc1->choice.requestLocationInformation_r9 = ies;

  // UE-assisted: measurements, not a location estimate (TS 38.305 8.10.1, 37.355 6.4.1).
  ies->commonIEsRequestLocationInformation =
      alloc<LPP_CommonIEsRequestLocationInformation_t>();
  ies->commonIEsRequestLocationInformation->locationInformationType =
      LPP_LocationInformationType_locationMeasurementsRequired;

  alloc_into(ies->ext2);
  auto* rtt = alloc<LPP_NR_Multi_RTT_RequestLocationInformation_r16_t>();
  ies->ext2->nr_Multi_RTT_RequestLocationInformation_r16 = rtt;
  // nr-RequestedMeasurements: the UE Rx-Tx time difference is always reported; no PRS-RSRP, first-path RSRP
  // or RSCP (bits 0..2 zero).
  rtt->nr_RequestedMeasurements_r16.buf = static_cast<uint8_t*>(calloc(1, 1));
  rtt->nr_RequestedMeasurements_r16.size        = 1;
  rtt->nr_RequestedMeasurements_r16.bits_unused = 5;
  rtt->nr_AssistanceAvailability_r16            = 0;
  alloc_into(rtt->nr_Multi_RTT_ReportConfig_r16.timingReportingGranularityFactor_r16);
  *rtt->nr_Multi_RTT_ReportConfig_r16.timingReportingGranularityFactor_r16 = k;
  if (ntn) {
    alloc_into(rtt->ext3);
    alloc_into(rtt->ext3->nr_NTN_UE_RxTxMeasurementsRequest_r18);
    *rtt->ext3->nr_NTN_UE_RxTxMeasurementsRequest_r18 = 0;  // requested
  }
  return encode(msg.get());
}

std::string encode_provide_assistance_data(
    long transaction_number, long sequence_number, const std::vector<dl_prs_assistance>& trps) {
  if (trps.empty()) return {};
  auto const& ref = trps.front().prs;
  // scs15 is the only numerology here (the NTN bands); its periodicity CHOICE lists n4..n10240 in the NRPPa
  // enumeration's order, so index i is alternative i + 1.
  for (auto const& t : trps) {
    if (t.prs.scs != 0 || t.prs.period < 0 || t.prs.period > 16) return {};
    // One NR-DL-PRS-AssistanceDataPerFreq carries one positioning frequency layer, so every TRP in it has to
    // agree on point A, bandwidth, comb and cyclic prefix. Here they are cells of one NTN carrier, so they do.
    if (t.prs.point_a != ref.point_a || t.prs.bandwidth != ref.bandwidth || t.prs.start_prb != ref.start_prb ||
        t.prs.comb != ref.comb || t.prs.cp != ref.cp)
      return {};
  }

  lpp_ptr msg{alloc<LPP_LPP_Message_t>()};
  msg->transactionID                    = alloc<LPP_LPP_TransactionID_t>();
  msg->transactionID->initiator         = LPP_Initiator_locationServer;
  msg->transactionID->transactionNumber = transaction_number;
  msg->endTransaction                   = 1;  // 5.2.2: delivery is one message
  msg->sequenceNumber                   = alloc<LPP_SequenceNumber_t>();
  *msg->sequenceNumber                  = sequence_number;
  msg->lpp_MessageBody          = alloc<LPP_LPP_MessageBody_t>();
  msg->lpp_MessageBody->present = LPP_LPP_MessageBody_PR_c1;
  alloc_into(msg->lpp_MessageBody->choice.c1);
  auto* c1    = msg->lpp_MessageBody->choice.c1;
  c1->present = LPP_LPP_MessageBody__c1_PR_provideAssistanceData;
  auto* pad   = alloc<LPP_ProvideAssistanceData_t>();
  c1->choice.provideAssistanceData = pad;
  pad->criticalExtensions.present  = LPP_ProvideAssistanceData__criticalExtensions_PR_c1;
  alloc_into(pad->criticalExtensions.choice.c1);
  pad->criticalExtensions.choice.c1->present =
      LPP_ProvideAssistanceData__criticalExtensions__c1_PR_provideAssistanceData_r9;
  auto* ies = alloc<LPP_ProvideAssistanceData_r9_IEs_t>();
  pad->criticalExtensions.choice.c1->choice.provideAssistanceData_r9 = ies;
  alloc_into(ies->ext2);
  auto* rtt = alloc<LPP_NR_Multi_RTT_ProvideAssistanceData_r16_t>();
  ies->ext2->nr_Multi_RTT_ProvideAssistanceData_r16 = rtt;
  auto* ad = alloc<LPP_NR_DL_PRS_AssistanceData_r16_t>();
  rtt->nr_DL_PRS_AssistanceData_r16 = ad;
  ad->nr_DL_PRS_ReferenceInfo_r16.dl_PRS_ID_r16 = trps.front().dl_prs_id;

  auto* freq = alloc<LPP_NR_DL_PRS_AssistanceDataPerFreq_r16_t>();
  auto& fl   = freq->nr_DL_PRS_PositioningFrequencyLayer_r16;
  fl.dl_PRS_SubcarrierSpacing_r16 = ref.scs;
  fl.dl_PRS_ResourceBandwidth_r16 = ref.bandwidth;  // same encoding in 38.455 and 37.355: 24 + 4 (v - 1) PRBs
  fl.dl_PRS_StartPRB_r16          = ref.start_prb;
  fl.dl_PRS_PointA_r16            = ref.point_a;
  fl.dl_PRS_CombSizeN_r16         = ref.comb;
  fl.dl_PRS_CyclicPrefix_r16      = ref.cp;

  for (size_t i = 0; i != trps.size(); ++i) {
    auto const& a = trps[i];
    auto const& p = a.prs;
    auto* trp          = alloc<LPP_NR_DL_PRS_AssistanceDataPerTRP_r16_t>();
    trp->dl_PRS_ID_r16 = a.dl_prs_id;
    if (a.pci) {
      trp->nr_PhysCellID_r16  = alloc<LPP_NR_PhysCellID_r16_t>();
      *trp->nr_PhysCellID_r16 = *a.pci;
    }
    if (a.arfcn) {
      trp->nr_ARFCN_r16  = alloc<LPP_ARFCN_ValueNR_r15_t>();
      *trp->nr_ARFCN_r16 = *a.arfcn;
    }
    // SFN0 offset, expected RSTD and uncertainty are all 0 for the reference TRP (6.4.3 NOTE 3). For a
    // neighbour the SFN0 offset says how far its frame grid sits behind the reference's; the expected RSTD is
    // the search window, which is left open here - its resolution is 4 Ts and its range +-500 us, while two
    // LEO satellites over one UE differ by milliseconds, so there is no in-range value to signal. The UE
    // searches its whole PRS occasion instead.
    trp->nr_DL_PRS_SFN0_Offset_r16.sfn_Offset_r16             = i ? a.sfn_offset : 0;
    trp->nr_DL_PRS_SFN0_Offset_r16.integerSubframeOffset_r16  = i ? a.subframe_offset : 0;
    trp->nr_DL_PRS_ExpectedRSTD_r16                           = 0;
    trp->nr_DL_PRS_ExpectedRSTD_Uncertainty_r16               = i ? 246 : 0;

    auto* set = alloc<LPP_NR_DL_PRS_ResourceSet_r16_t>();
    set->nr_DL_PRS_ResourceSetID_r16 = p.set_id;
    auto& per   = set->dl_PRS_Periodicity_and_ResourceSetSlotOffset_r16;
    per.present = LPP_NR_DL_PRS_Periodicity_and_ResourceSetSlotOffset_r16_PR_scs15_r16;
    alloc_into(per.choice.scs15_r16);
    per.choice.scs15_r16->present = static_cast<decltype(per.choice.scs15_r16->present)>(
        LPP_NR_DL_PRS_Periodicity_and_ResourceSetSlotOffset_r16__scs15_r16_PR_n4_r16 + p.period);
    per.choice.scs15_r16->choice.n4_r16 = p.set_slot_offset;  // every nN member is a long at the same place
    if (p.repetition > 0) {  // rf1 is the default, signalled by absence (Need OP)
      set->dl_PRS_ResourceRepetitionFactor_r16  = alloc<long>();
      *set->dl_PRS_ResourceRepetitionFactor_r16 = p.repetition - 1;
      set->dl_PRS_ResourceTimeGap_r16           = alloc<long>();  // Cond Rep
      *set->dl_PRS_ResourceTimeGap_r16          = p.time_gap;
    }
    set->dl_PRS_NumSymbols_r16     = p.nof_symbols;
    set->dl_PRS_ResourcePower_r16  = p.power_dbm;
    auto* res                      = alloc<LPP_NR_DL_PRS_Resource_r16_t>();
    res->nr_DL_PRS_ResourceID_r16  = p.resource_id;
    res->dl_PRS_SequenceID_r16     = p.sequence_id;
    res->dl_PRS_CombSizeN_AndReOffset_r16.present =
        static_cast<decltype(res->dl_PRS_CombSizeN_AndReOffset_r16.present)>(
            LPP_NR_DL_PRS_Resource_r16__dl_PRS_CombSizeN_AndReOffset_r16_PR_n2_r16 + p.comb);
    res->dl_PRS_CombSizeN_AndReOffset_r16.choice.n2_r16 = p.re_offset;
    res->dl_PRS_ResourceSlotOffset_r16                  = p.resource_slot_offset;
    res->dl_PRS_ResourceSymbolOffset_r16                = p.symbol_offset;
    ASN_SEQUENCE_ADD(&set->dl_PRS_ResourceList_r16.list, res);
    ASN_SEQUENCE_ADD(&trp->nr_DL_PRS_Info_r16.nr_DL_PRS_ResourceSetList_r16.list, set);
    ASN_SEQUENCE_ADD(&freq->nr_DL_PRS_AssistanceDataPerFreq_r16.list, trp);
  }
  ASN_SEQUENCE_ADD(&ad->nr_DL_PRS_AssistanceDataList_r16.list, freq);
  return encode(msg.get());
}

std::string encode_request_capabilities(
    long transaction_number, long sequence_number) {
  lpp_ptr msg{alloc<LPP_LPP_Message_t>()};

  msg->transactionID                    = alloc<LPP_LPP_TransactionID_t>();
  msg->transactionID->initiator         = LPP_Initiator_locationServer;
  msg->transactionID->transactionNumber = transaction_number;
  msg->endTransaction                   = 0;
  msg->sequenceNumber                   = alloc<LPP_SequenceNumber_t>();
  *msg->sequenceNumber                  = sequence_number;

  msg->lpp_MessageBody          = alloc<LPP_LPP_MessageBody_t>();
  msg->lpp_MessageBody->present = LPP_LPP_MessageBody_PR_c1;
  alloc_into(msg->lpp_MessageBody->choice.c1);
  auto* c1 = msg->lpp_MessageBody->choice.c1;
  c1->present = LPP_LPP_MessageBody__c1_PR_requestCapabilities;

  auto* req                      = alloc<LPP_RequestCapabilities_t>();
  c1->choice.requestCapabilities = req;
  req->criticalExtensions.present =
      LPP_RequestCapabilities__criticalExtensions_PR_c1;
  alloc_into(req->criticalExtensions.choice.c1);
  auto* rc1 = req->criticalExtensions.choice.c1;
  rc1->present = LPP_RequestCapabilities__criticalExtensions__c1_PR_requestCapabilities_r9;
  auto* ies                          = alloc<LPP_RequestCapabilities_r9_IEs_t>();
  rc1->choice.requestCapabilities_r9 = ies;
  // NR Multi-RTT is the method this LMF positions with (TS 38.305 8.10); ask only for it.
  alloc_into(ies->ext2);
  ies->ext2->nr_Multi_RTT_RequestCapabilities_r16 =
      alloc<LPP_NR_Multi_RTT_RequestCapabilities_r16_t>();

  auto enc = asn_encode_to_new_buffer(
      nullptr, ATS_UNALIGNED_BASIC_PER, &asn_DEF_LPP_LPP_Message, msg.get());
  if (enc.buffer == nullptr) return {};
  std::string pdu(static_cast<char*>(enc.buffer), enc.result.encoded);
  free(enc.buffer);
  return pdu;
}

std::optional<message> decode(const std::string& pdu, std::string& error) {
  LPP_LPP_Message_t* raw = nullptr;
  auto rv                = asn_decode(
      nullptr, ATS_UNALIGNED_BASIC_PER, &asn_DEF_LPP_LPP_Message,
      reinterpret_cast<void**>(&raw), pdu.data(), pdu.size());
  lpp_ptr msg{raw};
  if (rv.code != RC_OK) {
    error = "UPER decode failed after " + std::to_string(rv.consumed) +
            " of " + std::to_string(pdu.size()) + " bytes";
    return std::nullopt;
  }

  message m;
  if (msg->transactionID) {
    m.transaction_initiator = msg->transactionID->initiator;
    m.transaction_number    = msg->transactionID->transactionNumber;
  }
  m.end_transaction = msg->endTransaction != 0;
  if (msg->sequenceNumber) m.sequence_number = *msg->sequenceNumber;
  if (msg->acknowledgement) {
    m.ack_requested = msg->acknowledgement->ackRequested != 0;
    if (msg->acknowledgement->ackIndicator)
      m.ack_indicator = *msg->acknowledgement->ackIndicator;
  }
  m.body = body_name(msg.get());
  if (m.body == "provideCapabilities") {
    auto* prov = msg->lpp_MessageBody->choice.c1->choice.provideCapabilities;
    if (prov && prov->criticalExtensions.present ==
                    LPP_ProvideCapabilities__criticalExtensions_PR_c1 &&
        prov->criticalExtensions.choice.c1 &&
        prov->criticalExtensions.choice.c1->present ==
            LPP_ProvideCapabilities__criticalExtensions__c1_PR_provideCapabilities_r9) {
      auto* ies =
          prov->criticalExtensions.choice.c1->choice.provideCapabilities_r9;
      m.nr_multi_rtt_capable =
          ies && ies->ext2 && ies->ext2->nr_Multi_RTT_ProvideCapabilities_r16;
      if (m.nr_multi_rtt_capable) {
        auto const& mc = ies->ext2->nr_Multi_RTT_ProvideCapabilities_r16
                             ->nr_Multi_RTT_MeasurementCapability_r16;
        if (mc.ext1 && mc.ext1->multi_RTT_MeasCapabilityBandList_r17) {
          auto const& l = mc.ext1->multi_RTT_MeasCapabilityBandList_r17->list;
          for (int i = 0; i < l.count; i++)
            if (l.array[i]->ext1 && l.array[i]->ext1->nr_NTN_MeasAndReport_r18)
              m.nr_ntn_meas_and_report = true;
        }
      }
    }
  }
  if (m.body == "provideLocationInformation") {
    auto* prov = msg->lpp_MessageBody->choice.c1->choice.provideLocationInformation;
    if (prov && prov->criticalExtensions.present ==
                    LPP_ProvideLocationInformation__criticalExtensions_PR_c1 &&
        prov->criticalExtensions.choice.c1 &&
        prov->criticalExtensions.choice.c1->present ==
            LPP_ProvideLocationInformation__criticalExtensions__c1_PR_provideLocationInformation_r9 &&
        prov->criticalExtensions.choice.c1->choice.provideLocationInformation_r9)
      decode_multi_rtt(
          prov->criticalExtensions.choice.c1->choice.provideLocationInformation_r9, m);
  }
  m.xer = to_xer(msg.get());
  return m;
}

}  // namespace oai::lmf::app::lpp
