/*
 * LPP (TS 37.355 v18.7.0) messages the LMF sends and receives over N1, encoded with the Rel-18 codec in
 * src/lpp (generated from src/37355-i70.asn with the LPP_ prefix). LPP is BASIC-PER, UNALIGNED (37.355 6.1).
 */
#pragma once

#include <optional>
#include <string>
#include <vector>

namespace oai::lmf::app::lpp {

// One TRP's DL-PRS, as the gNB reported it in the NRPPa TRP Information PRS Configuration (TS 38.455
// 9.2.44): one resource set with one resource. Enumerations keep their NRPPa index; 37.355 uses the same
// order for all of them except the repetition factor (rf1 there is "absent" here).
struct dl_prs {
  long set_id = 0, scs = 0, bandwidth = 0, start_prb = 0, point_a = 0, comb = 0, cp = 0;
  long period = 0, set_slot_offset = 0, repetition = 0, time_gap = 0, nof_symbols = 0, power_dbm = 0;
  long resource_id = 0, sequence_id = 0, re_offset = 0, resource_slot_offset = 0, symbol_offset = 0;
};

// One TRP of NR-DL-PRS-AssistanceData (37.355 6.4.3). The first of the list is the reference TRP, whose SFN0
// offset, expected RSTD and uncertainty are all 0 (37.355 6.4.3 NOTE 3).
struct dl_prs_assistance {
  long dl_prs_id = 0;
  std::optional<long> pci;
  std::optional<long> arfcn;
  // nr-DL-PRS-SFN0-Offset: how far this TRP's SFN 0 is behind the reference TRP's, in frames and subframes.
  long sfn_offset = 0, subframe_offset = 0;
  dl_prs prs;
};

// One NR-Multi-RTT-MeasElement of a ProvideLocationInformation (37.355 6.5.12.4).
struct multi_rtt_meas {
  long dl_prs_id = 0;
  std::optional<long> pci;
  std::optional<long> arfcn;
  int k = 0;                   // reporting granularity of nr-UE-RxTxTimeDiff (k0..k5)
  long reported = 0;           // the reported value, TS 38.133 10.1.25.3.1
  double rxtx_tc = 0;          // decoded: the middle of the reported interval, Tc
  long sfn = 0, slot = -1;     // nr-TimeStamp
  long timing_quality_m = 0;   // nr-TimingQuality, metres (resolution folded in)
  std::optional<long> subframe_offset;  // nr-NTN-UE-RxTxMeasurements-r18
  std::optional<long> dl_drift_01ppm;
  std::optional<long> nta_offset_tc;    // nr-NTA-Offset
};

// Header fields every LPP message may carry (37.355 6.2 LPP-Message), plus what the body turned out to be.
struct message {
  std::optional<long> transaction_initiator;  // 0 locationServer, 1 targetDevice
  std::optional<long> transaction_number;
  bool end_transaction = false;
  std::optional<long> sequence_number;   // duplicate detection, 4.3.2
  std::optional<long> ack_indicator;     // acknowledgement response, 4.3.3
  bool ack_requested = false;
  std::string body;                      // "provideCapabilities", "requestCapabilities", ..., or "none"
  bool nr_multi_rtt_capable = false;     // ProvideCapabilities carries nr-Multi-RTT-ProvideCapabilities-r16
  bool nr_ntn_meas_and_report = false;   // ... with nr-NTN-MeasAndReport-r18 on some band
  std::vector<multi_rtt_meas> multi_rtt;          // ProvideLocationInformation, NR Multi-RTT measurements
  std::optional<long> multi_rtt_error;            // ... or its nr-Multi-RTT-Error target device cause
  std::optional<long> location_error;             // ... or commonIEs locationError cause
  std::string xer;                       // the whole message as XER, for the log
};

// RequestCapabilities asking for the NR Multi-RTT capabilities (37.355 5.1.1 step 1): transaction initiated
// by the location server, endTransaction FALSE (the target ends it, 5.1.1 step 2), and a sequence number as
// every message of a location session carries (4.3.2). No acknowledgement is requested.
std::string encode_request_capabilities(long transaction_number, long sequence_number);

// RequestLocationInformation for UE-assisted NR Multi-RTT (37.355 5.3.1 step 1, 6.5.12.5): measurements
// required, the UE Rx-Tx time difference at granularity k, and in NTN the Rel-18 subframe offset and DL
// timing drift (nr-NTN-UE-RxTxMeasurementsRequest-r18).
std::string encode_request_location_information(
    long transaction_number, long sequence_number, int k, bool ntn);

// ProvideAssistanceData with NR-Multi-RTT-ProvideAssistanceData (37.355 5.2.2 Assistance Data Delivery):
// server-initiated, endTransaction TRUE. trps[0] is the reference TRP; the rest are neighbours on the same
// frequency layer, which in NTN means the other satellites serving the UE (TS 38.305 5.4.2, 5.4.4). Empty
// string if the list is empty or a PRS cannot be expressed (not 15 kHz, or a periodicity outside the scs15
// set), or if the TRPs do not share one frequency layer.
std::string encode_provide_assistance_data(
    long transaction_number, long sequence_number, const std::vector<dl_prs_assistance>& trps);

// Decode any uplink LPP message. nullopt, with the reason in error, if it does not decode.
std::optional<message> decode(const std::string& pdu, std::string& error);

}  // namespace oai::lmf::app::lpp
