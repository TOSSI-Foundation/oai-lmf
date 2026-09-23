/*
 * Single-satellite NTN Multi-RTT location (TS 38.305 8.10, network verification of UE location in NTN): the
 * round trips measured at one TRP at different time instances, the satellite's position at each instant, and a
 * least-squares UE position on the Earth's surface.
 *
 * The satellite information comes from OAM, as 38.305 5.4 has it ("the LMF is configured by the OAM with
 * satellite related information ... as well as the association between TRP(s) and satellite(s)"): a JSON file
 * named by LMF_NTN_SATELLITES, re-read whenever it changes. Its ephemeris is an ECEF state vector at an epoch,
 * as SIB19's positionVelocity-r17, propagated as a two-body orbit in an inertial frame aligned with ECEF at the
 * epoch, the Earth turning under it at 7.2921151467e-5 rad/s.
 *
 *   {"satellites": [{"id": 0, "epoch": "2026-09-18T19:58:54Z",
 *                    "ecefPositionM": [x, y, z], "ecefVelocityMS": [vx, vy, vz],
 *                    "trps": [{"gnbId": 411, "trpId": 1}],
 *                    "commonDelayUs": 0, "rttCalibrationNs": 0,
 *                    "servesFrom": "2026-09-12T19:21:45Z", "servesUntil": "2026-09-12T19:25:57Z"}]}
 *
 * servesFrom / servesUntil are optional and bound when that satellite carries the TRP: a satellite switch
 * (TS 38.331 satSwitchWithReSync-r18) hands the same cell from one satellite to the next, so the association is
 * a function of time, not just of the TRP.
 *
 * commonDelayUs: the part of the RTT not on the service link (ta-Common / feeder link), removed before ranging.
 * rttCalibrationNs: a per-TRP RTT correction, the NTN counterpart of LMF_TRP_DELAY_NS.
 */
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace oai::lmf::app::ntn {

using vec3 = std::array<double, 3>;

struct satellite {
  int id = 0;
  double epoch_unix = 0;  // s since 1970, UTC
  vec3 r_ecef{}, v_ecef{};
  double common_delay_us = 0, rtt_calibration_ns = 0;
  double serves_from = 0, serves_until = 0;     // s since 1970; 0 = unbounded
  std::vector<std::pair<uint64_t, long>> trps;  // (gnbId, trpId)
};

// The satellite carrying (gnb_id, trp_id) at t_unix, from the OAM file; nullopt if unconfigured, not
// associated, or no satellite covers that instant.
std::optional<satellite> satellite_for(uint64_t gnb_id, long trp_id, double t_unix, std::string& error);

// ECEF position of the satellite at t_unix.
vec3 ecef_at(const satellite& sat, double t_unix);

struct range_meas {
  double t_unix;   // instant of the round trip (the SRS the gNB Rx-Tx was measured on)
  double range_m;  // service-link range, c (RTT - common delay - calibration) / 2
  vec3 sat;        // satellite ECEF at t_unix
  int sat_id = 0;  // which satellite it is: two at one instant fix the UE without waiting for a pass
};

// NOT used, and measured to be useless here: the UE's DL timing drift (TS 38.215 5.1.47) is a range rate, but a
// range rate over the same arc is the derivative of these ranges, so it carries no geometry they do not already
// have - two positions that fit the ranges fit their derivatives too, the mirror across the ground track
// included. Measured on a synthetic pass: the wrong side's residual was unchanged (36.5 m at 180 s, 21.6 m at
// 90 s) and the fix improved only from 290 m to 231 m, which is within the run-to-run scatter. At 0.1 ppm the
// drift is also 30 m/s per step, far coarser than differencing the ranges. What does break the ambiguity is the
// curvature of the ground track in Earth-fixed coordinates, i.e. letting the pass mature.

struct fix {
  vec3 ecef{};
  double lat_deg = 0, lon_deg = 0;
  double rms_m = 0;                                        // range residual RMS
  double semi_major_m = 0, semi_minor_m = 0, orient_deg = 0;  // 1-sigma horizontal ellipse, orientation from north
  int n = 0, iterations = 0;
  double span_s = 0;
};

// Least squares on the WGS84 surface (height 0), from every start given; each converged solution is returned,
// best first. More than one distinct solution with similar RMS means the pass geometry leaves the side of the
// ground track ambiguous.
std::vector<fix> solve(const std::vector<range_meas>& m);

// Relative Time 1900 (TS 38.455 / 38.473 9.3.1.183: 32.32 fixed point, s since 1900) to s since 1970.
double relative_time_1900_to_unix(uint64_t v);

}  // namespace oai::lmf::app::ntn
