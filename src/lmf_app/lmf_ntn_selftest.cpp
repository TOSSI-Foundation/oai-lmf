// Self-test of lmf_ntn's geometry and solver, runnable on a host without the LMF's dependencies:
//   g++ -std=c++17 -O2 -DLMF_NTN_NO_OAM lmf_ntn.cpp lmf_ntn_selftest.cpp -o /tmp/lmf_ntn_selftest && /tmp/lmf_ntn_selftest
// Checks: (1) the propagator reproduces the round trip the OCUDU gNB emulated for this ephemeris (its log of a run,
// "Emulated NTN channel updated ... rx_delay=", whole microseconds, so within 1 us); (2) the solver recovers a UE
// from exact ranges over a pass, and from ranges with 20 m noise.
#include "lmf_ntn.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>

using namespace oai::lmf::app::ntn;

static double dist(vec3 a, vec3 b) { return std::sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2])); }

int main() {
  setvbuf(stdout, nullptr, _IONBF, 0);
  satellite sat;  // configs/ntn/leo_rfsim_gnb.yml ephemeris_info_ecef
  sat.epoch_unix = 1e9;
  sat.r_ecef = {0, -2826323.2, 6356752.3};
  sat.v_ecef = {0, 6916.62, 3075.25};
  vec3 const pole{0, 0, 6356752.314245};
  // gNB log, epoch 19:58:54: rx_delay 19323 us at 19:58:43.860, 13445 us at 19:59:59... (t - epoch -10.14, +117.67 s)
  for (auto [dt, us] : {std::pair{-10.139979, 19323.0}, std::pair{117.672, 13445.0}}) {
    double const rtt = 2 * dist(ecef_at(sat, sat.epoch_unix + dt), pole) / 299792458.0 * 1e6;
    std::printf("t-epoch %+.2f s: RTT %.3f us, gNB emulated %.0f us\n", dt, rtt, us);
    assert(rtt - us >= 0 && rtt - us < 1);
  }
  // Zenith comes at T+384 s by the config's account: the range minimum should be ~600 km there.
  double best = 1e18, tbest = 0;
  for (double t = 0; t < 800; t += 0.5) {
    double const r = dist(ecef_at(sat, sat.epoch_unix + t), pole);
    if (r < best) best = r, tbest = t;
  }
  std::printf("closest %.1f km at T+%.1f s (config: zenith at T+384 s)\n", best / 1e3, tbest);
  assert(std::fabs(tbest - 384) < 3);

  // rate: give every measurement the range rate the UE's DL timing drift would report, quantised to its
  // 0.1 ppm (30 m/s) reporting step, and see what it does to a partial pass.
  for (auto [span, noise] : {std::pair{700.0, 0.0}, std::pair{700.0, 20.0}, std::pair{180.0, 20.0}, std::pair{90.0, 20.0}}) {
    std::mt19937 rng(1);
    std::normal_distribution<double> nd(0, noise > 0 ? noise : 1e-9);
    std::vector<range_meas> m;
    for (double t = 0; t < span; t += 5) {
      vec3 const s = ecef_at(sat, sat.epoch_unix + t);
      m.push_back({sat.epoch_unix + t, dist(s, pole) + nd(rng), s});
    }
    auto const fixes = solve(m);
    assert(!fixes.empty());
    auto const& f = fixes.front();
    double const err = dist(f.ecef, pole);
    // The second solution is the mirror across the ground track: how much worse it fits is what decides whether
    // the fix is ambiguous.
    double const alt_rms = fixes.size() > 1 ? fixes[1].rms_m : 0;
    double const alt_err = fixes.size() > 1 ? dist(fixes[1].ecef, pole) : 0;
    std::printf("pass 0..%3.0f s, noise %4.0f m: %zu solution(s), error %8.1f m, rms %5.1f m, 1-sigma %6.0f x %.0f m"
                " | alt: error %9.1f m, rms %7.1f m (%.0fx worse)\n",
                span, noise, fixes.size(), err, f.rms_m, f.semi_major_m, f.semi_minor_m,
                alt_err, alt_rms, f.rms_m > 0 ? alt_rms / f.rms_m : 0);
    assert(err < (noise > 0 ? 3 * f.semi_major_m + 50 : 1.0));
  }
  // Two satellites at once (configs/ntn/leo_satswitch_xplane.yml: satellite 1 rotated 90 degrees about the
  // Earth's axis). One satellite needs a pass to separate the UE from its mirror across the ground track; two
  // cross each other's tracks, so a handful of rounds over a few seconds should already place it.
  satellite sat2 = sat;
  sat2.r_ecef = {2826323.2, 0, 6356752.3};   // (x, y) -> (-y, x) on position and velocity
  sat2.v_ecef = {-6916.62, 0, 3075.25};
  {
    std::mt19937 rng{7};
    std::normal_distribution<double> noise{0.0, 20.0};
    std::vector<range_meas> m;
    for (double t = 0; t <= 20; t += 5) {           // 5 rounds, 20 s - no pass to speak of
      for (auto const& [id, s] : {std::pair{0, sat}, std::pair{1, sat2}}) {
        auto const p = ecef_at(s, s.epoch_unix + t);
        m.push_back({s.epoch_unix + t, dist(p, pole) + noise(rng), p, id});
      }
    }
    auto const fixes = solve(m);
    assert(!fixes.empty());
    auto const& f = fixes.front();
    double const err = dist(f.ecef, pole);
    std::printf("two satellites, %zu rounds over 20 s, noise 20 m: %zu solution(s), error %.1f m, rms %.1f m, "
                "1-sigma %.0f x %.0f m\n", m.size(), fixes.size(), err, f.rms_m, f.semi_major_m, f.semi_minor_m);
    // The point of the second satellite: one fix, not two candidates a pass apart.
    assert(err < 100);
    assert(fixes.size() == 1 || fixes[1].rms_m > 2 * f.rms_m);
  }

  std::printf("OK\n");
}
