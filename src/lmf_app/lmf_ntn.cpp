#include "lmf_ntn.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <mutex>
#include <sys/stat.h>

#ifndef LMF_NTN_NO_OAM  // host self-test build (lmf_ntn_selftest.cpp) has no JSON library
#include <nlohmann/json.hpp>
#endif

namespace oai::lmf::app::ntn {

namespace {

constexpr double MU      = 3.986004418e14;      // m^3/s^2, as the gNB (ntn_orbital_constants.h)
constexpr double OMEGA_E = 7.292115146706979e-5; // rad/s, as the gNB (reference_frame_converter.cpp)
constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_B = 6356752.314245;

vec3 add(vec3 a, vec3 b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
vec3 sub(vec3 a, vec3 b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
vec3 mul(vec3 a, double k) { return {a[0] * k, a[1] * k, a[2] * k}; }
double dot(vec3 a, vec3 b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
double norm(vec3 a) { return std::sqrt(dot(a, a)); }
vec3 cross(vec3 a, vec3 b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

// Point on the WGS84 surface below p, along the ellipsoid normal (one Newton-free scaling is enough for a start).
vec3 to_surface(vec3 p) {
  double const s = 1.0 / std::sqrt((p[0] * p[0] + p[1] * p[1]) / (WGS84_A * WGS84_A) + p[2] * p[2] / (WGS84_B * WGS84_B));
  return mul(p, s);
}

// Height-like constraint: b (sqrt(x^2/a^2 + y^2/a^2 + z^2/b^2) - 1), ~ geodetic height near the surface.
double surface_residual(vec3 p, vec3& grad) {
  double const q = std::sqrt((p[0] * p[0] + p[1] * p[1]) / (WGS84_A * WGS84_A) + p[2] * p[2] / (WGS84_B * WGS84_B));
  grad = {WGS84_B * p[0] / (WGS84_A * WGS84_A * q), WGS84_B * p[1] / (WGS84_A * WGS84_A * q),
          WGS84_B * p[2] / (WGS84_B * WGS84_B * q)};
  return WGS84_B * (q - 1);
}

bool solve3(double A[3][3], const double b[3], double x[3]) {
  double const det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                     A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                     A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  if (std::fabs(det) < 1e-30) return false;
  for (int c = 0; c < 3; c++) {
    double M[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) M[i][j] = j == c ? b[i] : A[i][j];
    x[c] = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
            M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) /
           det;
  }
  return true;
}

bool invert3(double A[3][3], double inv[3][3]) {
  for (int c = 0; c < 3; c++) {
    double e[3] = {0, 0, 0}, x[3];
    e[c] = 1;
    if (!solve3(A, e, x)) return false;
    for (int r = 0; r < 3; r++) inv[r][c] = x[r];
  }
  return true;
}

double parse_epoch(const std::string& s) {
  std::tm tm{};
  if (!strptime(s.c_str(), "%Y-%m-%dT%H:%M:%S", &tm)) return NAN;
  return double(timegm(&tm));
}

std::mutex m_cache;
std::string cache_path;
time_t cache_mtime = 0;
std::vector<satellite> cache;

}  // namespace

#ifndef LMF_NTN_NO_OAM
std::optional<satellite> satellite_for(uint64_t gnb_id, long trp_id, double t_unix, std::string& error) {
  char const* path = std::getenv("LMF_NTN_SATELLITES");
  if (!path || !*path) {
    error = "LMF_NTN_SATELLITES not set: no OAM satellite information";
    return std::nullopt;
  }
  struct stat st {};
  if (stat(path, &st) != 0) {
    error = std::string{"OAM satellite file "} + path + " not found";
    return std::nullopt;
  }
  std::scoped_lock lk{m_cache};
  if (cache_path != path || cache_mtime != st.st_mtime) {
    std::ifstream f{path};
    auto const j = nlohmann::json::parse(f, nullptr, false);
    if (j.is_discarded() || !j.contains("satellites")) {
      error = std::string{"OAM satellite file "} + path + " is not valid";
      return std::nullopt;
    }
    std::vector<satellite> sats;
    for (auto const& s : j["satellites"]) {
      satellite sat;
      sat.id         = s.value("id", 0);
      sat.epoch_unix = parse_epoch(s.value("epoch", ""));
      for (int i = 0; i < 3; i++) {
        sat.r_ecef[i] = s.at("ecefPositionM").at(i).get<double>();
        sat.v_ecef[i] = s.at("ecefVelocityMS").at(i).get<double>();
      }
      sat.common_delay_us    = s.value("commonDelayUs", 0.0);
      sat.rtt_calibration_ns = s.value("rttCalibrationNs", 0.0);
      sat.serves_from        = s.contains("servesFrom") ? parse_epoch(s.value("servesFrom", "")) : 0.0;
      sat.serves_until       = s.contains("servesUntil") ? parse_epoch(s.value("servesUntil", "")) : 0.0;
      if (std::isnan(sat.serves_from) || std::isnan(sat.serves_until)) {
        error = "OAM satellite " + std::to_string(sat.id) + ": bad servesFrom / servesUntil";
        return std::nullopt;
      }
      for (auto const& t : s.value("trps", nlohmann::json::array()))
        sat.trps.emplace_back(t.at("gnbId").get<uint64_t>(), t.at("trpId").get<long>());
      if (std::isnan(sat.epoch_unix)) {
        error = "OAM satellite " + std::to_string(sat.id) + ": bad epoch";
        return std::nullopt;
      }
      sats.push_back(sat);
    }
    cache       = sats;
    cache_path  = path;
    cache_mtime = st.st_mtime;
  }
  bool associated = false;
  for (auto const& s : cache)
    for (auto const& [g, t] : s.trps) {
      if (g != gnb_id || t != trp_id) continue;
      associated = true;
      if ((s.serves_from == 0 || t_unix >= s.serves_from) && (s.serves_until == 0 || t_unix < s.serves_until))
        return s;
    }
  error = associated ? "no OAM satellite carries gnbId " + std::to_string(gnb_id) + " trpId " +
                           std::to_string(trp_id) + " at this instant"
                     : "no OAM satellite associated with gnbId " + std::to_string(gnb_id) + " trpId " +
                           std::to_string(trp_id);
  return std::nullopt;
}

#endif

vec3 ecef_at(const satellite& sat, double t_unix) {
  double const dt = t_unix - sat.epoch_unix;
  // Inertial frame = ECEF at the epoch: v_inertial = v_ecef + omega x r.
  vec3 const r0 = sat.r_ecef;
  vec3 const v0 = add(sat.v_ecef, cross({0, 0, OMEGA_E}, r0));
  // Two-body, exact: Kepler's equation in the eccentric-anomaly difference and Lagrange f, g (elliptic orbits).
  double const rn = norm(r0), v2 = dot(v0, v0);
  double const a = 1.0 / (2.0 / rn - v2 / MU);
  double const sa = std::sqrt(a), sigma0 = dot(r0, v0) / std::sqrt(MU);
  double const n = std::sqrt(MU / (a * a * a));
  double const M = n * dt;
  double dE = M;
  for (int i = 0; i < 50; i++) {
    double const F  = dE + sigma0 / sa * (1 - std::cos(dE)) - (1 - rn / a) * std::sin(dE) - M;
    double const Fp = 1 + sigma0 / sa * std::sin(dE) - (1 - rn / a) * std::cos(dE);
    double const step = F / Fp;
    dE -= step;
    if (std::fabs(step) < 1e-13) break;
  }
  double const f = 1 - a / rn * (1 - std::cos(dE));
  double const g = dt + std::sqrt(a * a * a / MU) * (std::sin(dE) - dE);
  vec3 const ri  = add(mul(r0, f), mul(v0, g));
  // Back to ECEF: the Earth has turned by omega dt.
  double const ang = -OMEGA_E * dt, c = std::cos(ang), s = std::sin(ang);
  return {ri[0] * c - ri[1] * s, ri[0] * s + ri[1] * c, ri[2]};
}

std::vector<fix> solve(const std::vector<range_meas>& m) {
  std::vector<fix> out;
  if (m.size() < 3) return out;
  // Starts: the sub-satellite point at the shortest range, and points either side of the ground track there -
  // one pass leaves the side of the track to the curvature of the range profile, so try both.
  auto const closest = std::min_element(m.begin(), m.end(), [](auto& a, auto& b) { return a.range_m < b.range_m; });
  vec3 const p0    = to_surface(closest->sat);
  vec3 const track = sub(m.back().sat, m.front().sat);
  vec3 const up    = mul(p0, 1 / norm(p0));
  vec3 across      = cross(up, track);
  across           = mul(across, 1 / norm(across));
  double const w_surface = 400;  // (20 m range sigma / 1 m height sigma)^2: the surface is a hard constraint

  for (double off_km : {-800.0, -200.0, 200.0, 800.0}) {
    vec3 p = to_surface(add(p0, mul(across, off_km * 1e3)));
    double A[3][3], b[3];
    int it = 0;
    bool ok = false;
    for (; it < 60; it++) {
      for (auto& r : A) r[0] = r[1] = r[2] = 0;
      b[0] = b[1] = b[2] = 0;
      auto accumulate = [&](vec3 J, double r, double w) {
        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) A[i][j] += w * J[i] * J[j];
          b[i] -= w * J[i] * r;
        }
      };
      for (auto const& x : m) {
        vec3 const d  = sub(x.sat, p);
        double const dn = norm(d);
        accumulate(mul(d, 1 / dn), x.range_m - dn, 1);
      }
      vec3 gh;
      double const h = surface_residual(p, gh);
      accumulate(gh, h, w_surface);
      double dx[3];
      if (!solve3(A, b, dx)) break;
      p = add(p, {dx[0], dx[1], dx[2]});
      if (std::fabs(norm(p) - WGS84_B) > 3e6) break;  // diverged off the Earth
      if (std::sqrt(dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]) < 1e-3) {
        ok = true;
        break;
      }
    }
    if (!ok) continue;

    fix f;
    f.ecef = p;
    f.n = int(m.size());
    f.iterations = it + 1;
    f.span_s = m.back().t_unix - m.front().t_unix;
    double ss = 0;
    for (auto const& x : m) {
      double const r = x.range_m - norm(sub(x.sat, p));
      ss += r * r;
    }
    f.rms_m = std::sqrt(ss / m.size());
    double const lon = std::atan2(p[1], p[0]);
    double const e2  = 1 - WGS84_B * WGS84_B / (WGS84_A * WGS84_A);
    double const lat = std::atan2(p[2], (1 - e2) * std::hypot(p[0], p[1]));
    f.lat_deg = lat * 180 / M_PI;
    f.lon_deg = lon * 180 / M_PI;
    // Covariance: sigma^2 (J'WJ)^-1 with sigma^2 from the residuals (n - 2 degrees of freedom on the surface),
    // projected on the local east / north plane.
    double inv[3][3];
    if (invert3(A, inv) && m.size() > 2) {
      double const s2 = ss / double(m.size() - 2);
      vec3 const E{-std::sin(lon), std::cos(lon), 0};
      vec3 const N{-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat)};
      auto q = [&](vec3 u, vec3 v) {
        double acc = 0;
        for (int i = 0; i < 3; i++)
          for (int j = 0; j < 3; j++) acc += u[i] * inv[i][j] * v[j];
        return acc * s2;
      };
      double const cee = q(E, E), cnn = q(N, N), cen = q(E, N);
      double const tr = cee + cnn, disc = std::sqrt(std::max(0.0, (cee - cnn) * (cee - cnn) / 4 + cen * cen));
      f.semi_major_m = std::sqrt(std::max(0.0, tr / 2 + disc));
      f.semi_minor_m = std::sqrt(std::max(0.0, tr / 2 - disc));
      f.orient_deg   = std::atan2(2 * cen, cnn - cee) / 2 * 180 / M_PI;  // major axis, clockwise from north
    }
    bool dup = false;
    for (auto const& o : out) dup |= norm(sub(o.ecef, f.ecef)) < 1000;
    if (!dup) out.push_back(f);
  }
  std::sort(out.begin(), out.end(), [](auto& a, auto& b) { return a.rms_m < b.rms_m; });
  return out;
}

double relative_time_1900_to_unix(uint64_t v) {
  return double(v >> 32) - 2208988800.0 + double(v & 0xffffffffULL) / 4294967296.0;
}

}  // namespace oai::lmf::app::ntn
