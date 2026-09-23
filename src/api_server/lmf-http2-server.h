/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#ifndef FILE_LMF_HTTP2_SERVER_SEEN
#define FILE_LMF_HTTP2_SERVER_SEEN

#include "conversions.hpp"

#include "lmf_app.hpp"
#include <nghttp2/asio_http2_server.h>
#include "InputData.h"

using namespace nghttp2::asio_http2;
using namespace nghttp2::asio_http2::server;
// using namespace oai::_3gpp::model;
using namespace oai::lmf::app;

class lmf_http2_server {
 public:
  lmf_http2_server(
      std::string addr, uint32_t port, unsigned num_threads,
      lmf_app* lmf_app_inst)
      : m_address(addr),
        m_port(port),
        m_num_threads(num_threads),
        server(),
        m_lmf_app(lmf_app_inst) {}
  void start();
  void non_ue_n2info_nrppa_notification_post_handler(
      std::vector<mime_part>& parts, const response& response);
  // TS 29.518 5.2.2.3.5 N1MessageNotify for n1MessageClass LPP
  void n1_lpp_notification_post_handler(
      const std::string& ueContextId, std::vector<mime_part>& parts,
      const nghttp2::asio_http2::server::response& response);
  void n2info_nrppa_notification_post_handler(
      const std::string& ueContextId, std::vector<mime_part>& parts,
      const response& response);
  void init(size_t thr) {}

  void detemine_location_post_handler(
      const oai::_3gpp::model::InputData& inputData, const response& response);

  void stop();

 private:
  std::string m_address;
  uint32_t m_port;
  unsigned m_num_threads;
  http2 server;
  lmf_app* m_lmf_app;
  bool running_server;
};

#endif
