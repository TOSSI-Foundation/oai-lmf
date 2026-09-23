/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#include "lmf_sbi_helper.hpp"

#include <fmt/args.h>

#include <boost/algorithm/string.hpp>
#include <regex>
#include <string>
#include <vector>

#include "ProblemDetails.h"
#include "logger.hpp"

namespace oai::lmf::api {

namespace {
// Callback URIs this LMF hands to the AMF (n2NotifyCallbackUri, n1NotifyCallbackUri) are of OpenAPI type Uri:
// absolute, scheme included (TS 29.571 Uri, RFC 3986). get_ipv4_root() has no scheme, and a strict AMF rejects
// such a callback ("no scheme / not absolute") and never notifies. This LMF serves them over HTTP/2 without TLS.
std::string callback_root(const interface_cfg_t& sbi) {
  const std::string root = sbi.get_ipv4_root();
  return root.find("://") == std::string::npos ? "http://" + root : root;
}
}  // namespace

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_amf_comm_api_root(
    const nf_addr_t& amf_addr, std::string& api_root) {
  api_root = amf_addr.uri_root + sbi_helper::AmfCommBase + amf_addr.api_version;
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_amf_comm_n1n2_message_subscribe_uri(
    const nf_addr_t& amf_addr, const std::string& ue_context_id,
    std::string& uri) {
  std::string amf_api_root = {};
  get_amf_comm_api_root(amf_addr, amf_api_root);

  std::string path_str = {};
  get_fmt_format_form(
      sbi_helper::AmfCommPathUeContextContextIdN1N2MessageSubscriptions,
      path_str);
  uri = amf_api_root + fmt::format(path_str, ue_context_id);
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_amf_comm_n1n2_message_un_subscribe_uri(
    const nf_addr_t& amf_addr, const std::string& ue_context_id,
    const std::string& nf_instance, std::string& uri) {
  std::string amf_api_root = {};
  get_amf_comm_api_root(amf_addr, amf_api_root);

  std::string path_str = {};
  get_fmt_format_form(
      sbi_helper::
          AmfCommPathUeContextContextIdN1N2MessageSubscriptionsSubscriptionId,
      path_str);
  uri = amf_api_root + fmt::format(path_str, ue_context_id, nf_instance);
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_amf_comm_n1n2_message_transfer_uri(
    const nf_addr_t& amf_addr, const std::string& ue_context_id,
    std::string& uri) {
  std::string amf_api_root = {};
  get_amf_comm_api_root(amf_addr, amf_api_root);

  std::string path_str = {};
  get_fmt_format_form(
      sbi_helper::AmfCommPathUeContextContextIdN1N2Message, path_str);
  uri = amf_api_root + fmt::format(path_str, ue_context_id);
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_amf_comm_non_ue_n1n2_message_transfer_uri(
    const nf_addr_t& amf_addr, std::string& uri) {
  std::string amf_api_root = {};
  get_amf_comm_api_root(amf_addr, amf_api_root);

  std::string path_str = {};
  get_fmt_format_form(
      sbi_helper::AmfCommPathNonUeN1N2MessageTransfer, path_str);
  uri = amf_api_root + fmt::format(path_str);
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_amf_comm_non_ue_n2_info_subscribe_uri(
    const nf_addr_t& amf_addr, std::string& uri) {
  std::string amf_api_root = {};
  get_amf_comm_api_root(amf_addr, amf_api_root);

  std::string path_str = {};
  get_fmt_format_form(
      sbi_helper::AmfCommPathNonUeN1N2MessageSubscriptions, path_str);
  uri = amf_api_root + fmt::format(path_str);
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_amf_comm_non_ue_n2_info_un_subscribe_uri(
    const nf_addr_t& amf_addr, const std::string& subscription_id,
    std::string& uri) {
  std::string amf_api_root = {};
  get_amf_comm_api_root(amf_addr, amf_api_root);

  std::string path_str = {};
  get_fmt_format_form(
      sbi_helper::
          AmfCommPathNonUeN1N2MessageSubscriptionsn2NotifySubscriptionId,
      path_str);
  uri = amf_api_root + fmt::format(path_str, subscription_id);
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_lmf_loc_api_root(
    const interface_cfg_t& sbi, std::string& api_root) {
  api_root = sbi.get_ipv4_root() + sbi_helper::LmfLocBase +
             sbi.api_version.value_or(kDefaultSbiApiVersion);
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_lmf_loc_determine_location_uri(
    const interface_cfg_t& sbi, std::string& uri) {
  std::string lmf_api_root = {};
  get_lmf_loc_api_root(sbi, lmf_api_root);

  std::string path_str = {};
  get_fmt_format_form(sbi_helper::LmfLocDetermineLocation, path_str);
  uri = lmf_api_root + fmt::format(path_str);
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_lmf_loc_cancel_location_uri(
    const interface_cfg_t& sbi, std::string& uri) {
  std::string lmf_api_root = {};
  get_lmf_loc_api_root(sbi, lmf_api_root);

  std::string path_str = {};
  get_fmt_format_form(sbi_helper::LmfLocCancelLocation, path_str);
  uri = lmf_api_root + fmt::format(path_str);
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_lmf_loc_location_context_transfer_uri(
    const interface_cfg_t& sbi, std::string& uri) {
  std::string lmf_api_root = {};
  get_lmf_loc_api_root(sbi, lmf_api_root);

  std::string path_str = {};
  get_fmt_format_form(sbi_helper::LmfLocLocationContextTransfer, path_str);
  uri = lmf_api_root + fmt::format(path_str);
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_lmf_n2_info_notify_api_root(
    const interface_cfg_t& sbi, std::string& api_root) {
  api_root = callback_root(sbi) + sbi_helper::LmfN2InfoNotifyBase +
             lmf_cfg.sbi.api_version.value_or(kDefaultSbiApiVersion);
  ;
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_lmf_n2_info_notify_nrppa_callback_uri(
    const interface_cfg_t& sbi, std::string& uri) {
  std::string lmf_api_root = {};
  get_lmf_n2_info_notify_api_root(sbi, lmf_api_root);

  std::string path_str = {};
  get_fmt_format_form(sbi_helper::LmfN2InfoNotifyNrppaCallback, path_str);
  uri = lmf_api_root + fmt::format(path_str);
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_lmf_n2_info_notify_nrppa_callback_uri(
    const interface_cfg_t& sbi, const std::string& supi, std::string& uri) {
  std::string lmf_api_root = {};
  get_lmf_n2_info_notify_api_root(sbi, lmf_api_root);

  std::string path_str = {};
  get_fmt_format_form(
      sbi_helper::LmfN2InfoNotifyNrppaCallbackUeContextId, path_str);
  uri = lmf_api_root + fmt::format(path_str, supi);
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_lmf_non_ue_n2_info_notify_api_root(
    const interface_cfg_t& sbi, std::string& api_root) {
  api_root = callback_root(sbi) + sbi_helper::LmfNonUeN2InfoNotifyBase +
             lmf_cfg.sbi.api_version.value_or(kDefaultSbiApiVersion);
  ;
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_lmf_non_ue_n2_info_notify_nrppa_callback_uri(
    const interface_cfg_t& sbi, std::string& uri) {
  std::string lmf_api_root = {};
  get_lmf_non_ue_n2_info_notify_api_root(sbi, lmf_api_root);

  std::string path_str = {};
  get_fmt_format_form(sbi_helper::LmfNonUeN2InfoNotifyNrppaCallback, path_str);
  uri = lmf_api_root + fmt::format(path_str);
}

void lmf_sbi_helper::throwHttpError(
    std::string const& title, std::string const& detail,
    std::string const& instance, Pistache::Http::Code const& code) {
  oai::_3gpp::model::ProblemDetails pd;
  fmt::dynamic_format_arg_store<fmt::format_context> args;
  std::string fmt;

  pd.setTitle(title);
  args.push_back(title);
  fmt = "{}";

  if (!instance.empty()) {
    pd.setInstance(instance);
    args.push_back(instance);
    fmt += "[{}]";
  }
  pd.setDetail(detail);
  args.push_back(detail);
  fmt += ": {}";

  Logger::lmf_app().error(fmt::vformat(fmt, args));

  nlohmann::json json_data = {};
  to_json(json_data, pd);

  auto const& reason = json_data.dump();
  throw Pistache::Http::HttpError{code, reason};
}

//---------------------------------------------------------------------------------------------
void lmf_sbi_helper::get_lmf_n1_notify_lpp_callback_uri(
    const interface_cfg_t& sbi, const std::string& supi, std::string& uri) {
  uri = callback_root(sbi) + LmfN1NotifyServiceBase + LmfN1NotifyLppCallback + supi;
}

}  // namespace oai::lmf::api
