/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#include "mime_parser.hpp"
#include "logger.hpp"
#include "conversions.hpp"
#include "lmf.h"
#include "utils.hpp"

bool mime_parser::parse(const std::string& str) {
  std::string CRLF = "\r\n";
  Logger::lmf_app().debug("Parsing the message with Simple Parser");

  // find boundary (could be Content-Id, Content-Filename, ...)
  std::size_t content_type_pos = str.find("Content-");  // first part
  if ((content_type_pos <= 4) or (content_type_pos == std::string::npos))
    return false;

  std::string boundary_str =
      str.substr(2, content_type_pos - 4);  // 2 for -- and 2 for CRLF
  Logger::lmf_app().debug("Boundary: %s", boundary_str.c_str());
  std::string boundary_full = "--" + boundary_str + CRLF;
  std::string last_boundary = "--" + boundary_str + "--" + CRLF;

  std::size_t crlf_pos           = str.find(CRLF, content_type_pos);
  std::size_t boundary_pos       = str.find(boundary_full);
  std::size_t boundary_last_post = str.find(last_boundary);

  while (boundary_pos < boundary_last_post) {
    mime_part p      = {};
    content_type_pos = str.find("Content-Type", boundary_pos);
    crlf_pos         = str.find(CRLF, content_type_pos);
    if ((content_type_pos == std::string::npos) or
        (crlf_pos == std::string::npos))
      break;
    p.content_type =
        str.substr(content_type_pos + 14, crlf_pos - (content_type_pos + 14));
    Logger::lmf_app().debug("Content Type: %s", p.content_type.c_str());

    crlf_pos = str.find(CRLF + CRLF, content_type_pos);  // beginning of content
    boundary_pos = str.find(boundary_full, crlf_pos);
    if (boundary_pos == std::string::npos) {
      boundary_pos = str.find(last_boundary, crlf_pos);
    }
    if (boundary_pos > 0) {
      p.body = str.substr(crlf_pos + 4, boundary_pos - 2 - (crlf_pos + 4));
      Logger::lmf_app().debug("Body: %s", p.body.c_str());
      mime_parts.push_back(p);
    }
  }
  return true;
}

void mime_parser::get_mime_parts(std::vector<mime_part>& parts) const {
  for (auto it : mime_parts) {
    parts.push_back(it);
  }
}

//---------------------------------------------------------------------------------------------
unsigned char* mime_parser::format_string_as_hex(const std::string& str) {
  unsigned int str_len = str.length();
  char* data           = (char*) malloc(str_len + 1);
  memset(data, 0, str_len + 1);
  memcpy((void*) data, (void*) str.c_str(), str_len);

  unsigned char* data_hex = (uint8_t*) malloc(str_len / 2 + 1);
  oai::utils::conv::ascii_to_hex(data_hex, (const char*) data);

  Logger::lmf_app().debug("Input string (%d bytes): %s ", str_len, str.c_str());
  Logger::lmf_app().debug("Data (formatted):");
#if DEBUG_IS_ON
  for (int i = 0; i < str_len / 2; i++) printf(" %02x ", data_hex[i]);
  printf("\n");
#endif
  // free memory
  // free_wrapper((void**) &data);
  free(data);
  data = NULL;

  return data_hex;
}

//------------------------------------------------------------------------------
void mime_parser::create_multipart_related_content(
    std::string& body, const std::string& json_part, const std::string boundary,
    const std::string& n1_message, const std::string& n2_message,
    std::string json_format) {
  // TODO: provide Content-Ids as function parameters

  // format string as hex
  unsigned char* n1_msg_hex = format_string_as_hex(n1_message);
  unsigned char* n2_msg_hex = format_string_as_hex(n2_message);

  std::string CRLF = "\r\n";
  body.append("--" + boundary + CRLF);
  body.append("Content-Type: " + json_format + CRLF);
  body.append(CRLF);
  body.append(json_part + CRLF);

  body.append("--" + boundary + CRLF);
  body.append(
      "Content-Type: application/vnd.3gpp.5gnas" + CRLF +
      "Content-Id: n1SmMsg" + CRLF);
  body.append(CRLF);
  body.append(std::string((char*) n1_msg_hex, n1_message.length() / 2) + CRLF);
  body.append("--" + boundary + CRLF);
  body.append(
      "Content-Type: application/vnd.3gpp.ngap" + CRLF +
      "Content-Id: " + N2_NRPPa_CONTENT_ID + CRLF);
  body.append(CRLF);
  body.append(std::string((char*) n2_msg_hex, n2_message.length() / 2) + CRLF);
  body.append("--" + boundary + "--" + CRLF);

  // free memory
  oai::utils::utils::free_wrapper((void**) &n1_msg_hex);
  oai::utils::utils::free_wrapper((void**) &n2_msg_hex);
}

//------------------------------------------------------------------------------
void mime_parser::create_multipart_related_content(
    std::string& body, const std::string& json_part, const std::string boundary,
    const std::string& message,
    const multipart_related_content_part_e content_type,
    std::string json_format) {
  // TODO: provide Content-Id as function parameters
  // format string as hex
  unsigned char* msg_hex = format_string_as_hex(message);

  std::string CRLF = "\r\n";
  body.append("--" + boundary + CRLF);
  body.append("Content-Type: " + json_format + CRLF);
  body.append(CRLF);
  body.append(json_part + CRLF);

  body.append("--" + boundary + CRLF);
  if (content_type == multipart_related_content_part_e::NAS) {  // NAS
    // The only N1 part this LMF sends is LPP (TS 29.518 6.1.6.4.2).
    body.append(
        "Content-Type: application/vnd.3gpp.5gnas" + CRLF +
        "Content-Id: " + N1_LPP_CONTENT_ID + CRLF);
  } else if (content_type == multipart_related_content_part_e::NGAP) {  // NGAP
    body.append(
        "Content-Type: application/vnd.3gpp.ngap" + CRLF +
        "Content-Id: " + N2_NRPPa_CONTENT_ID + CRLF);
  }
  body.append(CRLF);
  body.append(std::string((char*) msg_hex, message.length() / 2) + CRLF);
  body.append("--" + boundary + "--" + CRLF);

  // free memory
  oai::utils::utils::free_wrapper((void**) &msg_hex);
}
