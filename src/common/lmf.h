/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#ifndef FILE_LMF_SEEN
#define FILE_LMF_SEEN

#include <string>
#include <vector>

#define HEART_BEAT_TIMER 10
#define NRF_REGISTRATION_RETRY_TIMER 5

#define N2_NRPPa_CONTENT_ID "n2NrppaMsg"
#define N1_LPP_CONTENT_ID "n1LppMsg"

#define _unused(x) ((void) (x))

#define NNRF_NFM_BASE "/nnrf-nfm/"
#define LMF_NF_REGISTER_URL "/nf-instances/"
constexpr auto CURL_MIME_BOUNDARY = "----Boundary";

typedef enum nf_type_s {
  NF_TYPE_NRF     = 0,
  NF_TYPE_AMF     = 1,
  NF_TYPE_SMF     = 2,
  NF_TYPE_AUSF    = 3,
  NF_TYPE_NEF     = 4,
  NF_TYPE_PCF     = 5,
  NF_TYPE_SMSF    = 6,
  NF_TYPE_NSSF    = 7,
  NF_TYPE_UDR     = 8,
  NF_TYPE_LMF     = 9,
  NF_TYPE_GMLC    = 10,
  NF_TYPE_5G_EIR  = 11,
  NF_TYPE_SEPP    = 12,
  NF_TYPE_UPF     = 13,
  NF_TYPE_N3IWF   = 14,
  NF_TYPE_AF      = 15,
  NF_TYPE_UDSF    = 16,
  NF_TYPE_BSF     = 17,
  NF_TYPE_CHF     = 18,
  NF_TYPE_NWDAF   = 19,
  NF_TYPE_UNKNOWN = 20
} nf_type_t;

static const std::vector<std::string> nf_type_e2str = {
    "NRF",   "AMF", "SMF",  "AUSF", "NEF",    "PCF",   "SMSF",
    "NSSF",  "UDR", "LMF",  "GMLC", "5G_EIR", "SEPP",  "UPF",
    "N3IWF", "AF",  "UDSF", "BSF",  "CHF",    "NWDAF", "UNKNOWN"};

typedef enum patch_op_type_s {
  PATCH_OP_ADD     = 0,
  PATCH_OP_REMOVE  = 1,
  PATCH_OP_REPLACE = 2,
  PATCH_OP_MOVE    = 3,
  PATCH_OP_COPY    = 4,
  PATCH_OP_TEST    = 5,
  PATCH_OP_UNKNOWN = 6

} patch_op_type_t;

static const std::vector<std::string> patch_op_type_e2str = {
    "ADD", "REMOVE", "REPLACE", "MOVE", "COPY", "TEST", "UNKNOWN"};

#define CURL_TIMEOUT_MS 1000L
#define MAX_WAIT_MSECS 20000  // 1 second

typedef uint64_t supi64_t;

typedef enum ExternalClientType_s {
  EMERGENCY_SERVICES = 1,
  VALUE_ADDED_SERVICES,
  PLMN_OPERATOR_SERVICES,
  LAWFUL_INTERCEPT_SERVICES,
  PLMN_OPERATOR_BROADCAST_SERVICES,
  PLMN_OPERATOR_OM,
  PLMN_OPERATOR_ANONYMOUS_STATISTICS,
  PLMN_OPERATOR_TARGET_MS_SERVICE_SUPPORT
} ExternalClientType_t;

static const std::vector<std::string> externalClientType_e2str = {
    "UNKNOWN_TYPE",
    "EMERGENCY_SERVICES",
    "VALUE_ADDED_SERVICES",
    "PLMN_OPERATOR_SERVICES",
    "LAWFUL_INTERCEPT_SERVICES",
    "PLMN_OPERATOR_BROADCAST_SERVICES",
    "PLMN_OPERATOR_OM",
    "PLMN_OPERATOR_ANONYMOUS_STATISTICS",
    "PLMN_OPERATOR_TARGET_MS_SERVICE_SUPPORT"};

typedef enum AccessType_s { _3GPP_ACCESS = 1, NON_3GPP_ACCESS } AccessType_t;

static const std::vector<std::string> accessType_e2str = {
    "UNKNOWN_TYPE", "3GPP_ACCESS", "NON_3GPP_ACCESS"};

typedef enum AnNodeType_s { GNB = 1, NG_ENB } AnNodeType_t;

static const std::vector<std::string> anNodeType_e2str = {
    "UNKNOWN_TYPE", "GNB", "NG_ENB"};

typedef enum RatType_s {
  NR = 1,
  EUTRA,
  WLAN,
  VIRTUAL,
  NBIOT,
  WIRELINE,
  WIRELINE_CABLE,
  WIRELINE_BBF,
  LTE_M,
  NR_U,
  EUTRA_U,
  TRUSTED_N3GA,
  TRUSTED_WLAN,
  UTRA,
  GERA
} RatType_t;

static const std::vector<std::string> ratType_e2str = {
    "UNKNOWN_TYPE", "NR",           "EUTRA",    "WLAN",
    "VIRTUAL",      "NBIOT",        "WIRELINE", "WIRELINE_CABLE",
    "WIRELINE_BBF", "LTE_M",        "NR_U",     "EUTRA_U",
    "TRUSTED_N3GA", "TRUSTED_WLAN", "UTRA",     "GERA"};

typedef struct lmf_info_s {
  std::vector<std::string> servingClientTypes;
  std::string lmfId;
  std::vector<std::string> servingAccessTypes;
  std::vector<std::string> servingAnNodeTypes;
  std::vector<std::string> servingRatTypes;
} lmf_info_t;

#endif
