# SPDX-License-Identifier: LicenseRef-CSSL-1.0

cmake_minimum_required(VERSION 3.10)

include_directories(${SRC_TOP_DIR}/api_server/api)
include_directories(${SRC_TOP_DIR}/api_server/impl)
include_directories(${SRC_TOP_DIR}/api_server/)
include_directories(${SRC_TOP_DIR}/common/)
include_directories(${SRC_TOP_DIR}/lpp)
include_directories(${SRC_TOP_DIR}/nrppa)
include_directories(${SRC_TOP_DIR}/${MOUNTED_COMMON}/common)
include_directories(${SRC_TOP_DIR}/${MOUNTED_COMMON}/config)
include_directories(${SRC_TOP_DIR}/${MOUNTED_COMMON}/http)
include_directories(${SRC_TOP_DIR}/${MOUNTED_COMMON}/logger)
include_directories(${SRC_TOP_DIR}/${MOUNTED_COMMON}/model)
include_directories(${SRC_TOP_DIR}/${MOUNTED_COMMON}/utils)
include_directories(${SRC_TOP_DIR}/${MOUNTED_COMMON}/utils/bstr)

include(${SRC_TOP_DIR}/${MOUNTED_COMMON}/http/http.cmake)
include(${SRC_TOP_DIR}/${MOUNTED_COMMON}/logger/logger.cmake)
include(${SRC_TOP_DIR}/${MOUNTED_COMMON}/model/lmf.cmake)
include(${SRC_TOP_DIR}/${MOUNTED_COMMON}/utils/utils.cmake)


# Find the LAPACK and BLAS packages
pkg_check_modules(blas REQUIRED blas)
pkg_check_modules(lapacke REQUIRED lapacke)
# RHEL needs also cblas, but Ubuntu does not have it. So `cblas_LIBRARIES` and
# `cblas_INCLUDE_DIRS` will be empty for Ubuntu, a no-op
pkg_check_modules(cblas cblas)


file(GLOB LMF_src_files
  lmf_app.cpp
  lmf_lpp.cpp
  lmf_ntn.cpp
  lmf_config.cpp
  lmf_config_yaml.cpp 
  task_manager.cpp
  lmf_event.cpp
  lmf_profile.cpp
  lmf_nrf.cpp 
  lmf_location_determination.cpp
  lmf_n1_n2_message_subscription.cpp
  lmf_non_ue_n2_message_subscription.cpp
  position_estimation.cpp
)


# Add library
add_library(LMF STATIC ${LMF_src_files})
target_include_directories(LMF PRIVATE ${blas_INCLUDE_DIRS} ${cblas_INCLUDE_DIRS} ${lapacke_INCLUDE_DIRS})

# Link libraries
target_link_libraries(LMF PRIVATE NRPPA ${blas_LIBRARIES} ${cblas_LIBRARIES} ${lapacke_LIBRARIES})
