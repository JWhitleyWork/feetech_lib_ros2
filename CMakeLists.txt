cmake_minimum_required(VERSION 3.5)
project(feetech_lib)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(FEETECH_LIB_HEADERS
  include/feetech_lib/INST.hpp
  include/feetech_lib/SCS.hpp
  include/feetech_lib/SCSCL.hpp
  include/feetech_lib/SCSerial.hpp
  include/feetech_lib/SCServo.hpp
  include/feetech_lib/SMS_STS.hpp
  include/feetech_lib/SMSBL.hpp
  include/feetech_lib/SMSCL.hpp
)

set(FEETECH_LIB_SOURCES
  src/SCS.cpp
  src/SCSCL.cpp
  src/SCSerial.cpp
  src/SMS_STS.cpp
  src/SMSBL.cpp
  src/SMSCL.cpp
)

ament_auto_add_library(feetech_lib SHARED
  ${FEETECH_LIB_HEADERS}
  ${FEETECH_LIB_SOURCES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)