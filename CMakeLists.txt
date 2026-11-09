cmake_minimum_required(VERSION 3.20)
project(liblighthouse VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
if(MSVC)
  # The static vcpkg triplet links the static CRT; match it.
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
  add_compile_options(/W3 /utf-8 /bigobj /permissive- /Zc:__cplusplus)
  # Eigen is ~3x faster with AVX2; dependencies must match (triplets/x64-windows-static-avx2).
  if(VCPKG_TARGET_TRIPLET MATCHES "avx2")
    add_compile_options(/arch:AVX2)
  endif()
  add_compile_definitions(_USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
else()
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()
if(WIN32)
  add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN)
endif()
if(MINGW)
  # Ship self-contained compiler runtimes rather than requiring MinGW DLLs in SteamVR.
  add_link_options(-static -static-libgcc -static-libstdc++)
  # Release packages: no debug info, and let the linker drop unused functions and
  # data (the driver exports one symbol, so most of Ceres is unreachable).
  add_compile_options($<$<CONFIG:Release,MinSizeRel>:-ffunction-sections> $<$<CONFIG:Release,MinSizeRel>:-fdata-sections>)
  add_link_options($<$<CONFIG:Release,MinSizeRel>:-Wl,--gc-sections> $<$<CONFIG:Release,MinSizeRel>:-s>)
endif()

option(LH_BUILD_USB "Build the optional hidapi transport" ON)
option(LH_BUILD_TOOLS "Build lhtool (requires USB transport)" ON)
option(LH_BUILD_VIEWER "Build GLFW/OpenGL diagnostic viewer if available" ON)
option(LH_BUILD_TESTS "Build tests" ON)
option(LH_BUILD_ORACLE_TESTS "Build tests that dlopen the original driver_lighthouse.so" ON)

include(GNUInstallDirs)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)

# ---- tracking: no hardware dependencies ---------------------------------------
# ---- lh_common: what every layer shares (log levels) ---------------------------
add_library(lh_common STATIC src/runtime/log_level.cpp src/common/net.cpp)
set_target_properties(lh_common PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(lh_common PUBLIC cxx_std_20)
target_include_directories(lh_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
if(WIN32)
  target_link_libraries(lh_common PUBLIC ws2_32)
endif()

# ---- lh_report: the bug report zip (driver and openhouse-viewer) ----------------
add_library(lh_report STATIC src/report/bug_report.cpp)
set_target_properties(lh_report PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(lh_report PRIVATE src)
target_link_libraries(lh_report PUBLIC lh_common PRIVATE ZLIB::ZLIB)
if(WIN32)
  target_link_libraries(lh_report PRIVATE shell32 ole32)
endif()
install(TARGETS lh_common EXPORT lighthouseTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

add_library(lh_tracking
  src/tracking/lighthouse_model.cpp
  src/tracking/ootx.cpp
  src/tracking/disambiguator_tdm.cpp
  src/tracking/disambiguator_multi.cpp
  src/tracking/disambiguator_sob.cpp
  src/tracking/direct_filter.cpp
  src/tracking/tracked_object.cpp
  src/tracking/pose_solver.cpp
  src/tracking/tracking_source.cpp
  src/tracking/sample_history.cpp
  src/tracking/lighthouse_direct.cpp
  src/tracking/tracker.cpp
)
add_library(lh::tracking ALIAS lh_tracking)
set_target_properties(lh_tracking PROPERTIES EXPORT_NAME tracking POSITION_INDEPENDENT_CODE ON)
target_compile_features(lh_tracking PUBLIC cxx_std_20)
target_include_directories(lh_tracking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE src)
target_link_libraries(lh_tracking PUBLIC lh_common Eigen3::Eigen Ceres::ceres nlohmann_json::nlohmann_json ZLIB::ZLIB Threads::Threads)
install(TARGETS lh_tracking EXPORT lighthouseTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# ---- replaceable USB transport ------------------------------------------------
if(LH_BUILD_USB)
  if(WIN32)
    find_package(hidapi CONFIG REQUIRED)
    set(LH_HIDAPI_TARGET hidapi::hidapi)
  elseif(APPLE)
    find_package(hidapi CONFIG REQUIRED)
    set(LH_HIDAPI_TARGET hidapi::hidapi)
    find_path(HIDAPI_PARENT_DIR NAMES "hidapi/hidapi.h")
  else()
    # hidapi's hidraw backend, built in with hidden symbols (third_party/hidapi/README.md):
    # a host's own hidapi (monado-service: libusb backend) must not replace it.
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(UDEV REQUIRED IMPORTED_TARGET libudev)
    enable_language(C)
    add_library(lh_hidapi STATIC third_party/hidapi/linux/hid.c)
    set_target_properties(lh_hidapi PROPERTIES POSITION_INDEPENDENT_CODE ON C_VISIBILITY_PRESET hidden)
    target_include_directories(lh_hidapi PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/hidapi>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/hidapi/hidapi>)
    target_link_libraries(lh_hidapi PUBLIC PkgConfig::UDEV)
    install(TARGETS lh_hidapi EXPORT lighthouseTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
    set(LH_HIDAPI_TARGET lh_hidapi)
  endif()
  add_library(lh_usb
    src/transport/usb/report_parser.cpp
    src/transport/usb/device_config.cpp
    src/transport/usb/raw_capture.cpp
    src/transport/usb/wireless_decoder.cpp
    src/transport/usb/usb_transport.cpp)
  add_library(lh::usb ALIAS lh_usb)
  set_target_properties(lh_usb PROPERTIES EXPORT_NAME usb POSITION_INDEPENDENT_CODE ON)
  target_compile_features(lh_usb PUBLIC cxx_std_20)
  target_include_directories(lh_usb PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/transport/usb>
    $<$<BOOL:${HIDAPI_PARENT_DIR}>:$<BUILD_INTERFACE:${HIDAPI_PARENT_DIR}>>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_include_directories(lh_usb PRIVATE src)
  target_link_libraries(lh_usb PUBLIC lh_common ${LH_HIDAPI_TARGET} Threads::Threads ZLIB::ZLIB)
  if(WIN32)
    target_link_libraries(lh_usb PUBLIC advapi32)
  endif()
  install(TARGETS lh_usb EXPORT lighthouseTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  install(DIRECTORY src/transport/usb/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/lh/usb FILES_MATCHING PATTERN "*.hpp")

  if(LH_BUILD_TOOLS)
    add_executable(lhtool apps/lhtool/main.cpp)
    target_link_libraries(lhtool PRIVATE lh_usb lh_tracking lh)
    # Started by SteamVR each session: warns when an update replaced our driver.
    add_executable(openhouse-check WIN32 apps/openhouse-check/main.cpp)
    add_executable(lhcapture apps/lhcapture/main.cpp)
    target_include_directories(lhcapture PRIVATE src)
    target_link_libraries(lhcapture PRIVATE lh_usb nlohmann_json::nlohmann_json)
    install(TARGETS lhtool lhcapture RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  endif()
endif()

# ---- liblh: the session runtime (shared room map, devices, input) -------------
add_library(lh_runtime OBJECT
  src/runtime/room_map.cpp
  src/runtime/index_input.cpp
  src/runtime/device_tracker.cpp
  src/runtime/room_solver.cpp
  src/runtime/session.cpp)
set_target_properties(lh_runtime PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(lh_runtime PUBLIC cxx_std_20)
target_include_directories(lh_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE src)
target_link_libraries(lh_runtime PRIVATE lh_tracking Ceres::ceres Eigen3::Eigen nlohmann_json::nlohmann_json)
if(LH_BUILD_USB)
  target_link_libraries(lh_runtime PRIVATE lh_usb)
  target_compile_definitions(lh_runtime PRIVATE LH_RUNTIME_USB=1)
endif()

add_library(lh SHARED $<TARGET_OBJECTS:lh_runtime>)
add_library(lh::lh ALIAS lh)
if(WIN32)
  target_link_libraries(lh PRIVATE advapi32)
endif()
if(MINGW)
  # Don't auto-export embedded libgcc/libstdc++/dependency symbols into consumers.
  target_link_options(lh PRIVATE -Wl,--exclude-libs,ALL)
endif()
set_target_properties(lh PROPERTIES EXPORT_NAME lh VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR}
  WINDOWS_EXPORT_ALL_SYMBOLS ON)
target_compile_features(lh PUBLIC cxx_std_20)
target_include_directories(lh PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(lh PRIVATE lh_tracking Ceres::ceres Eigen3::Eigen nlohmann_json::nlohmann_json)
if(LH_BUILD_USB)
  target_link_libraries(lh PRIVATE lh_usb)
endif()

# ---- drop-in OpenVR driver (SteamVR / Monado steamvr_lh): driver_lighthouse.so ----
option(LH_BUILD_OPENVR_DRIVER "Build the drop-in driver_lighthouse.so" ON)
find_path(OPENVR_INCLUDE_DIR openvr_driver.h PATH_SUFFIXES openvr)
if(LH_BUILD_OPENVR_DRIVER AND LH_BUILD_USB AND OPENVR_INCLUDE_DIR)
  add_library(driver_lighthouse MODULE src/openvr/driver_lighthouse.cpp src/openvr/hand_skeleton.cpp src/openvr/raw_alignment.cpp src/openvr/room_publisher.cpp src/openvr/host_frame.cpp src/openvr/stock_hmd.cpp src/openvr/pairing_mailbox.cpp src/openvr/telemetry_server.cpp $<TARGET_OBJECTS:lh_runtime>)
  if(WIN32)
    target_link_libraries(driver_lighthouse PRIVATE setupapi hid)  # stock_hmd: hiding devices from the stock DLL
    set(LH_OPENVR_PLATFORM win64)
  elseif(APPLE)
    set(LH_OPENVR_PLATFORM osx)
  else()
    set(LH_OPENVR_PLATFORM linux64)
  endif()
  set_target_properties(driver_lighthouse PROPERTIES PREFIX "" CXX_VISIBILITY_PRESET hidden
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/openvr/lighthouse/bin/${LH_OPENVR_PLATFORM}
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/openvr/lighthouse/bin/${LH_OPENVR_PLATFORM})
  target_include_directories(driver_lighthouse PRIVATE include src ${OPENVR_INCLUDE_DIR})
  target_link_libraries(driver_lighthouse PRIVATE lh_report lh_tracking lh_usb Ceres::ceres Eigen3::Eigen nlohmann_json::nlohmann_json)
  find_package(Python3 COMPONENTS Interpreter QUIET)
  find_program(PATCHELF_EXECUTABLE patchelf)
  if(NOT WIN32 AND NOT APPLE AND Python3_Interpreter_FOUND AND PATCHELF_EXECUTABLE)
    add_custom_target(driver_package
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/package-driver.py
        $<TARGET_FILE:driver_lighthouse> ${CMAKE_BINARY_DIR}/driver-package
      DEPENDS driver_lighthouse
      COMMENT "Bundling driver dependencies for Steam Runtime")
  endif()
  # Only HmdDriverFactory is exported, so nothing clashes with the host process.
  if(NOT WIN32 AND NOT APPLE)
    target_link_options(driver_lighthouse PRIVATE
      -Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/src/openvr/exports.map -Wl,--no-undefined)
  endif()
else()
  message(STATUS "OpenVR driver headers not found: skipping driver_lighthouse.so")
endif()

install(TARGETS lh EXPORT lighthouseTargets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

# ---- lhviz: 3D visualizer on top of liblh ---------------------------------------
if(LH_BUILD_VIEWER)
  find_package(glfw3 QUIET)
  find_package(OpenGL QUIET)
  if(TARGET glfw AND OpenGL_FOUND)
    add_library(lh_imgui STATIC
      third_party/imgui/imgui.cpp third_party/imgui/imgui_draw.cpp third_party/imgui/imgui_tables.cpp
      third_party/imgui/imgui_widgets.cpp third_party/imgui/backends/imgui_impl_glfw.cpp
      third_party/imgui/backends/imgui_impl_opengl2.cpp)
    target_include_directories(lh_imgui PUBLIC third_party/imgui)
    target_link_libraries(lh_imgui PUBLIC glfw OpenGL::GL)
    target_compile_options(lh_imgui PRIVATE -w)
    add_executable(lhviz apps/lhviz/main.cpp)
    target_link_libraries(lhviz PRIVATE lh lh_imgui Eigen3::Eigen)
    install(TARGETS lhviz RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    # openhouse-viewer: the running driver's status and log (telemetry over loopback)
    add_executable(openhouse-viewer apps/openhouse-viewer/main.cpp apps/openhouse-viewer/telemetry_client.cpp
                   apps/openhouse-viewer/log_view.cpp apps/openhouse-viewer/views.cpp)
    target_include_directories(openhouse-viewer PRIVATE src)
    target_link_libraries(openhouse-viewer PRIVATE lh_common lh_report lh_imgui nlohmann_json::nlohmann_json)
    install(TARGETS openhouse-viewer RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    # openhouse-compare: openhouse and SteamVR's own driver tracking side by side
    if(OPENVR_INCLUDE_DIR AND LH_BUILD_USB)
      add_executable(openhouse-compare apps/openhouse-compare/main.cpp apps/openhouse-compare/stock_host.cpp)
      target_include_directories(openhouse-compare PRIVATE src ${OPENVR_INCLUDE_DIR})
      target_link_libraries(openhouse-compare PRIVATE lh lh_imgui Eigen3::Eigen nlohmann_json::nlohmann_json ${CMAKE_DL_LIBS})
      install(TARGETS openhouse-compare RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()
  else()
    message(STATUS "GLFW/OpenGL not found: skipping lhviz")
  endif()
endif()

# ---- tests --------------------------------------------------------------------
if(LH_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(lh_unit_tests
    tests/unit/test_platform.cpp
    tests/unit/test_tracking_source.cpp
    tests/unit/test_pose_solver.cpp
    tests/unit/test_tracker.cpp)
  target_include_directories(lh_unit_tests PRIVATE src)
  target_link_libraries(lh_unit_tests PRIVATE lh_tracking GTest::gtest_main)
  if(WIN32)
    target_link_libraries(lh_unit_tests PRIVATE advapi32)
  endif()
  if(LH_BUILD_USB)
    target_sources(lh_unit_tests PRIVATE tests/unit/test_report_parser.cpp tests/unit/test_device_config.cpp
      tests/unit/test_raw_capture.cpp)
    target_link_libraries(lh_unit_tests PRIVATE lh_usb)
    add_executable(lh_replay_tests tests/integration/test_replay.cpp)
    target_compile_definitions(lh_replay_tests PRIVATE LH_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    target_include_directories(lh_replay_tests PRIVATE tests)
    target_link_libraries(lh_replay_tests PRIVATE lh_usb lh_tracking GTest::gtest_main)
    add_test(NAME replay COMMAND lh_replay_tests)
  endif()
  add_test(NAME unit COMMAND lh_unit_tests)
  if(TARGET driver_lighthouse)
    add_executable(lh_openvr_component_tests tests/openvr/test_components.cpp src/openvr/hand_skeleton.cpp src/openvr/raw_alignment.cpp src/openvr/room_publisher.cpp src/openvr/host_frame.cpp src/openvr/telemetry_server.cpp)
    target_include_directories(lh_openvr_component_tests PRIVATE src tests ${OPENVR_INCLUDE_DIR})
    target_compile_definitions(lh_openvr_component_tests PRIVATE LH_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    target_link_libraries(lh_openvr_component_tests PRIVATE lh lh_tracking Eigen3::Eigen nlohmann_json::nlohmann_json GTest::gtest_main)
    add_test(NAME openvr_components COMMAND lh_openvr_component_tests)
    find_library(OPENVR_API_LIBRARY openvr_api)
    if(OPENVR_API_LIBRARY)
      add_executable(lh_openvr_client_probe tests/openvr/client_probe.cpp)
      target_include_directories(lh_openvr_client_probe PRIVATE ${OPENVR_INCLUDE_DIR})
      target_link_libraries(lh_openvr_client_probe PRIVATE ${OPENVR_API_LIBRARY} Threads::Threads)
    endif()
    add_executable(lh_openvr_host_test tests/openvr/mock_host.cpp)
    target_include_directories(lh_openvr_host_test PRIVATE ${OPENVR_INCLUDE_DIR})
    target_link_libraries(lh_openvr_host_test PRIVATE ${CMAKE_DL_LIBS} Threads::Threads)
    add_dependencies(lh_openvr_host_test driver_lighthouse)
    add_test(NAME openvr_driver COMMAND lh_openvr_host_test $<TARGET_FILE:driver_lighthouse>
      ${CMAKE_SOURCE_DIR}/tests/data/captures/session-20261016-142931Z-147083/input.lhraw 14)
  endif()
  add_executable(lh_runtime_tests tests/runtime/test_runtime.cpp)
  target_include_directories(lh_runtime_tests PRIVATE src tests)
  target_compile_definitions(lh_runtime_tests PRIVATE LH_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
  if(LH_BUILD_USB)
    target_compile_definitions(lh_runtime_tests PRIVATE LH_TEST_USB=1)
    target_link_libraries(lh_runtime_tests PRIVATE lh_usb)
  endif()
  target_link_libraries(lh_runtime_tests PRIVATE lh lh_tracking Eigen3::Eigen nlohmann_json::nlohmann_json GTest::gtest_main)
  add_test(NAME runtime COMMAND lh_runtime_tests)
  if(TARGET lhcapture)
    add_test(NAME capture_help COMMAND lhcapture --help)
    add_test(NAME capture_bad_duration COMMAND lhcapture --seconds not-a-number)
    set_tests_properties(capture_bad_duration PROPERTIES WILL_FAIL TRUE)
    add_test(NAME capture_no_overwrite COMMAND lhcapture "${CMAKE_CURRENT_BINARY_DIR}")
    set_tests_properties(capture_no_overwrite PROPERTIES WILL_FAIL TRUE)
  endif()

  if(LH_BUILD_ORACLE_TESTS AND NOT WIN32 AND NOT APPLE)
    add_executable(lh_oracle_tests
      tests/oracle/oracle.cpp
      tests/oracle/test_lighthouse_model.cpp
      tests/oracle/test_residuals.cpp
      tests/oracle/test_direct_filter.cpp
      tests/oracle/test_pose_solver.cpp
      tests/oracle/test_sample_history.cpp
      tests/oracle/test_realres.cpp)
    target_include_directories(lh_oracle_tests PRIVATE src tests/oracle)
    target_compile_definitions(lh_oracle_tests PRIVATE
      LH_ORIGINAL_SO="${CMAKE_SOURCE_DIR}/driver_lighthouse.so" LH_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    target_link_libraries(lh_oracle_tests PRIVATE lh_tracking GTest::gtest_main ${CMAKE_DL_LIBS})
    if(LH_BUILD_USB)
      target_sources(lh_oracle_tests PRIVATE tests/oracle/test_tdm_diff.cpp tests/oracle/test_wireless_diff.cpp
        tests/oracle/test_motion_filter.cpp)
      target_link_libraries(lh_oracle_tests PRIVATE lh_usb)
      add_executable(tdm_probe tests/oracle/tdm_probe.cpp tests/oracle/oracle.cpp)
      target_include_directories(tdm_probe PRIVATE src tests/oracle)
      target_compile_definitions(tdm_probe PRIVATE LH_ORIGINAL_SO="${CMAKE_SOURCE_DIR}/driver_lighthouse.so")
      target_link_libraries(tdm_probe PRIVATE lh_usb ${CMAKE_DL_LIBS})
    endif()
    add_test(NAME oracle COMMAND lh_oracle_tests)
  endif()
endif()

# ---- standalone CMake package --------------------------------------------------
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/lighthouseConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/lighthouseConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lighthouse)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/lighthouseConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT lighthouseTargets NAMESPACE lh:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lighthouse)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/lighthouseConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/lighthouseConfigVersion.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lighthouse)
