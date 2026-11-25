cmake_minimum_required(VERSION 3.20)
project(liblighthouse VERSION 1.0.4 LANGUAGES CXX)

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
  # Release binaries name no path on the builder's machine (__FILE__ in messages).
  add_compile_options(-ffile-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}/=)
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

set(LIGHTHOUSE_INCLUDE_DIR "$ENV{LIGHTHOUSE_INCLUDE_DIR}" CACHE PATH
    "Optional ShayBox Lighthouse include directory for checking the BLE ABI")
if(LIGHTHOUSE_INCLUDE_DIR)
  set(lighthouse_header "${LIGHTHOUSE_INCLUDE_DIR}/lighthouse.h")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${lighthouse_header}")
  file(READ "${lighthouse_header}" lighthouse_abi)
  # 2.1.0 declares two unscoped Unknown enumerators. Rename the error enumerator
  # in the build copy without changing any of the types or values.
  string(REPLACE "Unknown = -99," "LighthouseErrorUnknown = -99," lighthouse_abi "${lighthouse_abi}")
  file(CONFIGURE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/lh/lighthouse_abi.h"
       CONTENT "${lighthouse_abi}" @ONLY)
endif()

include(GNUInstallDirs)

# The build's version stamp (cmake/build_id.cmake), redone on every build: lh/build_id.h for
# LH_VERSION_STRING, VERSION.txt for the packages. Every target waits for it (end of file).
add_custom_target(lh_build_id ALL
  COMMAND ${CMAKE_COMMAND} -DSRC=${CMAKE_CURRENT_SOURCE_DIR} -DOUT=${CMAKE_CURRENT_BINARY_DIR} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/build_id.cmake
  BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/generated/lh/build_id.h ${CMAKE_CURRENT_BINARY_DIR}/generated/lh/build_time.h ${CMAKE_CURRENT_BINARY_DIR}/VERSION.txt
  COMMENT "Version stamp")
include_directories(BEFORE ${CMAKE_CURRENT_BINARY_DIR}/generated)
if(WIN32)
  enable_language(RC)
endif()
include(cmake/windows/lh_windows_resources.cmake)
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

# ---- lh_report: the bug report zip (driver and openhouse-overlay) ----------------
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
  src/tracking/pulse_sorter.cpp
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
    src/transport/usb/usb_transport.cpp
    src/transport/usb/doorman_dongle.cpp)
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
    # liblh's code is built in (as in the driver), so the tool runs on its own.
    add_executable(lhtool apps/lhtool/main.cpp $<TARGET_OBJECTS:lh_runtime>)
    target_include_directories(lhtool PRIVATE include src)
    target_link_libraries(lhtool PRIVATE lh_usb lh_tracking Ceres::ceres Eigen3::Eigen nlohmann_json::nlohmann_json)
    if(WIN32)
      target_link_libraries(lhtool PRIVATE advapi32)
    endif()
    add_executable(lhcapture apps/lhcapture/main.cpp)
    target_include_directories(lhcapture PRIVATE src)
    target_link_libraries(lhcapture PRIVATE lh_usb nlohmann_json::nlohmann_json)
    install(TARGETS lhtool lhcapture RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  endif()
endif()

# ---- liblh: the session runtime (shared room map, devices, input) -------------
add_library(lh_runtime OBJECT
  src/runtime/mixed_light.cpp
  src/runtime/room_map.cpp
  src/runtime/index_input.cpp
  src/runtime/mapped_input.cpp
  src/runtime/model_scale.cpp
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
  add_library(driver_lighthouse MODULE src/openvr/driver_lighthouse.cpp src/openvr/hand_skeleton.cpp src/openvr/raw_alignment.cpp src/openvr/steam_rooms.cpp src/openvr/room_publisher.cpp src/openvr/host_frame.cpp src/openvr/stock_hmd.cpp src/openvr/pairing_mailbox.cpp src/openvr/telemetry_server.cpp src/openvr/watchdog.cpp $<TARGET_OBJECTS:lh_runtime>)
  if(WIN32)
    target_link_libraries(driver_lighthouse PRIVATE setupapi hid)  # stock_hmd: hiding devices from the stock DLL
    # HmdDriverFactory and our hidapi, for drivers that hook HID access (the .def says why).
    target_sources(driver_lighthouse PRIVATE src/openvr/driver_lighthouse.def)
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
option(LH_STATIC_GLFW "Build GLFW from source and link it statically (release packages)" OFF)
if(LH_BUILD_VIEWER AND LH_STATIC_GLFW)
  # GLFW loads X11/Wayland/libGL itself at run time: only their headers are needed here,
  # and users don't need libglfw installed.
  include(FetchContent)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS_SAVED ${BUILD_SHARED_LIBS})
  set(BUILD_SHARED_LIBS OFF)
  FetchContent_Declare(glfw URL https://github.com/glfw/glfw/releases/download/3.4/glfw-3.4.zip
                       URL_HASH SHA256=b5ec004b2712fd08e8861dc271428f048775200a2df719ccf575143ba749a3e9)
  FetchContent_MakeAvailable(glfw)
  set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_SAVED})
endif()
if(LH_BUILD_VIEWER)
  if(NOT TARGET glfw)
    find_package(glfw3 QUIET)
  endif()
  # Some distributions ship GLFW without its CMake package (only glfw3.pc): find it there.
  if(NOT TARGET glfw)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(LH_GLFW QUIET IMPORTED_TARGET glfw3)
      if(LH_GLFW_FOUND)
        add_library(glfw INTERFACE IMPORTED)
        target_link_libraries(glfw INTERFACE PkgConfig::LH_GLFW)
      endif()
    endif()
  endif()
  find_package(OpenGL QUIET)
  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/imgui.cpp)
    set(LH_VIEWER_MISSING "Dear ImGui (third_party/imgui is missing from the source tree)")
  elseif(NOT TARGET glfw)
    set(LH_VIEWER_MISSING "GLFW 3 (install its development package, e.g. glfw-devel / libglfw3-dev / glfw)")
  elseif(NOT OpenGL_FOUND)
    set(LH_VIEWER_MISSING "OpenGL (install its development package, e.g. mesa-libGL-devel / libgl-dev)")
  endif()
  if(NOT LH_VIEWER_MISSING)
    add_library(lh_imgui STATIC
      third_party/imgui/imgui.cpp third_party/imgui/imgui_draw.cpp third_party/imgui/imgui_tables.cpp
      third_party/imgui/imgui_widgets.cpp third_party/imgui/backends/imgui_impl_glfw.cpp
      third_party/imgui/backends/imgui_impl_opengl2.cpp)
    target_include_directories(lh_imgui PUBLIC third_party/imgui)
    target_compile_definitions(lh_imgui PUBLIC IMGUI_USE_WCHAR32)
    target_link_libraries(lh_imgui PUBLIC glfw OpenGL::GL)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
      # Fedora's Noto CJK uses CFF2 outlines, which stb_truetype cannot load.
      find_package(Freetype REQUIRED)
      target_sources(lh_imgui PRIVATE third_party/imgui/misc/freetype/imgui_freetype.cpp)
      target_compile_definitions(lh_imgui PRIVATE IMGUI_ENABLE_FREETYPE)
      target_link_libraries(lh_imgui PRIVATE Freetype::Freetype)
    endif()
    target_compile_options(lh_imgui PRIVATE -w)
    add_executable(lhviz apps/lhviz/main.cpp $<TARGET_OBJECTS:lh_runtime>)
    target_include_directories(lhviz PRIVATE include)
    target_link_libraries(lhviz PRIVATE lh_imgui lh_tracking Ceres::ceres Eigen3::Eigen nlohmann_json::nlohmann_json)
    if(LH_BUILD_USB)
      target_link_libraries(lhviz PRIVATE lh_usb)
    endif()
    if(WIN32)
      target_link_libraries(lhviz PRIVATE advapi32)
    endif()
    install(TARGETS lhviz RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    # openhouse-viewer: the running driver's status and log (telemetry over loopback)
    add_executable(openhouse-viewer WIN32 apps/openhouse-viewer/main.cpp apps/openhouse-viewer/telemetry_client.cpp
                   apps/openhouse-viewer/log_view.cpp apps/openhouse-viewer/views.cpp)
    target_include_directories(openhouse-viewer PRIVATE src)
    target_link_libraries(openhouse-viewer PRIVATE lh_common lh_report lh_imgui nlohmann_json::nlohmann_json)
    if(MSVC)
      target_link_options(openhouse-viewer PRIVATE /ENTRY:mainCRTStartup)  # a GUI app with main()
    endif()
    install(TARGETS openhouse-viewer RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    # openhouse-overlay: the management app, a SteamVR dashboard overlay or a window. SteamVR's
    # OpenVR client library is loaded from the SteamVR install at run time (not linked/shipped).
    if(OPENVR_INCLUDE_DIR)
      add_executable(openhouse-overlay WIN32 apps/openhouse-overlay/main.cpp apps/openhouse-overlay/ui.cpp
                     apps/openhouse-overlay/base_channels.cpp
                     apps/openhouse-overlay/openvr_runtime.cpp apps/openhouse-overlay/update_check.cpp
                     apps/openhouse-overlay/driver_check.cpp
                     apps/openhouse-overlay/telemetry_client.cpp apps/openhouse-overlay/log_view.cpp
                     apps/openhouse-overlay/views.cpp)
      target_include_directories(openhouse-overlay PRIVATE src ${OPENVR_INCLUDE_DIR})
      if(LIGHTHOUSE_INCLUDE_DIR)
        target_compile_definitions(openhouse-overlay PRIVATE LH_CHECK_LIGHTHOUSE_ABI)
      endif()
      target_link_libraries(openhouse-overlay PRIVATE lh_common lh_report lh_imgui nlohmann_json::nlohmann_json ${CMAKE_DL_LIBS})
      if(WIN32)
        target_link_libraries(openhouse-overlay PRIVATE winhttp shell32)
      endif()
      if(MSVC)
        target_link_options(openhouse-overlay PRIVATE /ENTRY:mainCRTStartup)  # a GUI app with main()
      endif()
      install(TARGETS openhouse-overlay RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()
    # openhouse-installer: installs/uninstalls the driver and flashes openhouse's firmware (base
    # stations, dongles, device radios), as a guided window. Self-contained: GLFW and ImGui are
    # linked in (GLFW statically with LH_STATIC_GLFW), and on Linux the C++ runtime too.
    add_executable(openhouse-installer WIN32 apps/openhouse-installer/main.cpp apps/openhouse-installer/ui.cpp
                   apps/openhouse-installer/system.cpp apps/openhouse-installer/driver_install.cpp
                   apps/openhouse-installer/drives.cpp apps/openhouse-installer/radio.cpp
                   apps/openhouse-installer/device_type.cpp apps/openhouse-installer/self_update.cpp apps/openhouse-installer/shortcuts.cpp
                   src/common/nordic_dfu.cpp)
    target_include_directories(openhouse-installer PRIVATE include src apps/openhouse-overlay)
    target_link_libraries(openhouse-installer PRIVATE lh_imgui ${LH_HIDAPI_TARGET} nlohmann_json::nlohmann_json)
    if(WIN32)
      target_link_libraries(openhouse-installer PRIVATE shell32 ole32 uuid setupapi winhttp)
    else()
      target_link_options(openhouse-installer PRIVATE -static-libstdc++ -static-libgcc)
    endif()
    if(MSVC)
      target_link_options(openhouse-installer PRIVATE /ENTRY:mainCRTStartup)
    endif()
    install(TARGETS openhouse-installer RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    # doorman-dongle-manager: the Doorman Dongle Manager (slots, pairing, radio telemetry, firmware).
    # A desktop window; device names from the driver's telemetry when it runs.
    if(LH_BUILD_USB)
      add_executable(doorman-dongle-manager WIN32 apps/doorman-dongle-manager/main.cpp apps/doorman-dongle-manager/ui.cpp
                     apps/doorman-dongle-manager/dongle.cpp apps/openhouse-overlay/telemetry_client.cpp src/common/nordic_dfu.cpp
                     apps/openhouse-overlay/update_check.cpp)
      target_include_directories(doorman-dongle-manager PRIVATE src src/transport/usb apps/openhouse-overlay)
      target_link_libraries(doorman-dongle-manager PRIVATE lh_usb lh_common lh_imgui nlohmann_json::nlohmann_json)
      if(WIN32)
        target_link_libraries(doorman-dongle-manager PRIVATE winhttp shell32 setupapi)
      endif()
      if(MSVC)
        target_link_options(doorman-dongle-manager PRIVATE /ENTRY:mainCRTStartup)  # a GUI app with main()
      endif()
      install(TARGETS doorman-dongle-manager RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()
    # openhouse-compare: openhouse and SteamVR's own driver tracking side by side
    if(OPENVR_INCLUDE_DIR AND LH_BUILD_USB)
      add_executable(openhouse-compare apps/openhouse-compare/main.cpp apps/openhouse-compare/stock_host.cpp)
      target_include_directories(openhouse-compare PRIVATE src ${OPENVR_INCLUDE_DIR})
      target_link_libraries(openhouse-compare PRIVATE lh lh_imgui Eigen3::Eigen nlohmann_json::nlohmann_json ${CMAKE_DL_LIBS})
      install(TARGETS openhouse-compare RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()
  else()
    message(WARNING "LH_BUILD_VIEWER is on, but ${LH_VIEWER_MISSING} wasn't found: lhviz, openhouse-overlay "
                    "and openhouse-compare are not built. Install it, or configure with -DLH_BUILD_VIEWER=OFF.")
  endif()
endif()

# ---- tests --------------------------------------------------------------------
if(LH_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(lh_unit_tests
    tests/unit/test_platform.cpp
    tests/unit/test_version.cpp
    tests/unit/test_tracker_names.cpp
    tests/unit/test_tracking_source.cpp
    tests/unit/test_pose_solver.cpp
    tests/unit/test_tracker.cpp
    tests/unit/test_direct_filter.cpp
    tests/unit/test_disambiguator_sob.cpp
    tests/unit/test_pulse_sorter.cpp
    tests/unit/test_mixed_light.cpp
    tests/unit/test_rotor_wobble.cpp
    tests/unit/test_rotor_reference.cpp
    src/runtime/mixed_light.cpp)
  target_include_directories(lh_unit_tests PRIVATE src)
  target_link_libraries(lh_unit_tests PRIVATE lh_tracking nlohmann_json::nlohmann_json GTest::gtest_main)
  if(WIN32)
    target_link_libraries(lh_unit_tests PRIVATE advapi32 winhttp)
  endif()
  if(LH_BUILD_USB)
    target_sources(lh_unit_tests PRIVATE tests/unit/test_report_parser.cpp tests/unit/test_device_config.cpp
      tests/unit/test_raw_capture.cpp tests/unit/test_radio_mode.cpp)
    target_link_libraries(lh_unit_tests PRIVATE lh_usb)
    get_target_property(LH_USB_TARGET_TYPE lh_usb TYPE)
    if(LH_USB_TARGET_TYPE STREQUAL "STATIC_LIBRARY")
      # Several Doorman Dongles at once. The test defines hidapi's entry points itself (fake
      # dongles); they replace the static transport's device calls, so no real device is opened.
      add_executable(lh_doorman_dongle_tests tests/unit/test_doorman_dongles.cpp)
      target_link_libraries(lh_doorman_dongle_tests PRIVATE lh_usb GTest::gtest_main)
      add_test(NAME doorman_dongles COMMAND lh_doorman_dongle_tests)
      set_tests_properties(doorman_dongles PROPERTIES TIMEOUT 60)
    endif()
    add_executable(lh_replay_tests tests/integration/test_replay.cpp)
    target_compile_definitions(lh_replay_tests PRIVATE LH_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    target_include_directories(lh_replay_tests PRIVATE tests)
    target_link_libraries(lh_replay_tests PRIVATE lh_usb lh_tracking GTest::gtest_main)
    add_test(NAME replay COMMAND lh_replay_tests)
  endif()
  add_test(NAME unit COMMAND lh_unit_tests)
  if(TARGET driver_lighthouse)
    add_library(lh_test_ble SHARED tests/openvr/fake_lighthouse_ble.cpp)
    add_executable(lh_openvr_component_tests tests/openvr/test_components.cpp tests/openvr/test_base_channels.cpp src/openvr/hand_skeleton.cpp src/openvr/raw_alignment.cpp src/openvr/steam_rooms.cpp src/openvr/room_publisher.cpp src/openvr/host_frame.cpp src/openvr/telemetry_server.cpp src/openvr/pairing_mailbox.cpp apps/openhouse-overlay/base_channels.cpp)
    target_include_directories(lh_openvr_component_tests PRIVATE src tests apps/openhouse-overlay ${OPENVR_INCLUDE_DIR})
    target_include_directories(lh_test_ble PRIVATE apps/openhouse-overlay)
    if(LIGHTHOUSE_INCLUDE_DIR)
      target_compile_definitions(lh_openvr_component_tests PRIVATE LH_CHECK_LIGHTHOUSE_ABI)
      target_compile_definitions(lh_test_ble PRIVATE LH_CHECK_LIGHTHOUSE_ABI)
    endif()
    target_compile_definitions(lh_openvr_component_tests PRIVATE LH_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    target_link_libraries(lh_openvr_component_tests PRIVATE lh lh_tracking Eigen3::Eigen nlohmann_json::nlohmann_json GTest::gtest_main ${CMAKE_DL_LIBS})
    target_link_libraries(lh_openvr_component_tests PRIVATE lh_test_ble)
    target_compile_definitions(lh_openvr_component_tests PRIVATE LH_TEST_BLE="$<TARGET_FILE:lh_test_ble>")
    add_test(NAME openvr_components COMMAND lh_openvr_component_tests)
    find_library(OPENVR_API_LIBRARY openvr_api)
    if(OPENVR_API_LIBRARY)
      add_executable(lh_openvr_client_probe tests/openvr/client_probe.cpp)
      target_include_directories(lh_openvr_client_probe PRIVATE ${OPENVR_INCLUDE_DIR})
      target_link_libraries(lh_openvr_client_probe PRIVATE ${OPENVR_API_LIBRARY} Threads::Threads)
      add_executable(lh_openvr_skeleton_recorder tests/openvr/skeleton_recorder.cpp)
      target_include_directories(lh_openvr_skeleton_recorder PRIVATE ${OPENVR_INCLUDE_DIR})
      target_link_libraries(lh_openvr_skeleton_recorder PRIVATE ${OPENVR_API_LIBRARY} Threads::Threads)
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
  # Doorman air codec: every frame of the bundled LH2 capture, batched as the radio does at 8 devices,
  # must come back exactly (doorman/test, ai/doorman/testing.md). Built as C++: the codec headers are
  # plain C that compiles either way.
  if(TARGET lhtool AND NOT MSVC)   # the codec uses GCC/Clang builtins, as the firmware's compiler does
    add_executable(doorman_codec_roundtrip doorman/test/codec_roundtrip.c)
    set_source_files_properties(doorman/test/codec_roundtrip.c PROPERTIES LANGUAGE CXX)
    target_include_directories(doorman_codec_roundtrip PRIVATE doorman/common)
    add_test(NAME doorman_codec COMMAND ${CMAKE_COMMAND}
      -DLHTOOL=$<TARGET_FILE:lhtool> -DROUNDTRIP=$<TARGET_FILE:doorman_codec_roundtrip>
      -DCAPTURE=${CMAKE_CURRENT_SOURCE_DIR}/tests/data/captures/lh2-five-devices/input.lhraw
      -DFRAMES=${CMAKE_CURRENT_BINARY_DIR}/doorman_codec_frames.bin
      -P ${CMAKE_CURRENT_SOURCE_DIR}/doorman/test/codec_test.cmake)
  endif()
  if(TARGET lhcapture)
    add_test(NAME capture_help COMMAND lhcapture --help)
    add_test(NAME capture_bad_duration COMMAND lhcapture --seconds not-a-number)
    set_tests_properties(capture_bad_duration PROPERTIES WILL_FAIL TRUE)
    add_test(NAME capture_no_overwrite COMMAND lhcapture "${CMAKE_CURRENT_BINARY_DIR}")
    set_tests_properties(capture_no_overwrite PROPERTIES WILL_FAIL TRUE)
  endif()

  # Only with the stock driver they dlopen (git-ignored): a fresh clone aborted ctest (#55).
  if(LH_BUILD_ORACLE_TESTS AND NOT WIN32 AND NOT APPLE AND EXISTS "${CMAKE_SOURCE_DIR}/driver_lighthouse.so")
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
# Version information in the Windows binaries.
foreach(entry
    "lh|openhouse tracking library"
    "driver_lighthouse|openhouse SteamVR lighthouse driver"
    "lhtool|openhouse replay and diagnostics tool"
    "lhcapture|openhouse device recorder"
    "openhouse-overlay|openhouse overlay|openhouse.ico"
    "doorman-dongle-manager|Doorman Dongle Manager|dongle-manager.ico"
    "openhouse-compare|openhouse and SteamVR driver comparison"
    "lhviz|openhouse developer viewer"
    "openhouse-viewer|openhouse driver status viewer"
    "openhouse-installer|openhouse driver installer|openhouse.ico")
  string(REPLACE "|" ";" parts "${entry}")
  list(GET parts 0 name)
  list(GET parts 1 text)
  set(icon "")
  list(LENGTH parts n)
  if(n GREATER 2)
    list(GET parts 2 icon)
  endif()
  if(TARGET ${name})
    lh_windows_resources(${name} "${text}" "${icon}")
  endif()
endforeach()

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

# Everything built here waits for the version stamp (lh/build_id.h).
get_property(lh_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(t IN LISTS lh_targets)
  get_target_property(type ${t} TYPE)
  if(NOT t STREQUAL "lh_build_id" AND NOT type STREQUAL "INTERFACE_LIBRARY" AND NOT type STREQUAL "UTILITY")
    add_dependencies(${t} lh_build_id)
  endif()
endforeach()
