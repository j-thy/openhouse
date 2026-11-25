#pragma once
// A crash's last words for the openhouse apps: <data dir>/logs/<app>-crash.txt.
//
// On Windows an abort() (an uncaught exception, or the C runtime refusing a bad argument)
// ends in __fastfail, 0xC0000409 in ucrtbase.dll, which no exception filter sees: users
// only get that code. So the runtime's own hooks write first: the exception's message, or
// the refused call, and the stack as module+offset (symbolize against the same build). A
// hardware fault (access violation) goes through the unhandled-exception filter instead.
// Bug reports take these files (src/report/bug_report.cpp).

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <string>
#include <typeinfo>

#include "lh/version.hpp"
#include "platform/platform.hpp"

#ifdef _WIN32
#include <windows.h>
#else
#include <execinfo.h>
#include <unistd.h>
#endif

namespace lh::crash {

namespace detail {
inline std::filesystem::path& File() {
  static std::filesystem::path path;
  return path;
}
inline std::string& App() {
  static std::string app;
  return app;
}

inline void WriteStack(std::FILE* f) {
#ifdef _WIN32
  void* frames[48];
  const USHORT n = RtlCaptureStackBackTrace(1, 48, frames, nullptr);
  for (USHORT i = 0; i < n; ++i) {
    HMODULE module = nullptr;
    wchar_t name[MAX_PATH] = L"?";
    if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           static_cast<LPCWSTR>(frames[i]), &module) && module) {
      wchar_t full[MAX_PATH];
      const DWORD len = GetModuleFileNameW(module, full, MAX_PATH);
      const wchar_t* base = full;
      for (DWORD k = 0; k < len; ++k) if (full[k] == L'\\' || full[k] == L'/') base = full + k + 1;
      lstrcpynW(name, len ? base : L"?", MAX_PATH);
    }
    std::fprintf(f, "  %2u %ls+0x%llx\n", unsigned(i), name,
                 static_cast<unsigned long long>(static_cast<const char*>(frames[i]) - reinterpret_cast<const char*>(module)));
  }
#else
  void* frames[48];
  const int n = backtrace(frames, 48);
  std::fflush(f);
  backtrace_symbols_fd(frames, n, fileno(f));
#endif
}

// Plain C calls only from here on: the process is going down.
inline void Write(const char* what, const char* detail) {
  std::FILE* f = nullptr;
#ifdef _WIN32
  f = _wfopen(File().c_str(), L"a");
#else
  f = std::fopen(File().c_str(), "a");
#endif
  if (!f) return;
  const std::time_t t = std::time(nullptr);
  char stamp[32] = "?";
  if (const std::tm* tm = std::localtime(&t)) std::strftime(stamp, sizeof stamp, "%Y-%m-%d %H:%M:%S", tm);
  std::fprintf(f, "%s %s crashed: %s%s%s\n", stamp, App().c_str(), what, detail && *detail ? ": " : "", detail ? detail : "");
  std::fprintf(f, "  openhouse %s\n", kVersion);
  WriteStack(f);
  std::fclose(f);
}

[[noreturn]] inline void OnTerminate() {
  std::string message = "no exception";
  if (const auto e = std::current_exception()) {
    try {
      std::rethrow_exception(e);
    } catch (const std::exception& x) {
      message = std::string(typeid(x).name()) + ": " + x.what();
    } catch (...) {
      message = "an exception that isn't a std::exception";
    }
  }
  Write("uncaught exception", message.c_str());
  std::signal(SIGABRT, SIG_DFL);
  std::abort();
}

inline void OnAbort(int) {
  Write("abort", nullptr);
  std::signal(SIGABRT, SIG_DFL);
  std::abort();
}

#ifdef _WIN32
inline void OnInvalidParameter(const wchar_t*, const wchar_t*, const wchar_t*, unsigned, uintptr_t) {
  Write("the C runtime refused an argument", nullptr);
  std::signal(SIGABRT, SIG_DFL);
  std::abort();
}

inline LONG WINAPI OnUnhandled(EXCEPTION_POINTERS* info) {
  char text[96];
  std::snprintf(text, sizeof text, "code 0x%08lx at %p", info->ExceptionRecord->ExceptionCode,
                info->ExceptionRecord->ExceptionAddress);
  Write("unhandled exception", text);
  return EXCEPTION_CONTINUE_SEARCH;
}
#endif
}  // namespace detail

// First thing in main().
inline void Install(const char* app) {
  std::error_code ec;
  const auto dir = platform::FsPath(platform::DataDirectory()) / "logs";
  std::filesystem::create_directories(dir, ec);
  detail::File() = dir / (std::string(app) + "-crash.txt");
  detail::App() = app;
  std::set_terminate(detail::OnTerminate);
  std::signal(SIGABRT, detail::OnAbort);
#ifdef _WIN32
  _set_invalid_parameter_handler(detail::OnInvalidParameter);
  SetUnhandledExceptionFilter(detail::OnUnhandled);
#endif
}

}  // namespace lh::crash
