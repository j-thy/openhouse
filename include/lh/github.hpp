// GET from the public GitHub API (api.github.com), for the update checks: WinHTTP on Windows
// (link winhttp), curl elsewhere. A 5 s timeout; "" on any failure.
#pragma once

#include <cctype>
#include <string>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <winhttp.h>
#else
#include <cstdio>
#include <cstdlib>
#endif

namespace lh {

// path: "/repos/...". Only ours (a tag, commit hashes): anything that isn't a plain path is refused.
inline std::string GithubGet(const std::string& path) {
  std::string body;
  if (path.empty() || path[0] != '/') return body;
  for (const char c : path)
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '/' || c == '.' || c == '-' || c == '_')) return body;
#ifdef _WIN32
  HINTERNET session = WinHttpOpen(L"openhouse", WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, WINHTTP_NO_PROXY_NAME,
                                  WINHTTP_NO_PROXY_BYPASS, 0);
  if (!session) return body;
  WinHttpSetTimeouts(session, 5000, 5000, 5000, 5000);
  HINTERNET connect = WinHttpConnect(session, L"api.github.com", INTERNET_DEFAULT_HTTPS_PORT, 0);
  const std::wstring wpath(path.begin(), path.end());
  HINTERNET request = connect ? WinHttpOpenRequest(connect, L"GET", wpath.c_str(), nullptr, WINHTTP_NO_REFERER,
                                                   WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE)
                              : nullptr;
  if (request && WinHttpSendRequest(request, L"Accept: application/vnd.github+json\r\n", DWORD(-1), WINHTTP_NO_REQUEST_DATA,
                                    0, 0, 0) &&
      WinHttpReceiveResponse(request, nullptr)) {
    DWORD status = 0, size = sizeof status;
    WinHttpQueryHeaders(request, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX,
                        &status, &size, WINHTTP_NO_HEADER_INDEX);
    for (DWORD available = 0; status == 200 && WinHttpQueryDataAvailable(request, &available) && available;) {
      std::string chunk(available, '\0');
      DWORD read = 0;
      if (!WinHttpReadData(request, chunk.data(), available, &read) || !read) break;
      body.append(chunk.data(), read);
    }
  }
  if (request) WinHttpCloseHandle(request);
  if (connect) WinHttpCloseHandle(connect);
  WinHttpCloseHandle(session);
#else
  if (std::system("command -v curl >/dev/null 2>&1") != 0) return body;
  FILE* pipe = popen(("curl -fsSL --max-time 5 -H 'Accept: application/vnd.github+json' 'https://api.github.com" + path +
                      "' 2>/dev/null").c_str(), "r");
  if (!pipe) return body;
  char buffer[4096];
  for (size_t n; (n = std::fread(buffer, 1, sizeof buffer, pipe)) > 0;) body.append(buffer, n);
  if (pclose(pipe) != 0) body.clear();   // curl -f: an HTTP error is a failure
#endif
  return body;
}

}  // namespace lh
