#include "net.hpp"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace lh::net {
namespace {
#ifdef _WIN32
bool Started() {
  static const bool ok = [] {
    WSADATA data;
    return WSAStartup(MAKEWORD(2, 2), &data) == 0;
  }();
  return ok;
}
bool WouldBlock() { return WSAGetLastError() == WSAEWOULDBLOCK; }
void NonBlocking(Socket s) { u_long on = 1; ioctlsocket(SOCKET(s), FIONBIO, &on); }
#else
bool Started() { return true; }
bool WouldBlock() { return errno == EAGAIN || errno == EWOULDBLOCK; }
void NonBlocking(Socket s) { fcntl(int(s), F_SETFL, fcntl(int(s), F_GETFL, 0) | O_NONBLOCK); }
#endif

sockaddr_in Loopback(int port) {
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(uint16_t(port));
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  return a;
}

void NoDelay(Socket s) {
  int on = 1;
  setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&on), sizeof on);
}
}  // namespace

Socket ListenLoopback(int port, std::string* error) {
  if (!Started()) { if (error) *error = "sockets unavailable"; return kInvalid; }
  const Socket s = Socket(socket(AF_INET, SOCK_STREAM, 0));
  if (s == kInvalid) { if (error) *error = "socket() failed"; return kInvalid; }
#ifndef _WIN32
  int on = 1;
  setsockopt(int(s), SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);  // a restart doesn't wait for TIME_WAIT
#endif
  const sockaddr_in a = Loopback(port);
  if (bind(s, reinterpret_cast<const sockaddr*>(&a), sizeof a) != 0 || listen(s, 4) != 0) {
    if (error) *error = "port " + std::to_string(port) + " is in use";
    Close(s);
    return kInvalid;
  }
  NonBlocking(s);
  return s;
}

Socket Accept(Socket listener) {
  const Socket s = Socket(accept(listener, nullptr, nullptr));
  if (s == kInvalid) return kInvalid;
  NonBlocking(s);
  NoDelay(s);
  return s;
}

Socket ConnectLoopback(int port) {
  if (!Started()) return kInvalid;
  const Socket s = Socket(socket(AF_INET, SOCK_STREAM, 0));
  if (s == kInvalid) return kInvalid;
  NonBlocking(s);
  NoDelay(s);
  const sockaddr_in a = Loopback(port);
  if (connect(s, reinterpret_cast<const sockaddr*>(&a), sizeof a) != 0) {
#ifdef _WIN32
    const bool pending = WSAGetLastError() == WSAEWOULDBLOCK;
#else
    const bool pending = errno == EINPROGRESS;
#endif
    if (!pending) { Close(s); return kInvalid; }
  }
  return s;
}

int ConnectDone(Socket socket) {
  fd_set w, e;
  FD_ZERO(&w); FD_ZERO(&e);
  FD_SET(socket, &w); FD_SET(socket, &e);
  timeval zero{};
  const int n = select(int(socket) + 1, nullptr, &w, &e, &zero);
  if (n < 0) return -1;
  if (n == 0) return 0;
  int err = 0;
  socklen_t len = sizeof err;
  if (getsockopt(socket, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&err), &len) != 0 || err != 0 || FD_ISSET(socket, &e))
    return -1;
  return 1;
}

long SendSome(Socket socket, const char* data, std::size_t size) {
#ifdef _WIN32
  const int n = send(SOCKET(socket), data, int(size), 0);
#else
  const long n = long(send(int(socket), data, size, MSG_NOSIGNAL));
#endif
  if (n >= 0) return long(n);
  return WouldBlock() ? 0 : -1;
}

long RecvSome(Socket socket, char* buffer, std::size_t size) {
#ifdef _WIN32
  const int n = recv(SOCKET(socket), buffer, int(size), 0);
#else
  const long n = long(recv(int(socket), buffer, size, 0));
#endif
  if (n > 0) return long(n);
  if (n == 0) return -1;  // closed
  return WouldBlock() ? 0 : -1;
}

void Close(Socket socket) {
  if (socket == kInvalid) return;
#ifdef _WIN32
  closesocket(SOCKET(socket));
#else
  close(int(socket));
#endif
}

}  // namespace lh::net
