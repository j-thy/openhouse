#include "pairing_mailbox.hpp"

#include <chrono>
#include <algorithm>
#include <cstring>

#include <nlohmann/json.hpp>

#include <lh/session.hpp>

namespace lh::openvr {
namespace {
using json = nlohmann::json;

double Now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

int StockMailbox::RegisterMailbox(const char* name, uint64_t* handle, char* address, uint32_t addressSize) {
  if (name && std::strcmp(name, kPairingMailbox) == 0) return 1;  // ours answers pairing
  return real_->RegisterMailbox(name, handle, address, addressSize);
}

bool PairingMailbox::Open(vr::IVRDriverContext* context, LogFn log) {
  log_ = log;
  vr::EVRInitError error = vr::VRInitError_None;
  mailbox_ = context ? static_cast<IVRMailbox*>(context->GetGenericInterface(kMailboxVersion, &error)) : nullptr;
  if (!mailbox_ || error != vr::VRInitError_None) {
    if (log_) log_("pairing: the host has no " + std::string(kMailboxVersion) + " (error " + std::to_string(int(error)) +
                   "); SteamVR's pairing menu can't reach us");
    mailbox_ = nullptr;
    return false;
  }
  char address[85] = {};
  const int r = mailbox_->RegisterMailbox(kPairingMailbox, &handle_, address, sizeof address);
  if (r != 0) {
    if (log_) log_("pairing: cannot register the " + std::string(kPairingMailbox) + " mailbox (" + std::to_string(r) + ")");
    mailbox_ = nullptr;
    return false;
  }
  if (log_) log_("pairing: answering SteamVR's dongle pairing (" + std::string(address) + ")");
  return true;
}

void PairingMailbox::Reply(const std::string& address, const std::string& text) {
  if (!mailbox_ || address.empty()) return;
  if (const int r = mailbox_->SendMessage(handle_, address.c_str(), text.c_str()); r != 0 && log_)
    log_("pairing: reply to " + address + " failed (" + std::to_string(r) + ")");
}

std::string PairingMailbox::Discover(Session& session, const std::string& expected, const std::string* messageId) const {
  json j = {{"type", "discover_dongles_response"}, {"isPairing", pairing_.has_value()}, {"dongles", json::array()},
            {"expectedControllerType", expected}};
  for (const auto& r : session.Receivers())
    j["dongles"].push_back({{"serial", r.serial}, {"firmwareVersion", ""}, {"connectedSerial", r.connected}});
  if (messageId) j["message_id"] = json::parse(*messageId);
  return j.dump(-1, ' ', false, json::error_handler_t::replace);  // a receiver serial may not be UTF-8 (#45)
}

void PairingMailbox::Finish(Session& session, const std::string& result, const std::string& connected,
                            const std::string& expected) {
  if (!pairing_) return;
  json j = {{"type", "pairing_result"}, {"result", result}};
  if (!connected.empty()) j["connectedSerial"] = connected;
  if (pairing_->haveMessageId) j["message_id"] = pairing_->messageId;
  const Pairing done = *pairing_;
  pairing_.reset();
  Reply(done.returnAddress, j.dump(-1, ' ', false, json::error_handler_t::replace));
  if (done.sendStatus) Reply(done.returnAddress, Discover(session, expected, nullptr));
  if (log_) log_("pairing: receiver " + done.receiver + ": " + result + (connected.empty() ? "" : " (" + connected + ")"));
}

void PairingMailbox::Poll(Session& session, const std::string& expected) {
  if (!mailbox_) return;
  for (int guard = 0; guard < 16; ++guard) {
    uint32_t needed = 0;
    int r = mailbox_->ReadMessage(handle_, buffer_.data(), uint32_t(buffer_.size()), &needed);
    if (needed > buffer_.size()) {
      buffer_.resize(needed + 1);
      r = mailbox_->ReadMessage(handle_, buffer_.data(), uint32_t(buffer_.size()), &needed);
    }
    if (r != 0 || needed == 0) break;
    const json message = json::parse(std::string(buffer_.data(), strnlen(buffer_.data(), buffer_.size())), nullptr, false);
    if (!message.is_object()) continue;
    // Fields of another JSON type read as missing: value() throws on them (#45).
    const auto text = [&](const char* key) {
      const auto it = message.find(key);
      return it != message.end() && it->is_string() ? it->get<std::string>() : std::string();
    };
    const std::string type = text("type");
    const std::string address = text("returnAddress");
    const std::string id = message.contains("message_id") ? message.at("message_id").dump() : "";
    const std::string* idp = id.empty() ? nullptr : &id;
    if (log_) log_("pairing: " + (type.empty() ? std::string("message without a type") : type) + " from " +
                   (address.empty() ? std::string("(no return address)") : address));
    if (type == "discover_dongles") {
      const std::string reply = Discover(session, expected, idp);
      if (log_) log_("pairing: dongles " + reply);
      Reply(address, reply);
    } else if (type == "start_pairing") {
      const std::string serial = text("serial");
      const double timeout = message.contains("timeoutSeconds") && message.at("timeoutSeconds").is_number()
                                 ? std::clamp(message.at("timeoutSeconds").get<double>(), 0.0, 600.0)
                                 : 10.0;
      // SteamVR's pairing page sends start_pairing again every 5 s while it waits: a
      // keep-alive. As in stock, it re-arms the receiver and moves the deadline, and the
      // pairing goes on. Ending it with "canceled" would tell the page a device paired:
      // it takes "canceled" the same as "success".
      const bool again = pairing_ && pairing_->receiver == serial;
      if (pairing_ && !again) Finish(session, "canceled", "", expected);
      if (!again) {
        std::string before;
        for (const auto& rcv : session.Receivers())
          if (rcv.serial == serial) before = rcv.connected;
        pairing_ = Pairing{};
        pairing_->receiver = serial;
        pairing_->before = before;
        pairing_->sawEmpty = before.empty();
      }
      Pairing& p = *pairing_;
      if (!again || !address.empty()) p.returnAddress = address;
      p.haveMessageId = message.contains("message_id") && message.at("message_id").is_number();
      if (p.haveMessageId) p.messageId = message.at("message_id").get<double>();
      {
        const auto it = message.find("sendStatusUpdates");
        p.sendStatus = it != message.end() && it->is_boolean() && it->get<bool>();
      }
      p.deadline = Now() + timeout;
      if (!session.SetPairing(serial, timeout)) {
        Finish(session, "failed_to_start", "", expected);
        continue;
      }
      if (log_)
        log_("pairing: receiver " + serial + (again ? " still" : "") + " in pairing mode for " +
             std::to_string(int(timeout)) + " s");
      if (p.sendStatus) Reply(address, Discover(session, expected, nullptr));
    } else if (type == "stop_pairing") {
      if (pairing_ && text("serial") == pairing_->receiver) {
        session.SetPairing(pairing_->receiver, 0);
        if (!address.empty()) pairing_->returnAddress = address;
        Finish(session, "canceled", "", expected);
      }
    }
  }
  if (!pairing_) return;
  // A device on the receiver: paired (one connected before must drop off first).
  for (const auto& r : session.Receivers()) {
    if (r.serial != pairing_->receiver) continue;
    if (r.connected.empty()) pairing_->sawEmpty = true;
    else if (pairing_->sawEmpty || r.connected != pairing_->before) {
      Finish(session, "success", r.connected, expected);
      return;
    }
  }
  if (Now() > pairing_->deadline) Finish(session, "timeout", "", expected);
}

}  // namespace lh::openvr
