// Flashes an application through Nordic's open USB DFU bootloader (nRF52840 Dongle style, as on
// Holyiot dongles): the serial DFU protocol nrfutil and nRF Connect Programmer speak, over the
// bootloader's USB serial port (VID 1915, PID 521F). Unsigned images, no SoftDevice.
#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace lh::dfu {

struct Image {
  uint32_t base = 0;           // flash address of data[0]
  std::vector<uint8_t> data;   // gaps filled with 0xFF, padded to whole words
};

// An Intel HEX file. Fails on anything outside [minAddress, maxAddress).
bool LoadHex(const std::string& path, uint32_t minAddress, uint32_t maxAddress, Image* image, std::string* error);

// Serial ports of plugged-in Nordic bootloaders: "COM7" on Windows, "/dev/ttyACM0" on Linux.
std::vector<std::string> FindBootloaderPorts();

// Sends the image; the bootloader checks it and starts it. progress: 0..1.
bool Flash(const std::string& port, const Image& image, const std::function<void(float)>& progress, std::string* error);

}  // namespace lh::dfu
