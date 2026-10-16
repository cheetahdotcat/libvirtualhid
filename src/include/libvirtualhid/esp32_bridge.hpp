/**
 * @file src/include/libvirtualhid/esp32_bridge.hpp
 * @brief ESP32-S3 USB bridge discovery helpers.
 */
#pragma once

// standard includes
#include <string>
#include <vector>

namespace lvh {

  /**
   * @brief List serial ports of attached ESP32-S3 USB bridges.
   *
   * Bridges are recognized by their `LVHB-` USB serial number. Discovery is
   * supported on Linux and Windows; other platforms return an empty list.
   *
   * @return Port paths such as `COM5` or `/dev/ttyACM0`.
   */
  std::vector<std::string> find_esp32_bridge_ports();

}  // namespace lvh
