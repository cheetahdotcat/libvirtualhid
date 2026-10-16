/**
 * @file src/platform/esp32/esp32_backend.hpp
 * @brief ESP32-S3 USB bridge backend factory and report helpers.
 */
#pragma once

// standard includes
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// local includes
#include "core/backend.hpp"
#include "platform/esp32/bridge_client.hpp"

namespace lvh::detail::esp32 {

  /**
   * @brief Pack normalized state into an Xbox 360 wired (XUSB) input report.
   *
   * @param state Gamepad state.
   * @return 20-byte XUSB report.
   */
  std::array<std::uint8_t, LVHB_XINPUT_REPORT_SIZE> pack_xinput_report(const GamepadState &state);

  /**
   * @brief Translate an XUSB interrupt OUT report into gamepad outputs.
   *
   * @param report Raw output report.
   * @return Rumble or player LED outputs; empty for unknown reports.
   */
  std::vector<GamepadOutput> parse_xinput_output(std::span<const std::uint8_t> report);

  /**
   * @brief Decide whether a `platform_default` runtime should use the bridge instead.
   *
   * Lets consumers such as Sunshine pick up the bridge without code changes.
   * `LVH_BACKEND=platform` disables the bridge and `LVH_BACKEND=esp32_bridge`
   * forces it. Otherwise a configured port (`LVH_ESP32_PORT` or
   * RuntimeOptions::esp32_bridge_port) or a detected bridge selects it.
   * RuntimeOptions::esp32_bridge_autodetect `false` disables all of this.
   *
   * @param options Runtime options.
   * @param env_backend Value of `LVH_BACKEND`, empty when unset.
   * @param env_port Value of `LVH_ESP32_PORT`, empty when unset.
   * @param bridge_present Probe for an attached bridge; only called when needed.
   * @return Bridge port to use (empty means auto-detect), or empty optional for the platform backend.
   */
  std::optional<std::string> select_bridge_for_platform_default(
    const RuntimeOptions &options,
    std::string_view env_backend,
    std::string_view env_port,
    const std::function<bool()> &bridge_present
  );

  /**
   * @brief Create the bridge backend with an injected client, used by tests.
   *
   * @param client Bridge client; started lazily on the first gamepad.
   * @param fallback Backend used for keyboard, mouse, and other non-gamepad
   *        devices, or `nullptr` to report them unsupported.
   * @return Backend.
   */
  std::unique_ptr<Backend> create_esp32_backend(std::shared_ptr<BridgeClient> client, std::unique_ptr<Backend> fallback);

  /**
   * @brief Create the bridge backend for a runtime.
   *
   * @param options Runtime options; uses `esp32_bridge_port` and `log_callback`.
   * @return Backend.
   */
  std::unique_ptr<Backend> create_esp32_backend(const RuntimeOptions &options);

}  // namespace lvh::detail::esp32
