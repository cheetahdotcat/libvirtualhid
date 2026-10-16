/**
 * @file src/platform/esp32/bridge_client.hpp
 * @brief Host-side client for the ESP32-S3 USB bridge wire protocol.
 */
#pragma once

// standard includes
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

// local includes
#include "platform/esp32/esp32_bridge_protocol.h"
#include "platform/esp32/serial_port.hpp"
#include <libvirtualhid/types.hpp>

namespace lvh::detail::esp32 {

  /**
   * @brief Configuration of one enumerated bridge slot.
   */
  struct SlotLayout {
    std::uint8_t kind = LVHB_SLOT_NONE;  ///< #lvhb_slot_kind.
    std::uint8_t interval_ms = 1;  ///< Interrupt IN polling interval.
    std::vector<std::uint8_t> report_descriptor;  ///< HID report descriptor, empty for XInput.
    std::vector<std::uint8_t> neutral_report;  ///< Report used at enumeration and when the host goes quiet.
    std::vector<std::vector<std::uint8_t>> feature_reports;  ///< GET_REPORT(Feature) replies, each starting with its report id.
  };

  /**
   * @brief Complete USB personality requested from the bridge.
   */
  struct BridgeLayout {
    std::uint16_t vendor_id = 0x303A;  ///< USB idVendor.
    std::uint16_t product_id = 0x40F0;  ///< USB idProduct.
    std::uint16_t bcd_device = 0x0100;  ///< USB bcdDevice.
    std::string manufacturer = "libvirtualhid";  ///< Manufacturer string.
    std::string product = "ESP32-S3 HID Bridge";  ///< Product string.
    std::array<std::optional<SlotLayout>, LVHB_MAX_SLOTS> slots {};  ///< Slot configuration.
  };

  /**
   * @brief Report the USB host sent to one slot.
   */
  struct BridgeOutput {
    std::uint8_t slot = 0;  ///< Slot index.
    std::uint8_t kind = LVHB_REPORT_OUTPUT;  ///< #lvhb_report_kind.
    std::vector<std::uint8_t> report;  ///< Report bytes, starting with the report id when the slot uses ids.
  };

  /**
   * @brief Timeouts used by the client; tests shorten them.
   */
  struct BridgeTimings {
    std::chrono::milliseconds ack_timeout {1000};  ///< Wait for one acknowledgement.
    std::chrono::milliseconds connect_timeout {3000};  ///< Wait for the first handshake.
    std::chrono::milliseconds reenumerate_timeout {20000};  ///< Wait for the bridge to come back after a layout change.
    std::chrono::milliseconds reopen_interval {100};  ///< Delay between attempts to reopen the port.
    std::chrono::milliseconds keepalive_interval {LVHB_KEEPALIVE_MS};  ///< Ping period.
    std::chrono::milliseconds silence_timeout {4000};  ///< Reopen the port after this long without any frame.
  };

  /**
   * @brief Diagnostic sink.
   */
  using BridgeLogger = std::function<void(LogLevel level, const std::string &message)>;

  /**
   * @brief Owns the bridge port, its reader thread, and layout state.
   *
   * The reader thread reopens the port whenever it disappears, which happens
   * on every layout change because the bridge re-enumerates. A supervisor
   * thread sends keepalives and restores the desired layout after the bridge
   * resets or drops slots on its own.
   */
  class BridgeClient {
  public:
    /**
     * @brief Create a client. No I/O happens until start().
     *
     * @param opener Opens the bridge port.
     * @param logger Diagnostic sink, may be empty.
     * @param timings Timeouts.
     */
    BridgeClient(SerialPortOpener opener, BridgeLogger logger, BridgeTimings timings = {});

    BridgeClient(const BridgeClient &) = delete;
    BridgeClient &operator=(const BridgeClient &) = delete;
    BridgeClient(BridgeClient &&) noexcept = delete;
    BridgeClient &operator=(BridgeClient &&) noexcept = delete;

    /**
     * @brief Stop the worker threads and close the port.
     */
    ~BridgeClient();

    /**
     * @brief Start the worker threads and wait for the first handshake.
     *
     * @return Success, or `backend_unavailable` when no bridge answers.
     */
    OperationStatus start();

    /**
     * @brief Replace the bridge personality and wait until it re-enumerates.
     *
     * @param layout New layout.
     * @return Apply status.
     */
    OperationStatus apply_layout(const BridgeLayout &layout);

    /**
     * @brief Send an input report to a slot. Not acknowledged.
     *
     * The latest report per gamepad slot is replayed after reconnects;
     * keyboard/mouse reports are events and are not.
     *
     * @param slot Slot index.
     * @param report Report bytes.
     */
    void send_input(std::uint8_t slot, std::span<const std::uint8_t> report);

    /**
     * @brief Register the handler for output and feature reports.
     *
     * The handler runs on the reader thread.
     *
     * @param handler Output handler.
     */
    void set_output_handler(std::function<void(const BridgeOutput &)> handler);

    /**
     * @brief Get the firmware version reported by the last handshake.
     *
     * @return Firmware version string.
     */
    std::string firmware_version() const;

    /**
     * @brief Get the path of the currently open port.
     *
     * @return Port path, or empty while disconnected.
     */
    std::string port_path() const;

    /**
     * @brief Forward a diagnostic message to the logger, swallowing logger exceptions.
     *
     * @param level Severity.
     * @param message Message text.
     */
    void log(LogLevel level, const std::string &message) const;

  private:
    struct PendingAck {
      bool active = false;
      bool received = false;
      std::uint8_t type = 0;
      std::uint8_t sequence = 0;
      std::uint8_t status = 0;
    };

    void reader_loop(std::stop_token stop_token);
    void supervisor_loop(std::stop_token stop_token);
    bool open_port();
    void drop_port();
    bool write_frame(std::uint8_t type, std::span<const std::uint8_t> payload, const std::function<void(std::uint8_t)> &on_sequence = {});
    void handle_frame(std::span<const std::uint8_t> frame);
    OperationStatus transact(std::uint8_t type, std::span<const std::uint8_t> payload);
    OperationStatus apply_layout_locked(const BridgeLayout &layout);
    bool wait_for_generation(std::uint32_t generation, std::chrono::milliseconds timeout);
    void replay_inputs();

    SerialPortOpener opener_;
    BridgeLogger logger_;
    BridgeTimings timings_;

    mutable std::mutex port_mutex_;
    std::shared_ptr<SerialPort> port_;  ///< Guarded by port_mutex_; the reader reads from a copy.
    std::uint8_t next_sequence_ = 0;  ///< Guarded by port_mutex_.

    mutable std::mutex state_mutex_;
    std::condition_variable state_changed_;
    bool connected_ = false;
    std::uint32_t device_generation_ = 0;
    std::string firmware_version_;
    PendingAck pending_ack_;
    std::chrono::steady_clock::time_point last_frame_time_;

    std::mutex layout_mutex_;
    std::optional<BridgeLayout> desired_layout_;
    std::uint32_t desired_generation_ = 0;
    std::uint32_t next_generation_;

    std::mutex input_mutex_;
    std::array<std::vector<std::uint8_t>, LVHB_MAX_SLOTS> latest_inputs_ {};
    std::array<bool, LVHB_MAX_SLOTS> replayable_ {};  ///< Event slots (keyboard/mouse) must never be replayed.

    std::mutex handler_mutex_;
    std::function<void(const BridgeOutput &)> output_handler_;

    std::jthread reader_;
    std::jthread supervisor_;
  };

}  // namespace lvh::detail::esp32
