/**
 * @file src/platform/esp32/bridge_client.cpp
 * @brief Host-side client for the ESP32-S3 USB bridge wire protocol.
 */

// standard includes
#include <algorithm>
#include <utility>

// local includes
#include "platform/esp32/bridge_client.hpp"
#include "platform/esp32/hmac_sha256.hpp"

namespace lvh::detail::esp32 {
  namespace {

    constexpr auto read_timeout = std::chrono::milliseconds {50};
    constexpr auto supervisor_tick = std::chrono::milliseconds {20};

    void append_u16(std::vector<std::uint8_t> &buffer, std::uint16_t value) {
      buffer.push_back(static_cast<std::uint8_t>(value & 0xFFU));
      buffer.push_back(static_cast<std::uint8_t>(value >> 8U));
    }

    void append_u32(std::vector<std::uint8_t> &buffer, std::uint32_t value) {
      for (unsigned shift = 0; shift < 32U; shift += 8U) {
        buffer.push_back(static_cast<std::uint8_t>((value >> shift) & 0xFFU));
      }
    }

    std::uint32_t read_u32(std::span<const std::uint8_t> data) {
      return static_cast<std::uint32_t>(data[0]) | (static_cast<std::uint32_t>(data[1]) << 8U) |
             (static_cast<std::uint32_t>(data[2]) << 16U) | (static_cast<std::uint32_t>(data[3]) << 24U);
    }

    void append_string(std::vector<std::uint8_t> &buffer, const std::string &value) {
      const auto size = std::min<std::size_t>(value.size(), LVHB_MAX_STRING);
      buffer.push_back(static_cast<std::uint8_t>(size));
      buffer.insert(buffer.end(), value.begin(), value.begin() + static_cast<std::ptrdiff_t>(size));
    }

    std::string status_message(std::uint8_t status) {
      switch (status) {
        case LVHB_STATUS_BAD_REQUEST:
          return "bridge rejected a malformed request";
        case LVHB_STATUS_BAD_STATE:
          return "bridge received a layout request out of order";
        case LVHB_STATUS_INVALID_LAYOUT:
          return "layout exceeds the ESP32-S3 endpoint budget";
        case LVHB_STATUS_TOO_LARGE:
          return "descriptor or report exceeds the bridge limits";
        case LVHB_STATUS_VERSION_MISMATCH:
          return "bridge firmware speaks a different protocol version; flash the current firmware";
        case LVHB_STATUS_UNAUTHORIZED:
          return "bridge is not paired with this host";
        default:
          return "bridge returned status " + std::to_string(status);
      }
    }

    bool same_slot(const std::optional<SlotLayout> &left, const std::optional<SlotLayout> &right) {
      if (!left.has_value() || !right.has_value()) {
        return left.has_value() == right.has_value();
      }
      return left->kind == right->kind && left->report_descriptor == right->report_descriptor;
    }

    // The client lvh::esp32_bridge_status() reports on; set by start(), cleared by the destructor.
    std::mutex active_client_mutex;
    const BridgeClient *active_client = nullptr;

    std::uint32_t initial_generation() {
      // Distinct per process so a bridge that kept an older host's layout is never mistaken for ours.
      const auto ticks = static_cast<std::uint32_t>(std::chrono::steady_clock::now().time_since_epoch().count());
      return (ticks & 0x7FFFFFFFU) | 1U;
    }

  }  // namespace

  BridgeClient::BridgeClient(SerialPortOpener opener, BridgeLogger logger, BridgeTimings timings, std::shared_ptr<CredentialStore> credentials, std::string client_name):
      opener_ {std::move(opener)},
      logger_ {std::move(logger)},
      timings_ {timings},
      credentials_ {std::move(credentials)},
      client_name_ {std::move(client_name)},
      next_generation_ {initial_generation()} {}

  BridgeClient::~BridgeClient() {
    {
      std::lock_guard lock {active_client_mutex};
      if (active_client == this) {
        active_client = nullptr;
      }
    }
    if (supervisor_.joinable()) {
      supervisor_.request_stop();
      supervisor_.join();
    }
    if (reader_.joinable()) {
      reader_.request_stop();
      reader_.join();
    }
  }

  OperationStatus BridgeClient::start() {
    {
      std::lock_guard lock {active_client_mutex};
      active_client = this;
    }
    if (!reader_.joinable()) {
      reader_ = std::jthread {[this](std::stop_token stop_token) {
        reader_loop(std::move(stop_token));
      }};
      supervisor_ = std::jthread {[this](std::stop_token stop_token) {
        supervisor_loop(std::move(stop_token));
      }};
    }

    std::unique_lock lock {state_mutex_};
    if (!state_changed_.wait_for(lock, timings_.connect_timeout, [this]() {
          return connected_ || awaiting_pairing_;
        })) {
      return OperationStatus::failure(
        ErrorCode::backend_unavailable,
        "no ESP32-S3 bridge answered (looked for a CDC port with USB serial " LVHB_SERIAL_PREFIX "*)"
      );
    }
    return OperationStatus::success();
  }

  OperationStatus BridgeClient::apply_layout(const BridgeLayout &layout) {
    std::lock_guard lock {layout_mutex_};
    return apply_layout_locked(layout);
  }

  OperationStatus BridgeClient::apply_layout_locked(const BridgeLayout &layout) {
    bool deferred;
    {
      std::unique_lock lock {state_mutex_};
      if (!state_changed_.wait_for(lock, timings_.connect_timeout, [this]() {
            return connected_ || awaiting_pairing_;
          })) {
        return OperationStatus::failure(ErrorCode::backend_unavailable, "ESP32-S3 bridge is not connected");
      }
      deferred = !connected_;
    }

    std::uint8_t kinds[LVHB_MAX_SLOTS] {};
    for (std::size_t index = 0; index < LVHB_MAX_SLOTS; ++index) {
      kinds[index] = layout.slots[index].has_value() ? layout.slots[index]->kind : static_cast<std::uint8_t>(LVHB_SLOT_NONE);
    }
    if (lvhb_layout_fits(kinds) == 0) {
      return OperationStatus::failure(ErrorCode::unsupported_profile, status_message(LVHB_STATUS_INVALID_LAYOUT));
    }

    if (deferred) {
      // The supervisor applies the desired layout once pairing completes.
      track_layout_inputs(layout);
      desired_layout_ = layout;
      desired_generation_ = take_generation();
      log(LogLevel::info, "ESP32-S3 bridge layout will be applied once the bridge is paired");
      return OperationStatus::success();
    }

    std::vector<std::uint8_t> payload;
    append_u16(payload, layout.vendor_id);
    append_u16(payload, layout.product_id);
    append_u16(payload, layout.bcd_device);
    append_string(payload, layout.manufacturer);
    append_string(payload, layout.product);
    if (auto status = transact(LVHB_MSG_LAYOUT_BEGIN, payload); !status.ok()) {
      return status;
    }

    for (std::size_t index = 0; index < LVHB_MAX_SLOTS; ++index) {
      if (!layout.slots[index].has_value()) {
        continue;
      }
      const auto &slot = *layout.slots[index];
      if (slot.report_descriptor.size() > LVHB_MAX_REPORT_DESCRIPTOR || slot.neutral_report.size() > LVHB_MAX_REPORT) {
        return OperationStatus::failure(ErrorCode::unsupported_profile, status_message(LVHB_STATUS_TOO_LARGE));
      }
      payload.clear();
      payload.push_back(static_cast<std::uint8_t>(index));
      payload.push_back(slot.kind);
      payload.push_back(slot.interval_ms);
      append_u16(payload, static_cast<std::uint16_t>(slot.report_descriptor.size()));
      payload.insert(payload.end(), slot.report_descriptor.begin(), slot.report_descriptor.end());
      payload.insert(payload.end(), slot.neutral_report.begin(), slot.neutral_report.end());
      if (auto status = transact(LVHB_MSG_LAYOUT_SLOT, payload); !status.ok()) {
        return status;
      }

      for (const auto &feature : slot.feature_reports) {
        if (feature.empty() || feature.size() > LVHB_MAX_REPORT) {
          return OperationStatus::failure(ErrorCode::unsupported_profile, status_message(LVHB_STATUS_TOO_LARGE));
        }
        payload.clear();
        payload.push_back(static_cast<std::uint8_t>(index));
        payload.insert(payload.end(), feature.begin(), feature.end());
        if (auto status = transact(LVHB_MSG_LAYOUT_FEATURE, payload); !status.ok()) {
          return status;
        }
      }
    }

    const auto generation = take_generation();
    track_layout_inputs(layout);
    auto previous_layout = std::exchange(desired_layout_, layout);
    const auto previous_generation = std::exchange(desired_generation_, generation);

    payload.clear();
    append_u32(payload, generation);
    if (auto status = transact(LVHB_MSG_LAYOUT_COMMIT, payload); !status.ok()) {
      desired_layout_ = std::move(previous_layout);
      desired_generation_ = previous_generation;
      return status;
    }

    log(LogLevel::info, "ESP32-S3 bridge re-enumerating with layout generation " + std::to_string(generation));
    // Some CDC drivers keep a stale handle readable after surprise removal, so
    // reopen explicitly instead of waiting for a read error.
    std::this_thread::sleep_for(std::chrono::milliseconds {100});
    drop_port();
    if (!wait_for_generation(generation, timings_.reenumerate_timeout)) {
      return OperationStatus::failure(
        ErrorCode::backend_failure,
        "ESP32-S3 bridge did not come back after re-enumeration; first-time Windows driver setup can take longer, retry"
      );
    }
    replay_inputs();
    return OperationStatus::success();
  }

  std::uint32_t BridgeClient::take_generation() {
    const auto generation = next_generation_++;
    if (next_generation_ == 0U) {
      next_generation_ = 1U;
    }
    return generation;
  }

  void BridgeClient::track_layout_inputs(const BridgeLayout &layout) {
    std::lock_guard lock {input_mutex_};
    for (std::size_t index = 0; index < LVHB_MAX_SLOTS; ++index) {
      if (!desired_layout_.has_value() || !same_slot(desired_layout_->slots[index], layout.slots[index])) {
        latest_inputs_[index].clear();
      }
      replayable_[index] = layout.slots[index].has_value() && layout.slots[index]->kind != LVHB_SLOT_KEYBOARD_MOUSE;
    }
  }

  void BridgeClient::send_input(std::uint8_t slot, std::span<const std::uint8_t> report) {
    if (slot >= LVHB_MAX_SLOTS || report.empty() || report.size() > LVHB_MAX_REPORT) {
      return;
    }
    {
      std::lock_guard lock {input_mutex_};
      if (replayable_[slot]) {
        latest_inputs_[slot].assign(report.begin(), report.end());
      }
    }
    std::vector<std::uint8_t> payload;
    payload.reserve(report.size() + 1U);
    payload.push_back(slot);
    payload.insert(payload.end(), report.begin(), report.end());
    static_cast<void>(write_frame(LVHB_MSG_INPUT_REPORT, payload));
  }

  void BridgeClient::set_output_handler(std::function<void(const BridgeOutput &)> handler) {
    std::lock_guard lock {handler_mutex_};
    output_handler_ = std::move(handler);
  }

  std::string BridgeClient::firmware_version() const {
    std::lock_guard lock {state_mutex_};
    return firmware_version_;
  }

  Esp32BridgeStatus BridgeClient::status() const {
    Esp32BridgeStatus status;
    status.active = true;
    status.port = port_path();
    std::lock_guard lock {state_mutex_};
    status.connected = connected_;
    status.awaiting_pairing = awaiting_pairing_;
    status.pairing_required = pairing_required_;
    status.device_id = device_id_;
    status.firmware_version = firmware_version_;
    return status;
  }

  Esp32BridgeStatus active_bridge_status() {
    std::lock_guard lock {active_client_mutex};
    return active_client != nullptr ? active_client->status() : Esp32BridgeStatus {};
  }

  std::string BridgeClient::device_id() const {
    std::lock_guard lock {state_mutex_};
    return device_id_;
  }

  bool BridgeClient::awaiting_pairing() const {
    std::lock_guard lock {state_mutex_};
    return awaiting_pairing_;
  }

  std::string BridgeClient::port_path() const {
    std::lock_guard lock {port_mutex_};
    return port_ != nullptr ? port_->path() : std::string {};
  }

  void BridgeClient::reader_loop(std::stop_token stop_token) {
    std::vector<std::uint8_t> encoded;
    std::array<std::uint8_t, 512> chunk {};
    std::array<std::uint8_t, LVHB_MAX_FRAME> frame {};
    bool overflow = false;

    while (!stop_token.stop_requested()) {
      std::shared_ptr<SerialPort> port;
      {
        std::lock_guard lock {port_mutex_};
        port = port_;
      }
      if (port == nullptr) {
        encoded.clear();
        overflow = false;
        if (!open_port()) {
          std::this_thread::sleep_for(timings_.reopen_interval);
        }
        continue;
      }

      const auto count = port->read(chunk, read_timeout);
      if (count < 0) {
        log(LogLevel::debug, "ESP32-S3 bridge port " + port->path() + " closed");
        drop_port();
        continue;
      }
      for (std::ptrdiff_t index = 0; index < count; ++index) {
        const auto byte = chunk[static_cast<std::size_t>(index)];
        if (byte != 0U) {
          if (encoded.size() < LVHB_MAX_ENCODED_FRAME) {
            encoded.push_back(byte);
          } else {
            overflow = true;
          }
          continue;
        }
        if (!overflow && !encoded.empty()) {
          const auto payload_size = lvhb_decode_frame(encoded.data(), encoded.size(), frame.data());
          if (payload_size >= 0) {
            handle_frame(std::span<const std::uint8_t> {frame.data(), static_cast<std::size_t>(payload_size) + 2U});
          } else {
            log(LogLevel::debug, "dropped corrupt frame from ESP32-S3 bridge");
          }
        }
        encoded.clear();
        overflow = false;
      }
    }
  }

  void BridgeClient::supervisor_loop(std::stop_token stop_token) {
    auto next_keepalive = std::chrono::steady_clock::now();
    std::uint32_t ping_token = 0;
    while (!stop_token.stop_requested()) {
      std::this_thread::sleep_for(supervisor_tick);
      const auto now = std::chrono::steady_clock::now();
      if (now < next_keepalive) {
        continue;
      }
      next_keepalive = now + timings_.keepalive_interval;

      bool connected;
      bool retry_pairing;
      std::uint32_t device_generation;
      std::chrono::steady_clock::time_point last_frame;
      {
        std::lock_guard lock {state_mutex_};
        connected = connected_;
        device_generation = device_generation_;
        last_frame = last_frame_time_;
        retry_pairing = awaiting_pairing_ && pairing_retry_at_.has_value() && now >= *pairing_retry_at_;
        if (retry_pairing) {
          pairing_retry_at_.reset();
        }
      }
      if (retry_pairing) {
        request_pairing();
      }
      if (!connected) {
        continue;
      }

      std::vector<std::uint8_t> payload;
      append_u32(payload, ++ping_token);
      static_cast<void>(write_frame(LVHB_MSG_PING, payload));
      if (now - last_frame > timings_.silence_timeout) {
        log(LogLevel::warning, "ESP32-S3 bridge stopped answering; reopening the port");
        drop_port();
        continue;
      }

      std::unique_lock layout_lock {layout_mutex_, std::try_to_lock};
      if (!layout_lock.owns_lock() || !desired_layout_.has_value() || device_generation == desired_generation_) {
        continue;
      }
      log(LogLevel::warning, "ESP32-S3 bridge lost its layout (reset or host timeout); restoring it");
      const auto layout = *desired_layout_;
      if (const auto status = apply_layout_locked(layout); !status.ok()) {
        log(LogLevel::error, "failed to restore ESP32-S3 bridge layout: " + status.message());
      }
    }
  }

  bool BridgeClient::open_port() {
    std::shared_ptr<SerialPort> port = opener_ ? opener_() : nullptr;
    if (port == nullptr) {
      return false;
    }
    log(LogLevel::debug, "opened ESP32-S3 bridge port " + port->path());
    {
      std::lock_guard lock {port_mutex_};
      port_ = std::move(port);
    }
    {
      std::lock_guard lock {state_mutex_};
      last_frame_time_ = std::chrono::steady_clock::now();
    }
    std::vector<std::uint8_t> payload;
    append_u16(payload, LVHB_PROTOCOL_VERSION);
    if (!write_frame(LVHB_MSG_HELLO, payload)) {
      drop_port();
      return false;
    }
    return true;
  }

  void BridgeClient::drop_port() {
    {
      std::lock_guard lock {port_mutex_};
      port_.reset();
    }
    std::lock_guard lock {state_mutex_};
    connected_ = false;
    awaiting_pairing_ = false;
    pairing_retry_at_.reset();
    state_changed_.notify_all();
  }

  bool BridgeClient::write_frame(
    std::uint8_t type,
    std::span<const std::uint8_t> payload,
    const std::function<void(std::uint8_t)> &on_sequence
  ) {
    std::array<std::uint8_t, LVHB_MAX_ENCODED_FRAME> encoded {};
    {
      // Writes stay under the lock so frames from different threads never
      // interleave and sequence numbers go out in order.
      std::lock_guard lock {port_mutex_};
      if (port_ == nullptr) {
        return false;
      }
      const auto sequence = next_sequence_++;
      if (on_sequence) {
        on_sequence(sequence);
      }
      const auto size = lvhb_encode_frame(type, sequence, payload.data(), payload.size(), encoded.data());
      if (size == 0U) {
        return false;
      }
      if (port_->write(std::span<const std::uint8_t> {encoded.data(), size})) {
        return true;
      }
    }
    drop_port();
    return false;
  }

  void BridgeClient::handle_frame(std::span<const std::uint8_t> frame) {
    const auto type = frame[0];
    const auto sequence = frame[1];
    const auto payload = frame.subspan(2);
    bool replay = false;
    bool authenticate_now = false;
    bool pair_now = false;
    {
      std::lock_guard lock {state_mutex_};
      last_frame_time_ = std::chrono::steady_clock::now();
      switch (type) {
        case LVHB_MSG_HELLO_ACK:
          {
            constexpr std::size_t flags_offset = 10U;
            constexpr std::size_t nonce_offset = flags_offset + 1U;
            constexpr std::size_t device_offset = nonce_offset + LVHB_NONCE_SIZE;
            constexpr std::size_t firmware_offset = device_offset + LVHB_DEVICE_ID_SIZE;
            if (payload.size() < firmware_offset) {
              break;
            }
            device_generation_ = read_u32(payload.subspan(4));
            firmware_version_.assign(payload.begin() + firmware_offset, payload.end());
            std::copy_n(payload.begin() + nonce_offset, LVHB_NONCE_SIZE, nonce_.begin());
            std::copy_n(payload.begin() + device_offset, LVHB_DEVICE_ID_SIZE, device_id_bytes_.begin());
            static constexpr char digits[] = "0123456789ABCDEF";
            device_id_.clear();
            for (const auto byte : device_id_bytes_) {
              device_id_.push_back(digits[byte >> 4U]);
              device_id_.push_back(digits[byte & 0x0FU]);
            }
            pairing_required_ = (payload[flags_offset] & LVHB_AUTH_PAIRING_REQUIRED) != 0U;
            if ((payload[flags_offset] & LVHB_AUTH_AUTHORIZED) != 0U) {
              connected_ = true;
              replay = device_generation_ != 0U && device_generation_ == desired_generation_;
              state_changed_.notify_all();
            } else {
              authenticate_now = true;
            }
            break;
          }
        case LVHB_MSG_ACK:
          if (payload.size() >= 3U && payload[0] == LVHB_MSG_AUTH) {
            if (payload[2] == LVHB_STATUS_OK) {
              log(LogLevel::debug, "authenticated with ESP32-S3 bridge " + device_id_);
              connected_ = true;
              awaiting_pairing_ = false;
              replay = device_generation_ != 0U && device_generation_ == desired_generation_;
              state_changed_.notify_all();
            } else {
              pair_now = true;
            }
            break;
          }
          if (payload.size() >= 3U && payload[0] == LVHB_MSG_PAIR) {
            if (payload[2] == LVHB_STATUS_BUSY) {
              pairing_retry_at_ = last_frame_time_ + std::chrono::seconds {1};
            } else if (payload[2] == LVHB_STATUS_FULL) {
              pairing_retry_at_ = last_frame_time_ + std::chrono::milliseconds {LVHB_CONFIRM_WINDOW_MS};
              log(LogLevel::error, "ESP32-S3 bridge already has " + std::to_string(LVHB_MAX_PAIRINGS) + " paired hosts; remove one on the flasher site's configure page, or hold BOOT for 10 s to remove all");
            }
          }
          if (payload.size() >= 3U) {
            if (payload[0] == LVHB_MSG_HELLO && payload[2] == LVHB_STATUS_VERSION_MISMATCH) {
              firmware_version_ = "incompatible";
            }
            if (pending_ack_.active && pending_ack_.type == payload[0] && pending_ack_.sequence == payload[1]) {
              pending_ack_.received = true;
              pending_ack_.status = payload[2];
              state_changed_.notify_all();
            }
          }
          break;
        case LVHB_MSG_STATUS:
          if (payload.size() >= 6U) {
            device_generation_ = read_u32(payload.subspan(2));
            state_changed_.notify_all();
          }
          break;
        case LVHB_MSG_CONFIRM_RESULT:
          if (payload.size() >= 2U && payload[0] == LVHB_MSG_PAIR && awaiting_pairing_) {
            if (payload[1] == LVHB_STATUS_OK && payload.size() >= 2U + LVHB_KEY_ID_SIZE + LVHB_SECRET_SIZE) {
              BridgeCredential credential;
              std::copy_n(payload.begin() + 2, LVHB_KEY_ID_SIZE, credential.key_id.begin());
              std::copy_n(payload.begin() + 2 + LVHB_KEY_ID_SIZE, LVHB_SECRET_SIZE, credential.secret.begin());
              if (credentials_ == nullptr || !credentials_->save(device_id_, credential)) {
                log(LogLevel::error, "paired with ESP32-S3 bridge " + device_id_ + " but could not store the pairing; it will ask again next time");
              } else {
                log(LogLevel::info, "paired with ESP32-S3 bridge " + device_id_);
              }
              // The bridge authorizes the session that completed pairing.
              connected_ = true;
              awaiting_pairing_ = false;
              pairing_announced_ = false;
              replay = device_generation_ != 0U && device_generation_ == desired_generation_;
              state_changed_.notify_all();
            } else {
              // Keep a request open while unpaired so a later BOOT press still pairs.
              pairing_retry_at_ = last_frame_time_ + (payload[1] == LVHB_STATUS_TIMEOUT ? std::chrono::milliseconds {0} : std::chrono::milliseconds {LVHB_CONFIRM_WINDOW_MS});
            }
          }
          break;
        default:
          break;
      }
    }
    static_cast<void>(sequence);

    if (authenticate_now) {
      authenticate();
    } else if (pair_now) {
      log(LogLevel::warning, "ESP32-S3 bridge rejected the stored pairing (it was removed on the bridge); pairing again");
      if (credentials_ != nullptr) {
        credentials_->erase(device_id());
      }
      request_pairing();
    }

    if (type == LVHB_MSG_ACK && payload.size() >= 3U && payload[0] == LVHB_MSG_HELLO && payload[2] == LVHB_STATUS_VERSION_MISMATCH) {
      log(LogLevel::error, status_message(LVHB_STATUS_VERSION_MISMATCH));
    } else if (type == LVHB_MSG_LOG) {
      log(LogLevel::debug, "ESP32-S3 bridge: " + std::string {payload.begin(), payload.end()});
    } else if (type == LVHB_MSG_OUTPUT_REPORT && payload.size() >= 2U && payload[0] < LVHB_MAX_SLOTS) {
      BridgeOutput output;
      output.slot = payload[0];
      output.kind = payload[1];
      output.report.assign(payload.begin() + 2, payload.end());
      std::function<void(const BridgeOutput &)> handler;
      {
        std::lock_guard lock {handler_mutex_};
        handler = output_handler_;
      }
      if (handler) {
        handler(output);
      }
    }

    if (replay) {
      replay_inputs();
    }
  }

  OperationStatus BridgeClient::transact(std::uint8_t type, std::span<const std::uint8_t> payload) {
    // The pending ACK is armed inside write_frame, before the bytes leave, so
    // a fast reply always finds its match.
    const auto arm = [this, type](std::uint8_t sequence) {
      std::lock_guard lock {state_mutex_};
      pending_ack_ = {};
      pending_ack_.active = true;
      pending_ack_.type = type;
      pending_ack_.sequence = sequence;
    };
    if (!write_frame(type, payload, arm)) {
      return OperationStatus::failure(ErrorCode::backend_unavailable, "ESP32-S3 bridge port is not open");
    }

    std::unique_lock lock {state_mutex_};
    const auto received = state_changed_.wait_for(lock, timings_.ack_timeout, [this]() {
      return pending_ack_.received || !connected_;
    });
    const auto acknowledged = received && pending_ack_.received;
    const auto status = pending_ack_.status;
    pending_ack_.active = false;
    if (!acknowledged) {
      return OperationStatus::failure(ErrorCode::backend_failure, "ESP32-S3 bridge did not acknowledge a request");
    }
    if (status != LVHB_STATUS_OK) {
      const auto code = status == LVHB_STATUS_INVALID_LAYOUT || status == LVHB_STATUS_TOO_LARGE ? ErrorCode::unsupported_profile : ErrorCode::backend_failure;
      return OperationStatus::failure(code, status_message(status));
    }
    return OperationStatus::success();
  }

  bool BridgeClient::wait_for_generation(std::uint32_t generation, std::chrono::milliseconds timeout) {
    std::unique_lock lock {state_mutex_};
    return state_changed_.wait_for(lock, timeout, [this, generation]() {
      return connected_ && device_generation_ == generation;
    });
  }

  void BridgeClient::authenticate() {
    std::string device_id;
    std::array<std::uint8_t, LVHB_NONCE_SIZE> nonce;
    std::array<std::uint8_t, LVHB_DEVICE_ID_SIZE> device_bytes;
    {
      std::lock_guard lock {state_mutex_};
      device_id = device_id_;
      nonce = nonce_;
      device_bytes = device_id_bytes_;
    }
    const auto credential = credentials_ != nullptr ? credentials_->load(device_id) : std::nullopt;
    if (!credential.has_value()) {
      request_pairing();
      return;
    }

    std::vector<std::uint8_t> message {LVHB_AUTH_CONTEXT, LVHB_AUTH_CONTEXT + sizeof(LVHB_AUTH_CONTEXT) - 1U};
    message.insert(message.end(), nonce.begin(), nonce.end());
    message.insert(message.end(), credential->key_id.begin(), credential->key_id.end());
    message.insert(message.end(), device_bytes.begin(), device_bytes.end());
    const auto mac = hmac_sha256(credential->secret, message);

    std::vector<std::uint8_t> payload {credential->key_id.begin(), credential->key_id.end()};
    payload.insert(payload.end(), mac.begin(), mac.end());
    static_cast<void>(write_frame(LVHB_MSG_AUTH, payload));
  }

  void BridgeClient::request_pairing() {
    bool announce;
    std::string device_id;
    {
      std::lock_guard lock {state_mutex_};
      awaiting_pairing_ = true;
      announce = !pairing_announced_;
      pairing_announced_ = true;
      device_id = device_id_;
      state_changed_.notify_all();
    }
    if (announce) {
      log(
        LogLevel::warning,
        "ESP32-S3 bridge " + device_id + " is not paired with this host. Press the BOOT button on the bridge to allow " + client_name_ + " to use it."
      );
    }
    std::vector<std::uint8_t> payload;
    const auto size = std::min<std::size_t>(client_name_.size(), LVHB_MAX_PAIRING_NAME);
    payload.push_back(static_cast<std::uint8_t>(size));
    payload.insert(payload.end(), client_name_.begin(), client_name_.begin() + static_cast<std::ptrdiff_t>(size));
    static_cast<void>(write_frame(LVHB_MSG_PAIR, payload));
  }

  void BridgeClient::replay_inputs() {
    std::array<std::vector<std::uint8_t>, LVHB_MAX_SLOTS> inputs;
    {
      std::lock_guard lock {input_mutex_};
      inputs = latest_inputs_;
    }
    for (std::size_t index = 0; index < inputs.size(); ++index) {
      if (inputs[index].empty()) {
        continue;
      }
      std::vector<std::uint8_t> payload;
      payload.push_back(static_cast<std::uint8_t>(index));
      payload.insert(payload.end(), inputs[index].begin(), inputs[index].end());
      static_cast<void>(write_frame(LVHB_MSG_INPUT_REPORT, payload));
    }
  }

  void BridgeClient::log(LogLevel level, const std::string &message) const {
    if (!logger_) {
      return;
    }
    try {
      logger_(level, message);
    } catch (...) {
      // Diagnostics must never take the bridge down.
    }
  }

}  // namespace lvh::detail::esp32
