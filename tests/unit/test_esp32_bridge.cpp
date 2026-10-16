/**
 * @file tests/unit/test_esp32_bridge.cpp
 * @brief Unit tests for the ESP32-S3 USB bridge protocol, client, and backend.
 */

// standard includes
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <thread>
#include <vector>

// local includes
#include "fixtures/fixtures.hpp"
#include "platform/esp32/esp32_backend.hpp"
#include "platform/esp32/keyboard_mouse_reports.hpp"

#include <libvirtualhid/profiles.hpp>
#include <libvirtualhid/report.hpp>

namespace {

  using namespace std::chrono_literals;
  using lvh::detail::esp32::BridgeClient;
  using lvh::detail::esp32::BridgeTimings;
  using lvh::detail::esp32::SerialPort;

  /**
   * @brief In-memory model of the bridge firmware's protocol behavior.
   */
  class FakeBridgeDevice: public std::enable_shared_from_this<FakeBridgeDevice> {
  public:
    struct Slot {
      std::uint8_t kind = LVHB_SLOT_NONE;
      std::vector<std::uint8_t> descriptor;
      std::vector<std::uint8_t> neutral;
      std::vector<std::vector<std::uint8_t>> features;
    };

    struct Layout {
      std::uint16_t vendor_id = 0;
      std::uint16_t product_id = 0;
      std::uint32_t generation = 0;
      std::array<Slot, LVHB_MAX_SLOTS> slots {};
    };

    bool present() {
      std::lock_guard lock {mutex_};
      return std::chrono::steady_clock::now() >= present_after_;
    }

    int instance() {
      std::lock_guard lock {mutex_};
      return instance_;
    }

    bool write(int instance, std::span<const std::uint8_t> data) {
      std::lock_guard lock {mutex_};
      if (instance != instance_) {
        return false;
      }
      for (const auto byte : data) {
        if (byte != 0U) {
          rx_.push_back(byte);
          continue;
        }
        std::array<std::uint8_t, LVHB_MAX_FRAME> frame {};
        const auto size = lvhb_decode_frame(rx_.data(), rx_.size(), frame.data());
        rx_.clear();
        if (size >= 0) {
          handle(std::span<const std::uint8_t> {frame.data(), static_cast<std::size_t>(size) + 2U});
        }
      }
      changed_.notify_all();
      return true;
    }

    std::ptrdiff_t read(int instance, std::span<std::uint8_t> buffer, std::chrono::milliseconds timeout) {
      std::unique_lock lock {mutex_};
      changed_.wait_for(lock, timeout, [this, instance]() {
        return instance != instance_ || !to_host_.empty() || pending_reenumeration_;
      });
      if (instance != instance_) {
        return -1;
      }
      if (to_host_.empty() && pending_reenumeration_) {
        reenumerate_locked();
        return -1;
      }
      std::size_t count = 0;
      while (count < buffer.size() && !to_host_.empty()) {
        buffer[count++] = to_host_.front();
        to_host_.pop_front();
      }
      return static_cast<std::ptrdiff_t>(count);
    }

    void emit_output(std::uint8_t slot, std::vector<std::uint8_t> report) {
      std::vector<std::uint8_t> payload {slot, static_cast<std::uint8_t>(LVHB_REPORT_OUTPUT)};
      payload.insert(payload.end(), report.begin(), report.end());
      std::lock_guard lock {mutex_};
      send_locked(LVHB_MSG_OUTPUT_REPORT, payload);
    }

    /**
     * @brief Simulate a power cycle: the bridge comes back with the boot layout.
     */
    void reboot() {
      std::lock_guard lock {mutex_};
      active_ = {};
      ++instance_;
      to_host_.clear();
      present_after_ = std::chrono::steady_clock::now() + 20ms;
      changed_.notify_all();
    }

    Layout active() {
      std::lock_guard lock {mutex_};
      return active_;
    }

    int commits() {
      std::lock_guard lock {mutex_};
      return commits_;
    }

    std::vector<std::pair<std::uint8_t, std::vector<std::uint8_t>>> inputs() {
      std::lock_guard lock {mutex_};
      return inputs_;
    }

    template<typename Predicate>
    bool wait_until(Predicate predicate, std::chrono::milliseconds timeout = 3000ms) {
      const auto deadline = std::chrono::steady_clock::now() + timeout;
      while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
          return true;
        }
        std::this_thread::sleep_for(5ms);
      }
      return predicate();
    }

  private:
    void handle(std::span<const std::uint8_t> frame) {
      const auto type = frame[0];
      const auto sequence = frame[1];
      const auto payload = frame.subspan(2);
      const auto ack = [this, type, sequence](std::uint8_t status) {
        const std::vector<std::uint8_t> reply {type, sequence, status};
        send_locked(LVHB_MSG_ACK, reply);
      };

      switch (type) {
        case LVHB_MSG_HELLO:
          {
            std::vector<std::uint8_t> reply {LVHB_PROTOCOL_VERSION, 0, LVHB_MAX_SLOTS, 1};
            for (unsigned shift = 0; shift < 32U; shift += 8U) {
              reply.push_back(static_cast<std::uint8_t>(active_.generation >> shift));
            }
            reply.push_back(0x00);
            reply.push_back(0x02);
            reply.insert(reply.end(), {'t', 'e', 's', 't'});
            send_locked(LVHB_MSG_HELLO_ACK, reply);
            break;
          }
        case LVHB_MSG_LAYOUT_BEGIN:
          staged_ = Layout {};
          staged_->vendor_id = static_cast<std::uint16_t>(payload[0] | (payload[1] << 8U));
          staged_->product_id = static_cast<std::uint16_t>(payload[2] | (payload[3] << 8U));
          ack(LVHB_STATUS_OK);
          break;
        case LVHB_MSG_LAYOUT_SLOT:
          {
            auto &slot = staged_->slots[payload[0]];
            slot.kind = payload[1];
            const auto descriptor_size = static_cast<std::size_t>(payload[3] | (payload[4] << 8U));
            slot.descriptor.assign(payload.begin() + 5, payload.begin() + 5 + static_cast<std::ptrdiff_t>(descriptor_size));
            slot.neutral.assign(payload.begin() + 5 + static_cast<std::ptrdiff_t>(descriptor_size), payload.end());
            ack(LVHB_STATUS_OK);
            break;
          }
        case LVHB_MSG_LAYOUT_FEATURE:
          staged_->slots[payload[0]].features.emplace_back(payload.begin() + 1, payload.end());
          ack(LVHB_STATUS_OK);
          break;
        case LVHB_MSG_LAYOUT_COMMIT:
          {
            std::uint8_t kinds[LVHB_MAX_SLOTS] {};
            for (std::size_t index = 0; index < LVHB_MAX_SLOTS; ++index) {
              kinds[index] = staged_->slots[index].kind;
            }
            if (lvhb_layout_fits(kinds) == 0) {
              ack(LVHB_STATUS_INVALID_LAYOUT);
              break;
            }
            staged_->generation = static_cast<std::uint32_t>(payload[0] | (payload[1] << 8U) | (payload[2] << 16U) | (payload[3] << 24U));
            ack(LVHB_STATUS_OK);
            pending_reenumeration_ = true;
            break;
          }
        case LVHB_MSG_INPUT_REPORT:
          inputs_.emplace_back(payload[0], std::vector<std::uint8_t> {payload.begin() + 1, payload.end()});
          break;
        case LVHB_MSG_PING:
          send_locked(LVHB_MSG_PONG, std::vector<std::uint8_t> {payload.begin(), payload.end()});
          break;
        default:
          break;
      }
    }

    void reenumerate_locked() {
      pending_reenumeration_ = false;
      active_ = *staged_;
      staged_.reset();
      ++commits_;
      ++instance_;
      to_host_.clear();
      present_after_ = std::chrono::steady_clock::now() + 20ms;
    }

    void send_locked(std::uint8_t type, const std::vector<std::uint8_t> &payload) {
      std::array<std::uint8_t, LVHB_MAX_ENCODED_FRAME> encoded {};
      const auto size = lvhb_encode_frame(type, tx_sequence_++, payload.data(), payload.size(), encoded.data());
      to_host_.insert(to_host_.end(), encoded.begin(), encoded.begin() + static_cast<std::ptrdiff_t>(size));
      changed_.notify_all();
    }

    std::mutex mutex_;
    std::condition_variable changed_;
    std::vector<std::uint8_t> rx_;
    std::deque<std::uint8_t> to_host_;
    std::optional<Layout> staged_;
    Layout active_;
    bool pending_reenumeration_ = false;
    int instance_ = 0;
    int commits_ = 0;
    std::uint8_t tx_sequence_ = 0;
    std::chrono::steady_clock::time_point present_after_ {};
    std::vector<std::pair<std::uint8_t, std::vector<std::uint8_t>>> inputs_;
  };

  /**
   * @brief Serial port bound to one enumeration of a FakeBridgeDevice.
   */
  class FakePort final: public SerialPort {
  public:
    FakePort(std::shared_ptr<FakeBridgeDevice> device, int instance):
        device_ {std::move(device)},
        instance_ {instance} {}

    bool write(std::span<const std::uint8_t> data) override {
      return device_->write(instance_, data);
    }

    std::ptrdiff_t read(std::span<std::uint8_t> buffer, std::chrono::milliseconds timeout) override {
      return device_->read(instance_, buffer, timeout);
    }

    std::string path() const override {
      return "fake" + std::to_string(instance_);
    }

  private:
    std::shared_ptr<FakeBridgeDevice> device_;
    int instance_;
  };

  BridgeTimings fast_timings() {
    BridgeTimings timings;
    timings.ack_timeout = 500ms;
    timings.connect_timeout = 1000ms;
    timings.reenumerate_timeout = 3000ms;
    timings.reopen_interval = 5ms;
    timings.keepalive_interval = 50ms;
    timings.silence_timeout = 2000ms;
    return timings;
  }

  struct BridgeHarness {
    std::shared_ptr<FakeBridgeDevice> device = std::make_shared<FakeBridgeDevice>();
    std::atomic_bool attached = true;
    std::shared_ptr<BridgeClient> client = std::make_shared<BridgeClient>(
      [this]() -> std::unique_ptr<SerialPort> {
        if (!attached || !device->present()) {
          return nullptr;
        }
        return std::make_unique<FakePort>(device, device->instance());
      },
      nullptr,
      fast_timings()
    );
    std::unique_ptr<lvh::detail::Backend> backend = lvh::detail::esp32::create_esp32_backend(client, nullptr);

    std::vector<std::vector<std::uint8_t>> slot_inputs(std::uint8_t slot) {
      std::vector<std::vector<std::uint8_t>> reports;
      for (const auto &[input_slot, report] : device->inputs()) {
        if (input_slot == slot) {
          reports.push_back(report);
        }
      }
      return reports;
    }
  };

  /**
   * @brief Input report sizes in bytes per report id, computed by walking a HID report descriptor.
   */
  struct DescriptorSummary {
    std::map<std::uint8_t, std::size_t> input_bytes;
    std::map<std::uint8_t, std::size_t> output_bytes;
    int collection_depth = 0;
    bool balanced = true;
  };

  DescriptorSummary summarize_descriptor(const std::vector<std::uint8_t> &descriptor) {
    DescriptorSummary summary;
    std::map<std::uint8_t, std::size_t> input_bits;
    std::map<std::uint8_t, std::size_t> output_bits;
    std::uint32_t report_size = 0;
    std::uint32_t report_count = 0;
    std::uint8_t report_id = 0;
    for (std::size_t index = 0; index < descriptor.size();) {
      const auto prefix = descriptor[index];
      const std::size_t size = (prefix & 0x03U) == 3U ? 4U : (prefix & 0x03U);
      std::uint32_t value = 0;
      for (std::size_t byte = 0; byte < size; ++byte) {
        value |= static_cast<std::uint32_t>(descriptor[index + 1U + byte]) << (8U * byte);
      }
      switch (prefix & 0xFCU) {
        case 0x74:
          report_size = value;
          break;
        case 0x94:
          report_count = value;
          break;
        case 0x84:
          report_id = static_cast<std::uint8_t>(value);
          break;
        case 0x80:
          input_bits[report_id] += report_size * report_count;
          break;
        case 0x90:
          output_bits[report_id] += report_size * report_count;
          break;
        case 0xA0:
          ++summary.collection_depth;
          break;
        case 0xC0:
          summary.balanced = summary.balanced && --summary.collection_depth >= 0;
          break;
        default:
          break;
      }
      index += 1U + size;
    }
    for (const auto &[id, bits] : input_bits) {
      summary.input_bytes[id] = 1U + (bits + 7U) / 8U;
    }
    for (const auto &[id, bits] : output_bits) {
      summary.output_bytes[id] = 1U + (bits + 7U) / 8U;
    }
    return summary;
  }

  lvh::CreateGamepadOptions options_for(lvh::DeviceProfile profile) {
    lvh::CreateGamepadOptions options;
    options.profile = std::move(profile);
    return options;
  }

}  // namespace

TEST(Esp32BridgeProtocolTest, FrameRoundTripsPayloadWithZeros) {
  const std::vector<std::uint8_t> payload {0x00, 0x01, 0x00, 0x00, 0xFF, 0x00};
  std::array<std::uint8_t, LVHB_MAX_ENCODED_FRAME> encoded {};
  const auto size = lvhb_encode_frame(LVHB_MSG_INPUT_REPORT, 7, payload.data(), payload.size(), encoded.data());
  ASSERT_GT(size, payload.size());
  EXPECT_EQ(encoded[size - 1U], 0U);
  for (std::size_t index = 0; index + 1U < size; ++index) {
    EXPECT_NE(encoded[index], 0U);
  }

  std::array<std::uint8_t, LVHB_MAX_FRAME> frame {};
  ASSERT_EQ(lvhb_decode_frame(encoded.data(), size - 1U, frame.data()), static_cast<int>(payload.size()));
  EXPECT_EQ(frame[0], LVHB_MSG_INPUT_REPORT);
  EXPECT_EQ(frame[1], 7U);
  EXPECT_TRUE(std::equal(payload.begin(), payload.end(), frame.begin() + 2));
}

TEST(Esp32BridgeProtocolTest, FrameRoundTripsLongRunWithoutZeros) {
  std::vector<std::uint8_t> payload(LVHB_MAX_PAYLOAD, 0x5A);
  std::array<std::uint8_t, LVHB_MAX_ENCODED_FRAME> encoded {};
  const auto size = lvhb_encode_frame(LVHB_MSG_LAYOUT_SLOT, 1, payload.data(), payload.size(), encoded.data());
  ASSERT_NE(size, 0U);
  ASSERT_LE(size, LVHB_MAX_ENCODED_FRAME);
  std::array<std::uint8_t, LVHB_MAX_FRAME> frame {};
  EXPECT_EQ(lvhb_decode_frame(encoded.data(), size - 1U, frame.data()), static_cast<int>(payload.size()));
}

TEST(Esp32BridgeProtocolTest, CorruptedFrameFailsCrc) {
  const std::vector<std::uint8_t> payload {1, 2, 3, 4};
  std::array<std::uint8_t, LVHB_MAX_ENCODED_FRAME> encoded {};
  const auto size = lvhb_encode_frame(LVHB_MSG_PING, 0, payload.data(), payload.size(), encoded.data());
  encoded[3] ^= 0x10U;
  std::array<std::uint8_t, LVHB_MAX_FRAME> frame {};
  EXPECT_EQ(lvhb_decode_frame(encoded.data(), size - 1U, frame.data()), -1);
}

TEST(Esp32BridgeProtocolTest, EndpointBudgetAllowsOneXinputBesideCdc) {
  const std::uint8_t xinput_and_hid[] {LVHB_SLOT_XINPUT, LVHB_SLOT_HID};
  const std::uint8_t two_hid[] {LVHB_SLOT_HID, LVHB_SLOT_HID};
  const std::uint8_t two_xinput[] {LVHB_SLOT_XINPUT, LVHB_SLOT_XINPUT};
  EXPECT_NE(lvhb_layout_fits(xinput_and_hid), 0);
  EXPECT_NE(lvhb_layout_fits(two_hid), 0);
  EXPECT_EQ(lvhb_layout_fits(two_xinput), 0);
}

TEST(Esp32BridgeReportTest, PacksXinputButtonsTriggersAndAxes) {
  lvh::GamepadState state;
  state.buttons.set(lvh::GamepadButton::a);
  state.buttons.set(lvh::GamepadButton::dpad_left);
  state.buttons.set(lvh::GamepadButton::guide);
  state.buttons.set(lvh::GamepadButton::back);
  state.left_trigger = 1.0F;
  state.left_stick = {1.0F, -1.0F};
  state.right_stick = {0.0F, 1.0F};

  const auto report = lvh::detail::esp32::pack_xinput_report(state);
  EXPECT_EQ(report[0], 0x00U);
  EXPECT_EQ(report[1], 0x14U);
  EXPECT_EQ(report[2], 0x04U | 0x20U);
  EXPECT_EQ(report[3], 0x10U | 0x04U);
  EXPECT_EQ(report[4], 0xFFU);
  EXPECT_EQ(report[5], 0x00U);
  EXPECT_EQ(report[6] | (report[7] << 8U), 0x7FFF);
  EXPECT_EQ(report[8] | (report[9] << 8U), 0x8000);
  EXPECT_EQ(report[10] | (report[11] << 8U), 0x0000);
  EXPECT_EQ(report[12] | (report[13] << 8U), 0x7FFF);
}

TEST(Esp32BridgeReportTest, ParsesXinputRumbleAndLeds) {
  const std::array<std::uint8_t, 8> rumble {0x00, 0x08, 0x00, 0xFF, 0x80, 0x00, 0x00, 0x00};
  const auto rumble_outputs = lvh::detail::esp32::parse_xinput_output(rumble);
  ASSERT_EQ(rumble_outputs.size(), 1U);
  EXPECT_EQ(rumble_outputs[0].kind, lvh::GamepadOutputKind::rumble);
  EXPECT_EQ(rumble_outputs[0].low_frequency_rumble, 0xFFFFU);
  EXPECT_EQ(rumble_outputs[0].high_frequency_rumble, 0x8080U);

  const std::array<std::uint8_t, 3> led {0x01, 0x03, 0x07};
  const auto led_outputs = lvh::detail::esp32::parse_xinput_output(led);
  ASSERT_EQ(led_outputs.size(), 1U);
  EXPECT_EQ(led_outputs[0].kind, lvh::GamepadOutputKind::player_leds);
  EXPECT_TRUE(led_outputs[0].player_leds[1]);
  EXPECT_FALSE(led_outputs[0].flashing_player_leds[1]);

  EXPECT_TRUE(lvh::detail::esp32::parse_xinput_output(std::array<std::uint8_t, 2> {0x02, 0x00}).empty());
}

TEST(Esp32BridgeBackendTest, ReportsBackendUnavailableWithoutBridge) {
  BridgeHarness harness;
  harness.attached = false;
  auto result = harness.backend->create_gamepad(1, options_for(lvh::profiles::xbox_360()));
  EXPECT_FALSE(result);
  EXPECT_EQ(result.status.code(), lvh::ErrorCode::backend_unavailable);
}

TEST(Esp32BridgeBackendTest, AdvertisesDriverlessGamepads) {
  BridgeHarness harness;
  const auto &caps = harness.backend->capabilities();
  EXPECT_EQ(caps.backend_name, "esp32_bridge");
  EXPECT_TRUE(caps.supports_gamepad);
  EXPECT_TRUE(caps.supports_output_reports);
  EXPECT_TRUE(caps.supports_keyboard);
  EXPECT_TRUE(caps.supports_mouse);
  EXPECT_FALSE(caps.requires_installed_driver);
  EXPECT_FALSE(harness.backend->create_touchscreen(1, {}));
}

TEST(Esp32BridgeBackendTest, XboxProfileEnumeratesXinputSlotAndForwardsReports) {
  BridgeHarness harness;
  auto created = harness.backend->create_gamepad(1, options_for(lvh::profiles::xbox_series()));
  ASSERT_TRUE(created) << created.status.message();
  ASSERT_TRUE(created.effective_profile.has_value());
  EXPECT_EQ(created.effective_profile->gamepad_kind, lvh::GamepadProfileKind::xbox_360);

  const auto layout = harness.device->active();
  EXPECT_EQ(harness.device->commits(), 1);
  EXPECT_EQ(layout.slots[0].kind, LVHB_SLOT_XINPUT);
  EXPECT_EQ(layout.slots[1].kind, LVHB_SLOT_NONE);
  EXPECT_EQ(layout.vendor_id, 0x303AU);
  EXPECT_EQ(layout.slots[0].neutral.size(), LVHB_XINPUT_REPORT_SIZE);

  std::mutex outputs_mutex;
  std::vector<lvh::GamepadOutput> outputs;
  created.gamepad->set_output_callback([&](const lvh::GamepadOutput &output) {
    std::lock_guard lock {outputs_mutex};
    outputs.push_back(output);
  });

  lvh::GamepadState state;
  state.buttons.set(lvh::GamepadButton::y);
  ASSERT_TRUE(created.gamepad->submit(state, {}).ok());
  ASSERT_TRUE(harness.device->wait_until([&]() {
    return !harness.device->inputs().empty();
  }));
  const auto input = harness.device->inputs().back();
  EXPECT_EQ(input.first, 0U);
  ASSERT_EQ(input.second.size(), LVHB_XINPUT_REPORT_SIZE);
  EXPECT_EQ(input.second[3], 0x80U);

  harness.device->emit_output(0, {0x00, 0x08, 0x00, 0x40, 0x20, 0x00, 0x00, 0x00});
  ASSERT_TRUE(harness.device->wait_until([&]() {
    std::lock_guard lock {outputs_mutex};
    return !outputs.empty();
  }));
  std::lock_guard lock {outputs_mutex};
  EXPECT_EQ(outputs[0].kind, lvh::GamepadOutputKind::rumble);
  EXPECT_EQ(outputs[0].low_frequency_rumble, 0x40U * 257U);
}

TEST(Esp32BridgeBackendTest, DualSenseUsesHidSlotWithSonyIdentityAndFeatureReports) {
  BridgeHarness harness;
  auto xbox = harness.backend->create_gamepad(1, options_for(lvh::profiles::xbox_360()));
  ASSERT_TRUE(xbox) << xbox.status.message();
  auto dualsense = harness.backend->create_gamepad(2, options_for(lvh::profiles::dualsense_bluetooth()));
  ASSERT_TRUE(dualsense) << dualsense.status.message();
  ASSERT_TRUE(dualsense.effective_profile.has_value());
  EXPECT_EQ(dualsense.effective_profile->bus_type, lvh::BusType::usb);

  const auto usb_profile = lvh::profiles::dualsense_usb();
  const auto layout = harness.device->active();
  EXPECT_EQ(harness.device->commits(), 2);
  EXPECT_EQ(layout.slots[0].kind, LVHB_SLOT_XINPUT);
  EXPECT_EQ(layout.slots[1].kind, LVHB_SLOT_HID);
  EXPECT_EQ(layout.vendor_id, usb_profile.vendor_id);
  EXPECT_EQ(layout.product_id, usb_profile.product_id);
  EXPECT_EQ(layout.slots[1].descriptor, usb_profile.report_descriptor);
  // DualSense reports carry a sequence counter and timestamp, so compare shape only.
  const auto neutral = lvh::reports::pack_input_report(usb_profile, {});
  ASSERT_EQ(layout.slots[1].neutral.size(), neutral.size());
  EXPECT_EQ(layout.slots[1].neutral[0], neutral[0]);
  ASSERT_EQ(layout.slots[1].features.size(), 3U);
  EXPECT_EQ(layout.slots[1].features[0][0], 0x05U);
  EXPECT_EQ(layout.slots[1].features[1][0], 0x09U);
  EXPECT_EQ(layout.slots[1].features[2][0], 0x20U);
}

TEST(Esp32BridgeBackendTest, RejectsSecondXinputPadAndThirdPad) {
  BridgeHarness harness;
  auto first = harness.backend->create_gamepad(1, options_for(lvh::profiles::xbox_360()));
  ASSERT_TRUE(first) << first.status.message();
  auto second_xbox = harness.backend->create_gamepad(2, options_for(lvh::profiles::xbox_one()));
  EXPECT_FALSE(second_xbox);
  EXPECT_EQ(second_xbox.status.code(), lvh::ErrorCode::unsupported_profile);
  EXPECT_EQ(harness.device->commits(), 1);

  auto second = harness.backend->create_gamepad(3, options_for(lvh::profiles::dualshock4()));
  ASSERT_TRUE(second) << second.status.message();
  auto third = harness.backend->create_gamepad(4, options_for(lvh::profiles::dualshock4()));
  EXPECT_FALSE(third);
  EXPECT_EQ(third.status.code(), lvh::ErrorCode::unsupported_profile);
}

TEST(Esp32BridgeBackendTest, RejectsMixedHidIdentities) {
  BridgeHarness harness;
  auto dualsense = harness.backend->create_gamepad(1, options_for(lvh::profiles::dualsense()));
  ASSERT_TRUE(dualsense) << dualsense.status.message();
  auto dualshock = harness.backend->create_gamepad(2, options_for(lvh::profiles::dualshock4()));
  EXPECT_FALSE(dualshock);
  EXPECT_EQ(dualshock.status.code(), lvh::ErrorCode::unsupported_profile);
}

TEST(Esp32BridgeBackendTest, ClosingGamepadReleasesSlot) {
  BridgeHarness harness;
  auto created = harness.backend->create_gamepad(1, options_for(lvh::profiles::xbox_360()));
  ASSERT_TRUE(created) << created.status.message();
  ASSERT_TRUE(created.gamepad->close().ok());
  EXPECT_EQ(harness.device->commits(), 2);
  EXPECT_EQ(harness.device->active().slots[0].kind, LVHB_SLOT_NONE);
  EXPECT_EQ(created.gamepad->submit({}, {}).code(), lvh::ErrorCode::device_closed);

  auto again = harness.backend->create_gamepad(2, options_for(lvh::profiles::xbox_360()));
  EXPECT_TRUE(again) << again.status.message();
}

TEST(Esp32BridgeBackendTest, RestoresLayoutAndLatestInputAfterBridgeReboot) {
  BridgeHarness harness;
  auto created = harness.backend->create_gamepad(1, options_for(lvh::profiles::xbox_360()));
  ASSERT_TRUE(created) << created.status.message();
  lvh::GamepadState state;
  state.buttons.set(lvh::GamepadButton::b);
  ASSERT_TRUE(created.gamepad->submit(state, {}).ok());

  harness.device->reboot();
  ASSERT_TRUE(harness.device->wait_until([&]() {
    return harness.device->commits() >= 2 && harness.device->active().slots[0].kind == LVHB_SLOT_XINPUT;
  }));
  ASSERT_TRUE(harness.device->wait_until([&]() {
    const auto inputs = harness.device->inputs();
    return inputs.size() >= 2U && inputs.back().second.size() == LVHB_XINPUT_REPORT_SIZE && inputs.back().second[3] == 0x20U;
  }));
}

TEST(Esp32BridgeKeyboardMouseReportTest, DescriptorMatchesPackedReportSizes) {
  const auto summary = summarize_descriptor(lvh::detail::esp32::keyboard_mouse_report_descriptor());
  EXPECT_TRUE(summary.balanced);
  EXPECT_EQ(summary.collection_depth, 0);
  ASSERT_EQ(summary.input_bytes.size(), 3U);
  EXPECT_EQ(summary.input_bytes.at(LVHB_KM_KEYBOARD_REPORT_ID), std::size_t {LVHB_KM_KEYBOARD_REPORT_SIZE});
  EXPECT_EQ(summary.input_bytes.at(LVHB_KM_RELATIVE_REPORT_ID), std::size_t {LVHB_KM_RELATIVE_REPORT_SIZE});
  EXPECT_EQ(summary.input_bytes.at(LVHB_KM_ABSOLUTE_REPORT_ID), std::size_t {LVHB_KM_ABSOLUTE_REPORT_SIZE});
  EXPECT_EQ(summary.output_bytes.at(LVHB_KM_KEYBOARD_REPORT_ID), 2U);
  EXPECT_LE(lvh::detail::esp32::keyboard_mouse_report_descriptor().size(), std::size_t {LVHB_MAX_REPORT_DESCRIPTOR});
}

TEST(Esp32BridgeKeyboardMouseReportTest, NeutralReportsAreLengthPrefixed) {
  const auto neutral = lvh::detail::esp32::keyboard_mouse_neutral_reports();
  ASSERT_EQ(neutral.size(), 2U + LVHB_KM_KEYBOARD_REPORT_SIZE + LVHB_KM_RELATIVE_REPORT_SIZE);
  EXPECT_EQ(neutral[0], LVHB_KM_KEYBOARD_REPORT_SIZE);
  EXPECT_EQ(neutral[1], LVHB_KM_KEYBOARD_REPORT_ID);
  EXPECT_EQ(neutral[1U + LVHB_KM_KEYBOARD_REPORT_SIZE], LVHB_KM_RELATIVE_REPORT_SIZE);
  EXPECT_EQ(neutral[2U + LVHB_KM_KEYBOARD_REPORT_SIZE], LVHB_KM_RELATIVE_REPORT_ID);
}

TEST(Esp32BridgeKeyboardMouseReportTest, MapsVirtualKeysToHidUsages) {
  using lvh::detail::esp32::hid_usage_from_virtual_key;
  EXPECT_EQ(hid_usage_from_virtual_key(0x41, false), 0x04U);  // A
  EXPECT_EQ(hid_usage_from_virtual_key(0x5A, false), 0x1DU);  // Z
  EXPECT_EQ(hid_usage_from_virtual_key(0x31, false), 0x1EU);  // 1
  EXPECT_EQ(hid_usage_from_virtual_key(0x30, false), 0x27U);  // 0
  EXPECT_EQ(hid_usage_from_virtual_key(0x60, false), 0x62U);  // Keypad 0
  EXPECT_EQ(hid_usage_from_virtual_key(0x69, false), 0x61U);  // Keypad 9
  EXPECT_EQ(hid_usage_from_virtual_key(0x70, false), 0x3AU);  // F1
  EXPECT_EQ(hid_usage_from_virtual_key(0x87, false), 0x73U);  // F24
  EXPECT_EQ(hid_usage_from_virtual_key(0x0D, false), 0x28U);  // Enter
  EXPECT_EQ(hid_usage_from_virtual_key(0x0D, true), 0x58U);  // Keypad Enter
  EXPECT_EQ(hid_usage_from_virtual_key(0x11, true), 0xE4U);  // Right Control
  EXPECT_EQ(hid_usage_from_virtual_key(0xA5, false), 0xE6U);  // Right Alt
  EXPECT_EQ(hid_usage_from_virtual_key(0xDE, false), 0x34U);  // '
  EXPECT_FALSE(hid_usage_from_virtual_key(0x01, false).has_value());  // Left mouse button
}

TEST(Esp32BridgeKeyboardMouseReportTest, MapsAsciiToUsLayout) {
  using lvh::detail::esp32::hid_usage_from_ascii;
  EXPECT_EQ(hid_usage_from_ascii('a'), (std::pair<std::uint8_t, bool> {0x04, false}));
  EXPECT_EQ(hid_usage_from_ascii('Q'), (std::pair<std::uint8_t, bool> {0x14, true}));
  EXPECT_EQ(hid_usage_from_ascii(')'), (std::pair<std::uint8_t, bool> {0x27, true}));
  EXPECT_EQ(hid_usage_from_ascii('?'), (std::pair<std::uint8_t, bool> {0x38, true}));
  EXPECT_FALSE(hid_usage_from_ascii('\x7F').has_value());
}

TEST(Esp32BridgeKeyboardMouseReportTest, KeyboardStateTracksModifiersAndRollover) {
  lvh::detail::esp32::KeyboardReportState state;
  EXPECT_TRUE(state.apply(0xE1, true));  // Left Shift
  EXPECT_FALSE(state.apply(0xE1, true));
  for (std::uint8_t usage = 0x04; usage < 0x0B; ++usage) {
    EXPECT_TRUE(state.apply(usage, true));
  }
  auto report = state.report();
  EXPECT_EQ(report[0], LVHB_KM_KEYBOARD_REPORT_ID);
  EXPECT_EQ(report[1], 0x02U);
  // The seventh key displaces the oldest.
  EXPECT_EQ(report[3], 0x05U);
  EXPECT_EQ(report[8], 0x0AU);
  EXPECT_FALSE(state.apply(0x04, false));
  EXPECT_TRUE(state.apply(0x05, false));
  EXPECT_TRUE(state.apply(0xE1, false));
  report = state.report_with(0x2C, true);
  EXPECT_EQ(report[1], 0x02U);
  EXPECT_EQ(report[8], 0x2CU);
}

TEST(Esp32BridgeKeyboardMouseReportTest, ScalesAbsolutePointerThroughViewport) {
  using lvh::detail::esp32::scale_absolute_pointer_axis;
  EXPECT_EQ(scale_absolute_pointer_axis(0.0F, 1920, 0, 0, 0, 0), 0U);
  EXPECT_EQ(scale_absolute_pointer_axis(1920.0F, 1920, 0, 0, 0, 0), 32767U);
  EXPECT_EQ(scale_absolute_pointer_axis(960.0F, 1920, 0, 0, 0, 0), 16384U);
  EXPECT_EQ(scale_absolute_pointer_axis(-5.0F, 1920, 0, 0, 0, 0), 0U);
  // Right half of a 3840-wide desktop that starts at -1920.
  EXPECT_EQ(scale_absolute_pointer_axis(0.0F, 1920, 0, 1920, -1920, 3840), 16384U);
  EXPECT_EQ(scale_absolute_pointer_axis(1920.0F, 1920, 0, 1920, -1920, 3840), 32758U);
  EXPECT_EQ(scale_absolute_pointer_axis(10.0F, 0, 0, 0, 0, 0), 0U);
}

TEST(Esp32BridgeKeyboardMouseTest, KeyboardAndMouseShareOneSlot) {
  BridgeHarness harness;
  auto keyboard = harness.backend->create_keyboard(1, {});
  ASSERT_TRUE(keyboard) << keyboard.status.message();
  auto mouse = harness.backend->create_mouse(2, {});
  ASSERT_TRUE(mouse) << mouse.status.message();
  EXPECT_EQ(harness.device->commits(), 1);
  const auto layout = harness.device->active();
  EXPECT_EQ(layout.slots[0].kind, LVHB_SLOT_KEYBOARD_MOUSE);
  EXPECT_EQ(layout.slots[0].descriptor, lvh::detail::esp32::keyboard_mouse_report_descriptor());
  EXPECT_EQ(layout.slots[0].neutral, lvh::detail::esp32::keyboard_mouse_neutral_reports());
  EXPECT_EQ(layout.slots[1].kind, LVHB_SLOT_NONE);
  EXPECT_FALSE(harness.backend->create_keyboard(3, {}));
}

TEST(Esp32BridgeKeyboardMouseTest, KeyboardSendsReportsOnlyOnChange) {
  BridgeHarness harness;
  auto keyboard = harness.backend->create_keyboard(1, {});
  ASSERT_TRUE(keyboard) << keyboard.status.message();
  ASSERT_TRUE(keyboard.keyboard->submit({.key_code = 0xA0, .pressed = true}).ok());
  ASSERT_TRUE(keyboard.keyboard->submit({.key_code = 0x41, .pressed = true}).ok());
  ASSERT_TRUE(keyboard.keyboard->submit({.key_code = 0x41, .pressed = true}).ok());
  ASSERT_TRUE(keyboard.keyboard->submit({.key_code = 0x41, .pressed = false}).ok());
  EXPECT_EQ(keyboard.keyboard->submit({.key_code = 0x01, .pressed = true}).code(), lvh::ErrorCode::invalid_argument);
  ASSERT_TRUE(harness.device->wait_until([&]() {
    return harness.slot_inputs(0).size() >= 3U;
  }));
  const auto reports = harness.slot_inputs(0);
  ASSERT_EQ(reports.size(), 3U);
  EXPECT_EQ(reports[0], (std::vector<std::uint8_t> {1, 0x02, 0, 0, 0, 0, 0, 0, 0}));
  EXPECT_EQ(reports[1], (std::vector<std::uint8_t> {1, 0x02, 0, 0x04, 0, 0, 0, 0, 0}));
  EXPECT_EQ(reports[2], (std::vector<std::uint8_t> {1, 0x02, 0, 0, 0, 0, 0, 0, 0}));
}

TEST(Esp32BridgeKeyboardMouseTest, TypesAsciiTextAndRejectsUnicodeWithoutFallback) {
  BridgeHarness harness;
  auto keyboard = harness.backend->create_keyboard(1, {});
  ASSERT_TRUE(keyboard) << keyboard.status.message();
  ASSERT_TRUE(keyboard.keyboard->type_text({.text = "aA"}).ok());
  ASSERT_TRUE(harness.device->wait_until([&]() {
    return harness.slot_inputs(0).size() >= 4U;
  }));
  const auto reports = harness.slot_inputs(0);
  EXPECT_EQ(reports[0][3], 0x04U);
  EXPECT_EQ(reports[0][1], 0x00U);
  EXPECT_EQ(reports[1][3], 0x00U);
  EXPECT_EQ(reports[2][1], 0x02U);
  EXPECT_EQ(reports[2][3], 0x04U);
  EXPECT_EQ(reports[3][1], 0x00U);
  EXPECT_EQ(keyboard.keyboard->type_text({.text = "\xC3\xA9"}).code(), lvh::ErrorCode::unsupported_profile);
}

TEST(Esp32BridgeKeyboardMouseTest, MouseSendsRelativeButtonScrollAndAbsoluteReports) {
  BridgeHarness harness;
  lvh::CreateMouseOptions options;
  options.desktop = {.offset_x = 0, .offset_y = 0, .width = 1920, .height = 1080};
  options.viewport = {.offset_x = 0, .offset_y = 0, .width = 1920, .height = 1080};
  auto mouse = harness.backend->create_mouse(1, options);
  ASSERT_TRUE(mouse) << mouse.status.message();
  auto &device = *mouse.mouse;
  ASSERT_TRUE(device.submit({.kind = lvh::MouseEventKind::relative_motion, .x = 40000, .y = -3}).ok());
  ASSERT_TRUE(device.submit({.kind = lvh::MouseEventKind::button, .button = lvh::MouseButton::right, .pressed = true}).ok());
  ASSERT_TRUE(device.submit({.kind = lvh::MouseEventKind::vertical_scroll, .high_resolution_scroll = 60}).ok());
  ASSERT_TRUE(device.submit({.kind = lvh::MouseEventKind::vertical_scroll, .high_resolution_scroll = 60}).ok());
  ASSERT_TRUE(device.submit({.kind = lvh::MouseEventKind::horizontal_scroll, .high_resolution_scroll = -240}).ok());
  ASSERT_TRUE(device.submit({.kind = lvh::MouseEventKind::absolute_motion, .x = 1920, .y = 0, .width = 1920, .height = 1080}).ok());
  ASSERT_TRUE(device.submit({.kind = lvh::MouseEventKind::button, .button = lvh::MouseButton::right, .pressed = false}).ok());
  EXPECT_EQ(device.submit({.kind = lvh::MouseEventKind::absolute_motion, .x = 1, .y = 1}).code(), lvh::ErrorCode::invalid_argument);

  ASSERT_TRUE(harness.device->wait_until([&]() {
    return harness.slot_inputs(0).size() >= 7U;
  }));
  const auto reports = harness.slot_inputs(0);
  ASSERT_EQ(reports.size(), 7U);
  // 40000 splits into 32767 + 7233.
  EXPECT_EQ(reports[0], (std::vector<std::uint8_t> {2, 0, 0xFF, 0x7F, 0xFD, 0xFF, 0, 0}));
  EXPECT_EQ(reports[1], (std::vector<std::uint8_t> {2, 0, 0x41, 0x1C, 0, 0, 0, 0}));
  EXPECT_EQ(reports[2], (std::vector<std::uint8_t> {2, 0x04, 0, 0, 0, 0, 0, 0}));
  // Two half detents make one wheel click.
  EXPECT_EQ(reports[3], (std::vector<std::uint8_t> {2, 0x04, 0, 0, 0, 0, 1, 0}));
  EXPECT_EQ(reports[4], (std::vector<std::uint8_t> {2, 0x04, 0, 0, 0, 0, 0, 0xFE}));
  EXPECT_EQ(reports[5][0], LVHB_KM_ABSOLUTE_REPORT_ID);
  EXPECT_EQ(reports[5][1], 0x04U);
  EXPECT_EQ(reports[5][2] | (reports[5][3] << 8U), 32750);
  // After absolute motion, buttons travel with the absolute position.
  EXPECT_EQ(reports[6], (std::vector<std::uint8_t> {3, 0, reports[5][2], reports[5][3], 0, 0}));
}

TEST(Esp32BridgeKeyboardMouseTest, KeyboardMouseFitsBesideXinputButNotTwoPads) {
  BridgeHarness harness;
  auto keyboard = harness.backend->create_keyboard(1, {});
  ASSERT_TRUE(keyboard) << keyboard.status.message();
  auto xbox = harness.backend->create_gamepad(2, options_for(lvh::profiles::xbox_360()));
  ASSERT_TRUE(xbox) << xbox.status.message();
  const auto layout = harness.device->active();
  EXPECT_EQ(layout.slots[0].kind, LVHB_SLOT_KEYBOARD_MOUSE);
  EXPECT_EQ(layout.slots[1].kind, LVHB_SLOT_XINPUT);
  auto second_pad = harness.backend->create_gamepad(3, options_for(lvh::profiles::dualsense()));
  EXPECT_FALSE(second_pad);
  EXPECT_EQ(second_pad.status.code(), lvh::ErrorCode::unsupported_profile);
  // The mouse joins the existing keyboard/mouse slot without re-enumerating.
  auto mouse = harness.backend->create_mouse(4, {});
  EXPECT_TRUE(mouse) << mouse.status.message();
  EXPECT_EQ(harness.device->commits(), 2);
}

TEST(Esp32BridgeKeyboardMouseTest, SlotStaysUntilKeyboardAndMouseClose) {
  BridgeHarness harness;
  auto keyboard = harness.backend->create_keyboard(1, {});
  auto mouse = harness.backend->create_mouse(2, {});
  ASSERT_TRUE(keyboard && mouse);
  ASSERT_TRUE(keyboard.keyboard->submit({.key_code = 0x41, .pressed = true}).ok());
  ASSERT_TRUE(keyboard.keyboard->close().ok());
  EXPECT_EQ(harness.device->commits(), 1);
  // Closing releases held keys before the slot goes away.
  ASSERT_TRUE(harness.device->wait_until([&]() {
    const auto reports = harness.slot_inputs(0);
    return !reports.empty() && reports.back() == std::vector<std::uint8_t> {1, 0, 0, 0, 0, 0, 0, 0, 0};
  }));
  ASSERT_TRUE(mouse.mouse->close().ok());
  EXPECT_EQ(harness.device->commits(), 2);
  EXPECT_EQ(harness.device->active().slots[0].kind, LVHB_SLOT_NONE);
}

TEST(Esp32BridgeKeyboardMouseTest, FullBridgeFailsWithoutFallbackAndUsesFallbackWhenPresent) {
  BridgeHarness harness;
  auto first = harness.backend->create_gamepad(1, options_for(lvh::profiles::xbox_360()));
  auto second = harness.backend->create_gamepad(2, options_for(lvh::profiles::dualsense()));
  ASSERT_TRUE(first && second);
  auto keyboard = harness.backend->create_keyboard(3, {});
  EXPECT_FALSE(keyboard);
  EXPECT_EQ(keyboard.status.code(), lvh::ErrorCode::unsupported_profile);

  lvh::RuntimeOptions fake_options;
  fake_options.backend = lvh::BackendKind::fake;
  BridgeHarness with_fallback;
  with_fallback.backend = lvh::detail::esp32::create_esp32_backend(with_fallback.client, lvh::detail::create_backend(fake_options));
  ASSERT_TRUE(with_fallback.backend->create_gamepad(1, options_for(lvh::profiles::xbox_360())));
  ASSERT_TRUE(with_fallback.backend->create_gamepad(2, options_for(lvh::profiles::dualsense())));
  auto fallback_keyboard = with_fallback.backend->create_keyboard(3, {});
  EXPECT_TRUE(fallback_keyboard) << fallback_keyboard.status.message();
  auto fallback_mouse = with_fallback.backend->create_mouse(4, {});
  EXPECT_TRUE(fallback_mouse) << fallback_mouse.status.message();
}

TEST(Esp32BridgeKeyboardMouseTest, UnicodeTextUsesFallbackKeyboard) {
  lvh::RuntimeOptions fake_options;
  fake_options.backend = lvh::BackendKind::fake;
  BridgeHarness harness;
  harness.backend = lvh::detail::esp32::create_esp32_backend(harness.client, lvh::detail::create_backend(fake_options));
  auto keyboard = harness.backend->create_keyboard(1, {});
  ASSERT_TRUE(keyboard) << keyboard.status.message();
  EXPECT_TRUE(keyboard.keyboard->type_text({.text = "\xC3\xA9"}).ok());
  EXPECT_TRUE(harness.slot_inputs(0).empty());
}

TEST(Esp32BridgeKeyboardMouseTest, KeyboardMouseReportsAreNotReplayedAfterReboot) {
  BridgeHarness harness;
  auto mouse = harness.backend->create_mouse(1, {});
  ASSERT_TRUE(mouse) << mouse.status.message();
  ASSERT_TRUE(mouse.mouse->submit({.kind = lvh::MouseEventKind::relative_motion, .x = 5, .y = 5}).ok());
  ASSERT_TRUE(harness.device->wait_until([&]() {
    return harness.slot_inputs(0).size() == 1U;
  }));
  harness.device->reboot();
  ASSERT_TRUE(harness.device->wait_until([&]() {
    return harness.device->commits() >= 2 && harness.device->active().slots[0].kind == LVHB_SLOT_KEYBOARD_MOUSE;
  }));
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(harness.slot_inputs(0).size(), 1U);
}

TEST(Esp32BridgeSelectionTest, PlatformDefaultPicksBridgeFromEnvironmentPortOrDetection) {
  using lvh::detail::esp32::select_bridge_for_platform_default;
  lvh::RuntimeOptions options;
  int probes = 0;
  const auto present = [&probes]() {
    ++probes;
    return true;
  };
  const auto absent = []() {
    return false;
  };
  EXPECT_FALSE(select_bridge_for_platform_default(options, "", "", absent).has_value());
  EXPECT_EQ(select_bridge_for_platform_default(options, "", "", present), std::optional<std::string> {""});
  EXPECT_FALSE(select_bridge_for_platform_default(options, "platform", "COM5", present).has_value());
  EXPECT_EQ(select_bridge_for_platform_default(options, "esp32_bridge", "", absent), std::optional<std::string> {""});
  EXPECT_EQ(select_bridge_for_platform_default(options, "", "COM5", absent), std::optional<std::string> {"COM5"});
  options.esp32_bridge_port = "/dev/ttyACM1";
  EXPECT_EQ(select_bridge_for_platform_default(options, "", "", absent), std::optional<std::string> {"/dev/ttyACM1"});
  EXPECT_EQ(probes, 1);
  options.esp32_bridge_autodetect = false;
  EXPECT_FALSE(select_bridge_for_platform_default(options, "esp32_bridge", "COM5", present).has_value());
}
