/**
 * @file src/platform/esp32/credential_store.cpp
 * @brief Storage of ESP32-S3 bridge pairing secrets on the host.
 */

// standard includes
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <span>
#include <system_error>
#include <utility>

// local includes
#include "platform/esp32/credential_store.hpp"

#ifdef _WIN32
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <windows.h>
  // clang-format off
  #include <sddl.h>
  // clang-format on
#else
  #include <fcntl.h>
  #include <unistd.h>
#endif

namespace lvh::detail::esp32 {
  namespace {

    std::string to_hex(std::span<const std::uint8_t> bytes) {
      static constexpr char digits[] = "0123456789abcdef";
      std::string text;
      for (const auto byte : bytes) {
        text.push_back(digits[byte >> 4U]);
        text.push_back(digits[byte & 0x0FU]);
      }
      return text;
    }

    template<std::size_t N>
    bool from_hex(const std::string &text, std::array<std::uint8_t, N> &bytes) {
      if (text.size() != N * 2U) {
        return false;
      }
      const auto nibble = [](char c) -> int {
        if (c >= '0' && c <= '9') {
          return c - '0';
        }
        if (c >= 'a' && c <= 'f') {
          return c - 'a' + 10;
        }
        if (c >= 'A' && c <= 'F') {
          return c - 'A' + 10;
        }
        return -1;
      };
      for (std::size_t index = 0; index < N; ++index) {
        const auto high = nibble(text[index * 2U]);
        const auto low = nibble(text[index * 2U + 1U]);
        if (high < 0 || low < 0) {
          return false;
        }
        bytes[index] = static_cast<std::uint8_t>((high << 4) | low);
      }
      return true;
    }

    bool write_private_file(const std::filesystem::path &path, const std::string &contents) {
#ifdef _WIN32
      // An existing file keeps its DACL on overwrite, so always create it anew.
      ::DeleteFileW(path.c_str());
      PSECURITY_DESCRIPTOR descriptor = nullptr;
      if (!::ConvertStringSecurityDescriptorToSecurityDescriptorW(L"D:P(A;;FA;;;SY)(A;;FA;;;BA)(A;;FA;;;OW)", SDDL_REVISION_1, &descriptor, nullptr)) {
        return false;
      }
      SECURITY_ATTRIBUTES attributes {sizeof(attributes), descriptor, FALSE};
      const HANDLE handle = ::CreateFileW(path.c_str(), GENERIC_WRITE, 0, &attributes, CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr);
      ::LocalFree(descriptor);
      if (handle == INVALID_HANDLE_VALUE) {
        return false;
      }
      DWORD written = 0;
      const auto ok = ::WriteFile(handle, contents.data(), static_cast<DWORD>(contents.size()), &written, nullptr) && written == contents.size();
      ::CloseHandle(handle);
      return ok;
#else
      const auto temporary = path.string() + ".tmp";
      const int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
      if (fd < 0) {
        return false;
      }
      const auto written = ::write(fd, contents.data(), contents.size());
      const auto ok = written == static_cast<ssize_t>(contents.size()) && ::fsync(fd) == 0;
      ::close(fd);
      if (!ok) {
        ::unlink(temporary.c_str());
        return false;
      }
      return std::rename(temporary.c_str(), path.c_str()) == 0;
#endif
    }

    class FileCredentialStore final: public CredentialStore {
    public:
      explicit FileCredentialStore(std::string directory):
          directory_ {std::move(directory)} {}

      std::optional<BridgeCredential> load(const std::string &device_id) override {
        std::ifstream file {path_for(device_id)};
        std::string key_id;
        std::string secret;
        BridgeCredential credential;
        if (!(file >> key_id >> secret) || !from_hex(key_id, credential.key_id) || !from_hex(secret, credential.secret)) {
          return std::nullopt;
        }
        return credential;
      }

      bool save(const std::string &device_id, const BridgeCredential &credential) override {
        std::error_code error;
        std::filesystem::create_directories(directory_, error);
        if (error) {
          return false;
        }
        return write_private_file(path_for(device_id), to_hex(credential.key_id) + " " + to_hex(credential.secret) + "\n");
      }

      void erase(const std::string &device_id) override {
        std::error_code error;
        std::filesystem::remove(path_for(device_id), error);
      }

    private:
      std::filesystem::path path_for(const std::string &device_id) const {
        return std::filesystem::path {directory_} / ("bridge-" + device_id + ".key");
      }

      std::filesystem::path directory_;
    };

    std::string environment(const char *name) {
#ifdef _WIN32
      char *value = nullptr;
      std::size_t size = 0;
      if (_dupenv_s(&value, &size, name) != 0 || value == nullptr) {
        return {};
      }
      std::string result {value};
      std::free(value);
      return result;
#else
      const char *value = std::getenv(name);
      return value != nullptr ? std::string {value} : std::string {};
#endif
    }

  }  // namespace

  std::string default_credential_directory() {
#ifdef _WIN32
    const auto base = environment("ProgramData");
    return base.empty() ? std::string {} : (std::filesystem::path {base} / "libvirtualhid" / "esp32-bridge").string();
#else
    if (const auto xdg = environment("XDG_CONFIG_HOME"); !xdg.empty()) {
      return (std::filesystem::path {xdg} / "libvirtualhid" / "esp32-bridge").string();
    }
    const auto home = environment("HOME");
    return home.empty() ? std::string {} : (std::filesystem::path {home} / ".config" / "libvirtualhid" / "esp32-bridge").string();
#endif
  }

  std::shared_ptr<CredentialStore> make_file_credential_store(std::string directory) {
    if (directory.empty()) {
      directory = default_credential_directory();
    }
    return std::make_shared<FileCredentialStore>(std::move(directory));
  }

}  // namespace lvh::detail::esp32
