#ifndef AUTO_AIM_SERIAL_CONTROLLER_HPP_
#define AUTO_AIM_SERIAL_CONTROLLER_HPP_

#include <cstdint>
#include <string>

namespace auto_aim {

// Protocol:
//   turn: 0x01 + float32 angle_deg (little-endian, 0 deg = right, [-180, 180])
//   fire: 0x02
class SerialController {
 public:
  SerialController() = default;
  ~SerialController();

  SerialController(const SerialController&) = delete;
  SerialController& operator=(const SerialController&) = delete;

  // device_path: explicit /dev/pts/N, or "auto" to probe unowned pts.
  bool Open(const std::string& device_path);
  void Close();
  bool IsOpen() const;
  // True while some OTHER process still holds our pty open (the game keeps
  // the master side). When the game window closes, the master dies and our
  // own open fd is the only thing keeping the slave node alive — writes go
  // into the void. Detect that and force a re-scan.
  bool OwnedByGame() const;
  const std::string& device_path() const { return device_path_; }

  // Find homework simulator slave pts: unowned /dev/pts/* (game keeps master),
  // excluding this process's controlling tty. Highest index first.
  static std::string FindGameSerialPath();

  bool SendTurn(float angle_deg);
  bool SendFire();

 private:
  bool WriteAll(const uint8_t* data, size_t length);
  bool OpenPath(const std::string& device_path);

  int fd_ = -1;
  std::string device_path_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM_SERIAL_CONTROLLER_HPP_
