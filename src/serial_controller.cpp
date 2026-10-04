#include "auto_aim/serial_controller.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

namespace auto_aim {
namespace {

bool ConfigurePort(int fd) {
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    return false;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  return tcsetattr(fd, TCSANOW, &tty) == 0;
}

std::string SelfTty() {
  const char* paths[] = {
      ttyname(STDIN_FILENO), ttyname(STDOUT_FILENO), ttyname(STDERR_FILENO)};
  for (const char* p : paths) {
    if (p != nullptr && std::strncmp(p, "/dev/pts/", 9) == 0) {
      return p;
    }
  }
  return {};
}

std::string ReadComm(const std::string& pid) {
  std::ifstream in("/proc/" + pid + "/comm");
  std::string comm;
  if (in) {
    std::getline(in, comm);
  }
  return comm;
}

bool IsShellComm(const std::string& comm) {
  return comm == "bash" || comm == "sh" || comm == "zsh" || comm == "dash" ||
         comm == "fish" || comm == "sudo" || comm == "sshd" ||
         comm == "tmux" || comm == "screen" || comm == "node" ||
         comm.find("terminal") != std::string::npos;
}

bool IsGameComm(const std::string& comm) {
  if (comm.empty()) {
    return false;
  }
  return comm.find("homework") != std::string::npos ||
         comm.find("Homework") != std::string::npos ||
         comm.find("godot") != std::string::npos ||
         comm.find("Godot") != std::string::npos ||
         comm.find("x86_64") != std::string::npos;
}

// pts path -> owner processes (pid + comm)
struct PtsOwnerInfo {
  pid_t pid = 0;
  std::string comm;
};

std::map<std::string, std::vector<PtsOwnerInfo>> PtsOwners() {
  std::map<std::string, std::vector<PtsOwnerInfo>> owners;
  DIR* proc = opendir("/proc");
  if (proc == nullptr) {
    return owners;
  }
  while (dirent* ent = readdir(proc)) {
    if (ent->d_name[0] < '0' || ent->d_name[0] > '9') {
      continue;
    }
    const std::string pid = ent->d_name;
    const std::string comm = ReadComm(pid);
    const std::string fd_dir = "/proc/" + pid + "/fd";
    DIR* fds = opendir(fd_dir.c_str());
    if (fds == nullptr) {
      continue;
    }
    while (dirent* fd_ent = readdir(fds)) {
      if (fd_ent->d_name[0] == '.') {
        continue;
      }
      char buf[256];
      const std::string fd_path = fd_dir + "/" + fd_ent->d_name;
      const ssize_t n = readlink(fd_path.c_str(), buf, sizeof(buf) - 1);
      if (n <= 0) {
        continue;
      }
      buf[n] = '\0';
      if (std::strncmp(buf, "/dev/pts/", 9) == 0) {
        PtsOwnerInfo info;
        info.pid = static_cast<pid_t>(std::atoi(pid.c_str()));
        info.comm = comm.empty() ? pid : comm;
        owners[buf].push_back(info);
      }
    }
    closedir(fds);
  }
  closedir(proc);
  return owners;
}

}  // namespace

SerialController::~SerialController() { Close(); }

std::string SerialController::FindGameSerialPath() {
  const std::string self_tty = SelfTty();
  const auto owners = PtsOwners();

  struct Candidate {
    int index = 0;
    int score = 0;
    pid_t newest_game_pid = 0;
    std::string path;
  };
  std::vector<Candidate> candidates;

  DIR* dir = opendir("/dev/pts");
  if (dir == nullptr) {
    return {};
  }
  while (dirent* ent = readdir(dir)) {
    if (ent->d_name[0] < '0' || ent->d_name[0] > '9') {
      continue;
    }
    const std::string path = std::string("/dev/pts/") + ent->d_name;
    if (!self_tty.empty() && path == self_tty) {
      continue;
    }

    const auto it = owners.find(path);
    int score = 0;
    pid_t newest_game_pid = 0;
    if (it == owners.end() || it->second.empty()) {
      score = 2;  // nobody holds it — possible slave waiting for client
    } else {
      bool game = false;
      bool shell = false;
      for (const auto& owner : it->second) {
        if (IsGameComm(owner.comm)) {
          game = true;
          newest_game_pid = std::max(newest_game_pid, owner.pid);
        }
        if (IsShellComm(owner.comm)) {
          shell = true;
        }
      }
      if (game && !shell) {
        score = 3;  // held by homework/Godot — best guess
      } else if (game) {
        score = 3;
      } else if (!shell) {
        score = 1;  // some non-shell process
      } else {
        score = 0;  // interactive shell/terminal — skip
      }
    }

    if (score <= 0) {
      continue;
    }
    candidates.push_back({std::atoi(ent->d_name), score, newest_game_pid, path});
  }
  closedir(dir);

  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) {
              if (a.score != b.score) {
                return a.score > b.score;
              }
              // Several stale game instances may still hold their ptys:
              // trust the one held by the most recently started game.
              if (a.newest_game_pid != b.newest_game_pid) {
                return a.newest_game_pid > b.newest_game_pid;
              }
              return a.index > b.index;
            });

  for (const auto& item : candidates) {
    const int fd =
        ::open(item.path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd >= 0) {
      ::close(fd);
      std::cout << "[Serial] auto-detect " << item.path << " score=" << item.score
                << "\n";
      return item.path;
    }
  }
  return {};
}

bool SerialController::Open(const std::string& device_path) {
  if (device_path == "auto" || device_path.empty()) {
    const std::string found = FindGameSerialPath();
    if (found.empty()) {
      std::cerr << "[Serial] auto-detect found no usable /dev/pts/*\n";
      return false;
    }
    std::cout << "[Serial] auto-detect -> " << found << "\n";
    return OpenPath(found);
  }
  return OpenPath(device_path);
}

bool SerialController::OpenPath(const std::string& device_path) {
  Close();
  fd_ = ::open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    std::cerr << "[Serial] open failed: " << device_path
              << " errno=" << errno << " (" << std::strerror(errno) << ")\n";
    return false;
  }
  if (!ConfigurePort(fd_)) {
    std::cerr << "[Serial] configure failed on " << device_path << "\n";
    Close();
    return false;
  }
  device_path_ = device_path;
  std::cout << "[Serial] opened " << device_path << "\n";
  return true;
}

void SerialController::Close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  device_path_.clear();
}

bool SerialController::IsOpen() const { return fd_ >= 0; }

bool SerialController::OwnedByGame() const {
  if (!IsOpen()) {
    return false;
  }
  const std::string self_tty = SelfTty();
  for (const auto& [path, infos] : PtsOwners()) {
    if (path != self_tty) {
      continue;
    }
    for (const auto& info : infos) {
      if (IsGameComm(info.comm)) {
        return true;
      }
    }
  }
  return false;
}

bool SerialController::SendTurn(float angle_deg) {
  if (!IsOpen()) {
    return false;
  }
  if (!std::isfinite(angle_deg)) {
    return false;
  }
  if (angle_deg > 180.0F) {
    angle_deg = 180.0F;
  }
  if (angle_deg < -180.0F) {
    angle_deg = -180.0F;
  }

  uint8_t packet[5];
  packet[0] = 0x01;
  std::memcpy(packet + 1, &angle_deg, sizeof(float));
  return WriteAll(packet, sizeof(packet));
}

bool SerialController::SendFire() {
  if (!IsOpen()) {
    return false;
  }
  const uint8_t packet = 0x02;
  return WriteAll(&packet, 1);
}

bool SerialController::WriteAll(const uint8_t* data, size_t length) {
  size_t written = 0;
  const auto start = std::chrono::steady_clock::now();
  while (written < length) {
    const ssize_t n = ::write(fd_, data + written, length - written);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // A pty with no reader fills up and writes keep returning EAGAIN:
        // never spin here, or the whole node hangs at 100% CPU.
        if (std::chrono::steady_clock::now() - start >
            std::chrono::milliseconds(50)) {
          return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      return false;
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

}  // namespace auto_aim
