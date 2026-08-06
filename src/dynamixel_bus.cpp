// dynamixel_bus.cpp
// Dynamixel SDK SyncRead/SyncWrite 구현
//
// 원본 Python DXLBus 대비 변경점:
//  - read_positions(): SyncRead 실패 시 single-read 1회만 시도 (반복 루프 제거)
//  - 읽기 실패 시 last_pos_cache_ 반환 (원본과 동일한 hold-last 전략)
//  - velocity/current SyncRead 제거 (제어루프에서 position만 사용)
//  - GroupSyncWrite는 매 호출마다 로컬 생성 (txPacket only, 응답 대기 없음)

#include "ft_fb_leaderarm/dynamixel_bus.hpp"

#include <algorithm>
#include <chrono>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <cstdio>
#include <thread>

namespace teleop_cpp {

namespace {

constexpr int kCriticalWriteAttempts = 3;
constexpr auto kCriticalRetryDelay = std::chrono::milliseconds(2);

}  // namespace

static int32_t to_signed32(uint32_t raw) {
  return static_cast<int32_t>(raw);
}

DxlBus::DxlBus(const std::string& device, int baud, double protocol) {
  auto* port = dynamixel::PortHandler::getPortHandler(device.c_str());
  if (!port->openPort()) {
    throw std::runtime_error("DXL port open failed: " + device);
  }
  if (!port->setBaudRate(baud)) {
    throw std::runtime_error("DXL setBaudRate failed: " + std::to_string(baud));
  }
  port_ = port;

  auto* ph = dynamixel::PacketHandler::getPacketHandler(static_cast<float>(protocol));
  ph_ = ph;

  auto* grp = new dynamixel::GroupSyncRead(
    port, ph, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
  grp_pos_ = grp;

  auto* grp_vel_pos = new dynamixel::GroupSyncRead(
    port, ph, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VEL_POS);
  grp_vel_pos_ = grp_vel_pos;
}

DxlBus::~DxlBus() {
  if (grp_pos_) delete static_cast<dynamixel::GroupSyncRead*>(grp_pos_);
  if (grp_vel_pos_) delete static_cast<dynamixel::GroupSyncRead*>(grp_vel_pos_);
  if (port_) {
    static_cast<dynamixel::PortHandler*>(port_)->closePort();
  }
}

void DxlBus::register_ids(const std::array<int, 6>& ids) {
  register_ids(std::vector<int>(ids.begin(), ids.end()));
}

void DxlBus::register_ids(const std::vector<int>& ids) {
  auto* grp = static_cast<dynamixel::GroupSyncRead*>(grp_pos_);
  auto* grp_vel_pos = static_cast<dynamixel::GroupSyncRead*>(grp_vel_pos_);
  grp->clearParam();
  grp_vel_pos->clearParam();
  for (int id : ids) {
    if (!grp->addParam(id)) {
      throw std::runtime_error("GroupSyncRead addParam failed for id=" + std::to_string(id));
    }
    if (!grp_vel_pos->addParam(id)) {
      throw std::runtime_error("GroupSyncRead vel+pos addParam failed for id=" + std::to_string(id));
    }
  }
  registered_ids_ = ids;
  ids_registered_ = true;
}

std::string DxlBus::format_txrx_failure(
    int comm_result, uint8_t packet_error, const char* operation, int id) const {
  auto* ph = static_cast<dynamixel::PacketHandler*>(ph_);
  std::ostringstream out;
  out << operation << " failed for id=" << id;
  if (comm_result != COMM_SUCCESS) {
    out << ": " << ph->getTxRxResult(comm_result);
  }
  if (packet_error != 0) {
    out << (comm_result == COMM_SUCCESS ? ": " : "; ")
        << ph->getRxPacketError(packet_error);
  }
  return out.str();
}

bool DxlBus::record_txrx_result(
    int comm_result, uint8_t packet_error, const char* operation, int id) {
  if (comm_result == COMM_SUCCESS && packet_error == 0) return true;
  last_error_ = format_txrx_failure(comm_result, packet_error, operation, id);
  return false;
}

bool DxlBus::write1_byte_verified(
    int id, uint16_t address, uint8_t expected, const char* operation) {
  auto* port = static_cast<dynamixel::PortHandler*>(port_);
  auto* ph = static_cast<dynamixel::PacketHandler*>(ph_);
  std::string final_detail;

  for (int attempt = 1; attempt <= kCriticalWriteAttempts; ++attempt) {
    clear_port();
    uint8_t write_error = 0;
    const int write_result = ph->write1ByteTxRx(
      port, id, address, expected, &write_error);
    const bool write_ack = write_result == COMM_SUCCESS && write_error == 0;
    const bool write_packet_clean = write_error == 0;
    const std::string write_detail = write_ack ? std::string() :
      format_txrx_failure(write_result, write_error, operation, id);

    // A write can reach the motor even when its communication ACK is lost.
    // A clean matching readback may recover that case, but never overrides an
    // explicit error reported by the motor itself.
    clear_port();
    uint8_t actual = 0;
    uint8_t read_error = 0;
    const int read_result = ph->read1ByteTxRx(
      port, id, address, &actual, &read_error);
    const bool read_ok = read_result == COMM_SUCCESS && read_error == 0;
    if (read_ok && actual == expected && write_packet_clean) {
      if (!write_ack || attempt > 1) {
        std::fprintf(stderr,
          "[DXL RECOVERY] %s id=%d verified by register readback "
          "on attempt %d/%d (write_ack=%s)\n",
          operation, id, attempt, kCriticalWriteAttempts,
          write_ack ? "true" : "false");
      }
      last_error_.clear();
      return true;
    }

    std::ostringstream detail;
    if (!write_detail.empty()) detail << write_detail << "; ";
    if (!read_ok) {
      detail << format_txrx_failure(
        read_result, read_error, "register readback", id);
    } else if (!write_packet_clean) {
      detail << operation << " motor packet error prevents readback recovery"
             << " for id=" << id;
    } else {
      detail << operation << " readback mismatch for id=" << id
             << ": expected=" << static_cast<unsigned int>(expected)
             << ", actual=" << static_cast<unsigned int>(actual);
    }
    final_detail = detail.str();
    if (attempt < kCriticalWriteAttempts) {
      std::this_thread::sleep_for(kCriticalRetryDelay);
    }
  }

  std::ostringstream out;
  out << operation << " failed after " << kCriticalWriteAttempts
      << " attempts for id=" << id << "; " << final_detail;
  last_error_ = out.str();
  return false;
}

bool DxlBus::write2_byte_verified(
    int id, uint16_t address, uint16_t expected, const char* operation) {
  auto* port = static_cast<dynamixel::PortHandler*>(port_);
  auto* ph = static_cast<dynamixel::PacketHandler*>(ph_);
  std::string final_detail;

  for (int attempt = 1; attempt <= kCriticalWriteAttempts; ++attempt) {
    clear_port();
    uint8_t write_error = 0;
    const int write_result = ph->write2ByteTxRx(
      port, id, address, expected, &write_error);
    const bool write_ack = write_result == COMM_SUCCESS && write_error == 0;
    const bool write_packet_clean = write_error == 0;
    const std::string write_detail = write_ack ? std::string() :
      format_txrx_failure(write_result, write_error, operation, id);

    clear_port();
    uint16_t actual = 0;
    uint8_t read_error = 0;
    const int read_result = ph->read2ByteTxRx(
      port, id, address, &actual, &read_error);
    const bool read_ok = read_result == COMM_SUCCESS && read_error == 0;
    if (read_ok && actual == expected && write_packet_clean) {
      if (!write_ack || attempt > 1) {
        std::fprintf(stderr,
          "[DXL RECOVERY] %s id=%d verified by register readback "
          "on attempt %d/%d (write_ack=%s)\n",
          operation, id, attempt, kCriticalWriteAttempts,
          write_ack ? "true" : "false");
      }
      last_error_.clear();
      return true;
    }

    std::ostringstream detail;
    if (!write_detail.empty()) detail << write_detail << "; ";
    if (!read_ok) {
      detail << format_txrx_failure(
        read_result, read_error, "register readback", id);
    } else if (!write_packet_clean) {
      detail << operation << " motor packet error prevents readback recovery"
             << " for id=" << id;
    } else {
      detail << operation << " readback mismatch for id=" << id
             << ": expected=" << expected << ", actual=" << actual;
    }
    final_detail = detail.str();
    if (attempt < kCriticalWriteAttempts) {
      std::this_thread::sleep_for(kCriticalRetryDelay);
    }
  }

  std::ostringstream out;
  out << operation << " failed after " << kCriticalWriteAttempts
      << " attempts for id=" << id << "; " << final_detail;
  last_error_ = out.str();
  return false;
}

bool DxlBus::write4_byte_verified(
    int id, uint16_t address, uint32_t expected, const char* operation) {
  auto* port = static_cast<dynamixel::PortHandler*>(port_);
  auto* ph = static_cast<dynamixel::PacketHandler*>(ph_);
  std::string final_detail;

  for (int attempt = 1; attempt <= kCriticalWriteAttempts; ++attempt) {
    clear_port();
    uint8_t write_error = 0;
    const int write_result = ph->write4ByteTxRx(
      port, id, address, expected, &write_error);
    const bool write_ack = write_result == COMM_SUCCESS && write_error == 0;
    const bool write_packet_clean = write_error == 0;
    const std::string write_detail = write_ack ? std::string() :
      format_txrx_failure(write_result, write_error, operation, id);

    clear_port();
    uint32_t actual = 0;
    uint8_t read_error = 0;
    const int read_result = ph->read4ByteTxRx(
      port, id, address, &actual, &read_error);
    const bool read_ok = read_result == COMM_SUCCESS && read_error == 0;
    if (read_ok && actual == expected && write_packet_clean) {
      if (!write_ack || attempt > 1) {
        std::fprintf(stderr,
          "[DXL RECOVERY] %s id=%d verified by register readback "
          "on attempt %d/%d (write_ack=%s)\n",
          operation, id, attempt, kCriticalWriteAttempts,
          write_ack ? "true" : "false");
      }
      last_error_.clear();
      return true;
    }

    std::ostringstream detail;
    if (!write_detail.empty()) detail << write_detail << "; ";
    if (!read_ok) {
      detail << format_txrx_failure(
        read_result, read_error, "register readback", id);
    } else if (!write_packet_clean) {
      detail << operation << " motor packet error prevents readback recovery"
             << " for id=" << id;
    } else {
      detail << operation << " readback mismatch for id=" << id
             << ": expected=" << expected << ", actual=" << actual;
    }
    final_detail = detail.str();
    if (attempt < kCriticalWriteAttempts) {
      std::this_thread::sleep_for(kCriticalRetryDelay);
    }
  }

  std::ostringstream out;
  out << operation << " failed after " << kCriticalWriteAttempts
      << " attempts for id=" << id << "; " << final_detail;
  last_error_ = out.str();
  return false;
}

bool DxlBus::force_status_return(
    const std::array<int, 6>& ids, uint8_t level, uint8_t delay) {
  return force_status_return(std::vector<int>(ids.begin(), ids.end()), level, delay);
}

bool DxlBus::force_status_return(
    const std::vector<int>& ids, uint8_t level, uint8_t delay) {
  last_error_.clear();
  std::vector<std::string> failures;
  for (int id : ids) {
    if (!write1_byte_verified(
          id, ADDR_STATUS_RETURN_LEVEL, level, "set status return level")) {
      failures.push_back(last_error_);
    }
    if (!write1_byte_verified(
          id, ADDR_RETURN_DELAY_TIME, delay, "set return delay")) {
      failures.push_back(last_error_);
    }
  }
  if (failures.empty()) {
    last_error_.clear();
    return true;
  }
  std::ostringstream out;
  for (std::size_t i = 0; i < failures.size(); ++i) {
    if (i) out << " | ";
    out << failures[i];
  }
  last_error_ = out.str();
  return false;
}

bool DxlBus::torque_on(const std::array<int, 6>& ids) {
  return torque_on(std::vector<int>(ids.begin(), ids.end()));
}

bool DxlBus::torque_on(const std::vector<int>& ids) {
  last_error_.clear();
  for (int id : ids) {
    if (write1_byte_verified(id, ADDR_TORQUE_ENABLE, 1, "torque on")) {
      continue;
    }
    const std::string primary_error = last_error_;
    const bool rollback_ok = torque_off(ids);
    const std::string rollback_error = last_error_;
    last_error_ = primary_error + "; rollback torque-off=" +
      (rollback_ok ? std::string("verified") :
       std::string("FAILED: ") + rollback_error);
    return false;
  }
  last_error_.clear();
  return true;
}

bool DxlBus::torque_off(const std::array<int, 6>& ids) {
  return torque_off(std::vector<int>(ids.begin(), ids.end()));
}

bool DxlBus::torque_off(const std::vector<int>& ids) {
  last_error_.clear();
  std::vector<std::string> failures;
  for (int id : ids) {
    if (!write1_byte_verified(id, ADDR_TORQUE_ENABLE, 0, "torque off")) {
      failures.push_back(last_error_);
    }
  }
  if (failures.empty()) {
    last_error_.clear();
    return true;
  }
  std::ostringstream out;
  for (std::size_t i = 0; i < failures.size(); ++i) {
    if (i) out << " | ";
    out << failures[i];
  }
  last_error_ = out.str();
  return false;
}

bool DxlBus::prepare_operating_mode(
    const std::array<int, 6>& ids, uint8_t mode) {
  return prepare_operating_mode(std::vector<int>(ids.begin(), ids.end()), mode);
}

bool DxlBus::prepare_operating_mode(
    const std::vector<int>& ids, uint8_t mode) {
  last_error_.clear();
  // Phase barrier 1: every motor must be verifiably torque-off before any
  // operating mode register changes.  Transactions remain ID-by-ID unicast.
  if (!torque_off(ids)) return false;

  // Phase barrier 2: change and read back every mode while all motors remain
  // torque-off.  A failure rolls the full set back to verified torque-off.
  for (int id : ids) {
    if (!write1_byte_verified(id, ADDR_OPERATING_MODE, mode, "set operating mode")) {
      const std::string primary_error = last_error_;
      const bool rollback_ok = torque_off(ids);
      const std::string rollback_error = last_error_;
      last_error_ = primary_error + "; rollback torque-off=" +
        (rollback_ok ? std::string("verified") :
         std::string("FAILED: ") + rollback_error);
      return false;
    }
  }
  last_error_.clear();
  return true;
}

bool DxlBus::set_operating_mode(const std::array<int, 6>& ids, uint8_t mode) {
  return set_operating_mode(std::vector<int>(ids.begin(), ids.end()), mode);
}

bool DxlBus::set_operating_mode(const std::vector<int>& ids, uint8_t mode) {
  if (!prepare_operating_mode(ids, mode)) return false;
  return torque_on(ids);
}

bool DxlBus::set_profile(
    const std::array<int, 6>& ids, uint32_t vel_ticks, uint32_t acc_ticks) {
  return set_profile(std::vector<int>(ids.begin(), ids.end()), vel_ticks, acc_ticks);
}

bool DxlBus::set_profile(
    const std::vector<int>& ids, uint32_t vel_ticks, uint32_t acc_ticks) {
  last_error_.clear();
  std::vector<std::string> failures;
  for (int id : ids) {
    if (!write4_byte_verified(
          id, ADDR_PROFILE_VELOCITY, vel_ticks, "set profile velocity")) {
      failures.push_back(last_error_);
    }
    if (!write4_byte_verified(
          id, ADDR_PROFILE_ACCELERATION, acc_ticks,
          "set profile acceleration")) {
      failures.push_back(last_error_);
    }
  }
  if (failures.empty()) {
    last_error_.clear();
    return true;
  }
  std::ostringstream out;
  for (std::size_t i = 0; i < failures.size(); ++i) {
    if (i) out << " | ";
    out << failures[i];
  }
  last_error_ = out.str();
  return false;
}

bool DxlBus::set_profile_deg(
    const std::array<int, 6>& ids, double vel_deg_s, double acc_deg_s2) {
  return set_profile_deg(
    std::vector<int>(ids.begin(), ids.end()), vel_deg_s, acc_deg_s2);
}

bool DxlBus::set_profile_deg(
    const std::vector<int>& ids, double vel_deg_s, double acc_deg_s2) {
  const double rpm = (vel_deg_s / 360.0) * 60.0;
  const auto vel_unit = static_cast<uint32_t>(
    std::max(1.0, std::round(rpm / 0.229)));
  const double rev_per_min2 = (acc_deg_s2 / 360.0) * 3600.0;
  const auto acc_unit = static_cast<uint32_t>(
    std::max(1.0, std::round(rev_per_min2 / 214.577)));
  return set_profile(ids, vel_unit, acc_unit);
}

void DxlBus::clear_port() {
  auto* port = static_cast<dynamixel::PortHandler*>(port_);
  port->clearPort();
}

std::map<int, int32_t> DxlBus::read_positions() {
  std::map<int, int32_t> out;
  if (!ids_registered_) return out;
  last_error_.clear();
  last_missing_count_ = 0;
  last_fallback_count_ = 0;
  last_read_all_fresh_ = false;
  last_velocity_missing_count_ = 0;
  last_velocity_fallback_count_ = 0;
  last_velocity_read_all_fresh_ = false;

  auto* port = static_cast<dynamixel::PortHandler*>(port_);
  auto* ph = static_cast<dynamixel::PacketHandler*>(ph_);
  auto* grp = static_cast<dynamixel::GroupSyncRead*>(grp_pos_);

  clear_port();
  int comm = grp->txRxPacket();
  bool ok = (comm == COMM_SUCCESS);
  if (!ok) {
    auto* packet = static_cast<dynamixel::PacketHandler*>(ph_);
    last_error_ = std::string("position SyncRead failed: ") + packet->getTxRxResult(comm);
  }

  for (int id : registered_ids_) {
    int32_t val;
    bool got = false;

    if (ok && grp->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
      uint32_t raw = grp->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      val = to_signed32(raw);
      got = true;
    }

    if (!got) {
      // Single-read fallback (no retry loop — one shot only)
      clear_port();
      uint32_t raw = 0;
      uint8_t err = 0;
      int res = ph->read4ByteTxRx(port, id, ADDR_PRESENT_POSITION, &raw, &err);
      if (res == COMM_SUCCESS && err == 0) {
        val = to_signed32(raw);
        got = true;
        ++last_fallback_count_;
      } else {
        record_txrx_result(res, err, "fallback position read", id);
      }
    }

    if (got) {
      out[id] = val;
      last_pos_cache_[id] = val;
    } else if (last_pos_cache_.count(id)) {
      out[id] = last_pos_cache_[id];
      ++last_missing_count_;
    } else {
      ++last_missing_count_;
    }
  }
  last_read_all_fresh_ = (last_missing_count_ == 0 && static_cast<int>(out.size()) == static_cast<int>(registered_ids_.size()));
  return out;
}

std::map<int, DxlState> DxlBus::read_position_velocity() {
  std::map<int, DxlState> out;
  if (!ids_registered_) return out;
  last_error_.clear();
  last_missing_count_ = 0;
  last_fallback_count_ = 0;
  last_read_all_fresh_ = false;
  last_velocity_missing_count_ = 0;
  last_velocity_fallback_count_ = 0;
  last_velocity_read_all_fresh_ = false;

  auto* port = static_cast<dynamixel::PortHandler*>(port_);
  auto* ph = static_cast<dynamixel::PacketHandler*>(ph_);
  auto* grp = static_cast<dynamixel::GroupSyncRead*>(grp_vel_pos_);

  clear_port();
  int comm = grp->txRxPacket();
  bool ok = (comm == COMM_SUCCESS);
  if (!ok) {
    auto* packet = static_cast<dynamixel::PacketHandler*>(ph_);
    last_error_ = std::string("velocity+position SyncRead failed: ") +
      packet->getTxRxResult(comm);
  }

  for (int id : registered_ids_) {
    DxlState state;
    bool pos_got = false;
    bool vel_got = false;

    if (ok && grp->isAvailable(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)) {
      uint32_t raw = grp->getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
      state.velocity = to_signed32(raw);
      vel_got = true;
    }
    if (ok && grp->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
      uint32_t raw = grp->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      state.position = to_signed32(raw);
      pos_got = true;
    }

    if (!vel_got) {
      clear_port();
      uint32_t raw = 0;
      uint8_t err = 0;
      int res = ph->read4ByteTxRx(port, id, ADDR_PRESENT_VELOCITY, &raw, &err);
      if (res == COMM_SUCCESS && err == 0) {
        state.velocity = to_signed32(raw);
        vel_got = true;
        ++last_velocity_fallback_count_;
      } else {
        record_txrx_result(res, err, "fallback velocity read", id);
      }
    }

    if (!pos_got) {
      clear_port();
      uint32_t raw = 0;
      uint8_t err = 0;
      int res = ph->read4ByteTxRx(port, id, ADDR_PRESENT_POSITION, &raw, &err);
      if (res == COMM_SUCCESS && err == 0) {
        state.position = to_signed32(raw);
        pos_got = true;
        ++last_fallback_count_;
      } else {
        record_txrx_result(res, err, "fallback position read", id);
      }
    }

    state.position_fresh = pos_got;
    state.velocity_fresh = vel_got;

    bool has_position = false;
    if (pos_got) {
      last_pos_cache_[id] = state.position;
      has_position = true;
    } else if (last_pos_cache_.count(id)) {
      state.position = last_pos_cache_[id];
      ++last_missing_count_;
      has_position = true;
    } else {
      ++last_missing_count_;
    }

    if (vel_got) {
      last_vel_cache_[id] = state.velocity;
    } else if (last_vel_cache_.count(id)) {
      state.velocity = last_vel_cache_[id];
      ++last_velocity_missing_count_;
    } else {
      ++last_velocity_missing_count_;
    }

    if (has_position) {
      out[id] = state;
    }
  }

  last_read_all_fresh_ =
    (last_missing_count_ == 0 && static_cast<int>(out.size()) == static_cast<int>(registered_ids_.size()));
  last_velocity_read_all_fresh_ =
    (last_velocity_missing_count_ == 0 && static_cast<int>(out.size()) == static_cast<int>(registered_ids_.size()));
  return out;
}

bool DxlBus::write_goal_currents(const std::map<int, int16_t>& units) {
  auto* port = static_cast<dynamixel::PortHandler*>(port_);
  auto* ph = static_cast<dynamixel::PacketHandler*>(ph_);

  last_error_.clear();
  dynamixel::GroupSyncWrite writer(port, ph, ADDR_GOAL_CURRENT, 2);
  for (auto& [id, val] : units) {
    uint8_t data[2];
    data[0] = DXL_LOBYTE(static_cast<uint16_t>(val));
    data[1] = DXL_HIBYTE(static_cast<uint16_t>(val));
    if (!writer.addParam(id, data)) {
      last_error_ = "goal current addParam failed for id=" + std::to_string(id);
      return false;
    }
  }
  const int result = writer.txPacket();
  if (result != COMM_SUCCESS) {
    last_error_ = std::string("goal current SyncWrite failed: ") + ph->getTxRxResult(result);
    return false;
  }
  return true;
}

bool DxlBus::write_goal_currents_verified(
    const std::map<int, int16_t>& units) {
  last_error_.clear();
  for (const auto& [id, value] : units) {
    const auto raw = static_cast<uint16_t>(value);
    if (!write2_byte_verified(
          id, ADDR_GOAL_CURRENT, raw, "set safe goal current")) {
      return false;
    }
  }
  last_error_.clear();
  return true;
}

bool DxlBus::write_goal_positions(const std::map<int, int32_t>& ticks) {
  auto* port = static_cast<dynamixel::PortHandler*>(port_);
  auto* ph = static_cast<dynamixel::PacketHandler*>(ph_);

  last_error_.clear();
  dynamixel::GroupSyncWrite writer(port, ph, ADDR_GOAL_POSITION, 4);
  for (auto& [id, val] : ticks) {
    const int32_t clipped = std::clamp<int32_t>(val, 0, 4095);
    uint8_t data[4];
    data[0] = DXL_LOBYTE(DXL_LOWORD(static_cast<uint32_t>(clipped)));
    data[1] = DXL_HIBYTE(DXL_LOWORD(static_cast<uint32_t>(clipped)));
    data[2] = DXL_LOBYTE(DXL_HIWORD(static_cast<uint32_t>(clipped)));
    data[3] = DXL_HIBYTE(DXL_HIWORD(static_cast<uint32_t>(clipped)));
    if (!writer.addParam(id, data)) {
      last_error_ = "goal position addParam failed for id=" + std::to_string(id);
      return false;
    }
  }
  const int result = writer.txPacket();
  if (result != COMM_SUCCESS) {
    last_error_ = std::string("goal position SyncWrite failed: ") + ph->getTxRxResult(result);
    return false;
  }
  return true;
}

bool DxlBus::write_goal_positions_verified(
    const std::map<int, int32_t>& ticks) {
  last_error_.clear();
  for (const auto& [id, value] : ticks) {
    const int32_t clipped = std::clamp<int32_t>(value, 0, 4095);
    if (!write4_byte_verified(
          id, ADDR_GOAL_POSITION, static_cast<uint32_t>(clipped),
          "set safe goal position")) {
      return false;
    }
  }
  last_error_.clear();
  return true;
}

bool DxlBus::ping_all(const std::array<int, 6>& ids) {
  return ping_all(std::vector<int>(ids.begin(), ids.end()));
}

bool DxlBus::ping_all(const std::vector<int>& ids) {
  auto* port = static_cast<dynamixel::PortHandler*>(port_);
  auto* ph = static_cast<dynamixel::PacketHandler*>(ph_);
  last_error_.clear();
  for (int id : ids) {
    bool ping_ok = false;
    uint16_t model = 0;
    std::string final_detail;
    for (int attempt = 1; attempt <= kCriticalWriteAttempts; ++attempt) {
      clear_port();
      uint8_t err = 0;
      const int res = ph->ping(port, id, &model, &err);
      if (res == COMM_SUCCESS && err == 0) {
        ping_ok = true;
        if (attempt > 1) {
          std::fprintf(stderr,
            "[DXL RECOVERY] ping id=%d recovered on attempt %d/%d\n",
            id, attempt, kCriticalWriteAttempts);
        }
        break;
      }
      final_detail = format_txrx_failure(res, err, "ping", id);
      if (attempt < kCriticalWriteAttempts) {
        std::this_thread::sleep_for(kCriticalRetryDelay);
      }
    }
    if (!ping_ok) {
      last_error_ = final_detail;
      std::fprintf(stderr, "[DXL] %s\n", last_error_.c_str());
      return false;
    }
    std::printf("[DXL] id=%d model=%u OK\n", id, model);
  }
  last_error_.clear();
  return true;
}

}  // namespace teleop_cpp
