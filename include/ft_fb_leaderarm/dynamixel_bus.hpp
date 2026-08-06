// dynamixel_bus.hpp
// Dynamixel SDK C++ 래퍼 (원본 Python DXLBus 클래스 대응)
//
// 원본 대비 변경점:
//  - SyncRead position 전용 (velocity/current SyncRead 제거 — 제어루프에서 미사용)
//  - fallback single-read 최대 1회만 시도 (원본: 미싱 ID마다 반복 → 최대 12ms 추가)
//  - void* 핸들로 DXL SDK 타입을 헤더에서 숨김 (include 의존성 최소화)

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace teleop_cpp {

// Dynamixel XM/XH series control table addresses
constexpr uint16_t ADDR_OPERATING_MODE    = 11;
constexpr uint16_t ADDR_TORQUE_ENABLE     = 64;
constexpr uint16_t ADDR_GOAL_CURRENT      = 102;
constexpr uint16_t ADDR_GOAL_POSITION     = 116;
constexpr uint16_t ADDR_PRESENT_CURRENT   = 126;
constexpr uint16_t ADDR_PRESENT_VELOCITY  = 128;
constexpr uint16_t ADDR_PRESENT_POSITION  = 132;

constexpr uint16_t LEN_PRESENT_CURRENT    = 2;
constexpr uint16_t LEN_PRESENT_VELOCITY   = 4;
constexpr uint16_t LEN_PRESENT_POSITION   = 4;
constexpr uint16_t LEN_PRESENT_VEL_POS    = 8;  // velocity(128..131) + position(132..135)

// Profile velocity / acceleration for position mode
constexpr uint16_t ADDR_PROFILE_VELOCITY      = 112;
constexpr uint16_t ADDR_PROFILE_ACCELERATION  = 108;
constexpr uint16_t ADDR_STATUS_RETURN_LEVEL   = 68;
constexpr uint16_t ADDR_RETURN_DELAY_TIME     = 9;

// Conversion constants
constexpr double TICKS_TO_RAD = 2.0 * M_PI / 4096.0;
constexpr double RAD_TO_TICKS = 4096.0 / (2.0 * M_PI);
constexpr double DXL_VELOCITY_UNIT_RAD_S = 0.229 * 2.0 * M_PI / 60.0;

struct DxlState {
  int32_t position{0};
  int32_t velocity{0};
  bool position_fresh{false};
  bool velocity_fresh{false};
};

class DxlBus {
public:
  DxlBus(const std::string& device, int baud, double protocol = 2.0);
  ~DxlBus();

  // Register motor IDs for SyncRead
  void register_ids(const std::array<int, 6>& ids);
  void register_ids(const std::vector<int>& ids);

  // Torque on/off
  bool torque_on(const std::array<int, 6>& ids);
  bool torque_off(const std::array<int, 6>& ids);
  bool torque_on(const std::vector<int>& ids);
  bool torque_off(const std::vector<int>& ids);

  // Operating mode: 0=current, 3=position, 5=current-based position
  // prepare_operating_mode() verifies every ID is torque-off, then changes and
  // reads back every mode while leaving all motors torque-off.  Callers may
  // preload a safe goal before torque_on().
  bool prepare_operating_mode(const std::array<int, 6>& ids, uint8_t mode);
  bool prepare_operating_mode(const std::vector<int>& ids, uint8_t mode);

  // Compatibility helper: phased prepare_operating_mode() followed by a
  // verified ID-by-ID torque_on().
  bool set_operating_mode(const std::array<int, 6>& ids, uint8_t mode);
  bool set_operating_mode(const std::vector<int>& ids, uint8_t mode);

  // Profile velocity/acceleration for position mode
  bool set_profile(const std::array<int, 6>& ids, uint32_t vel_ticks, uint32_t acc_ticks);
  bool set_profile(const std::vector<int>& ids, uint32_t vel_ticks, uint32_t acc_ticks);
  bool set_profile_deg(const std::array<int, 6>& ids, double vel_deg_s, double acc_deg_s2);
  bool set_profile_deg(const std::vector<int>& ids, double vel_deg_s, double acc_deg_s2);

  // Force status return level and return delay
  bool force_status_return(const std::array<int, 6>& ids, uint8_t level = 2, uint8_t delay = 0);
  bool force_status_return(const std::vector<int>& ids, uint8_t level = 2, uint8_t delay = 0);

  // SyncRead positions (ticks) — returns map {id: signed_ticks}
  std::map<int, int32_t> read_positions();
  // SyncRead present velocity + position in one packet — returns map {id: state}
  std::map<int, DxlState> read_position_velocity();
  int last_missing_count() const { return last_missing_count_; }
  int last_fallback_count() const { return last_fallback_count_; }
  bool last_read_all_fresh() const { return last_read_all_fresh_; }
  int last_velocity_missing_count() const { return last_velocity_missing_count_; }
  int last_velocity_fallback_count() const { return last_velocity_fallback_count_; }
  bool last_velocity_read_all_fresh() const { return last_velocity_read_all_fresh_; }

  // SyncWrite goal currents
  bool write_goal_currents(const std::map<int, int16_t>& units);
  // Non-real-time transition helper: ID-by-ID write with bounded retry and
  // register readback.  Do not use this in the 500 Hz control loop.
  bool write_goal_currents_verified(const std::map<int, int16_t>& units);

  // SyncWrite goal positions (ticks)
  bool write_goal_positions(const std::map<int, int32_t>& ticks);
  // Non-real-time transition helper matching write_goal_currents_verified().
  bool write_goal_positions_verified(const std::map<int, int32_t>& ticks);

  // Ping IDs and verify connectivity
  bool ping_all(const std::array<int, 6>& ids);
  bool ping_all(const std::vector<int>& ids);
  const std::string& last_error() const { return last_error_; }

private:
  void clear_port();
  std::string format_txrx_failure(
      int comm_result, uint8_t packet_error,
      const char* operation, int id) const;
  bool record_txrx_result(int comm_result, uint8_t packet_error,
                          const char* operation, int id);
  bool write1_byte_verified(
      int id, uint16_t address, uint8_t expected, const char* operation);
  bool write2_byte_verified(
      int id, uint16_t address, uint16_t expected, const char* operation);
  bool write4_byte_verified(
      int id, uint16_t address, uint32_t expected, const char* operation);

  // Opaque Dynamixel SDK handles
  void* port_{nullptr};
  void* ph_{nullptr};
  void* grp_pos_{nullptr};
  void* grp_vel_pos_{nullptr};
  bool ids_registered_{false};
  std::vector<int> registered_ids_;
  std::map<int, int32_t> last_pos_cache_;
  std::map<int, int32_t> last_vel_cache_;
  int last_missing_count_{0};
  int last_fallback_count_{0};
  bool last_read_all_fresh_{false};
  int last_velocity_missing_count_{0};
  int last_velocity_fallback_count_{0};
  bool last_velocity_read_all_fresh_{false};
  std::string last_error_;
};

}  // namespace teleop_cpp
