#include "robotiq_2f85_driver/robotiq_2f85_driver.h"

#include <algorithm>
#include <chrono>
#include <thread>
#include <cerrno>
#include <cmath>

#include <ros/ros.h>

namespace robotiq_2f85
{

// ── Modbus register addresses (from comModbusRtu.py) ─────────────────────────
static constexpr int     WRITE_ADDR = 0x03E8;  // 1000 dec — output registers
static constexpr int     READ_ADDR  = 0x07D0;  // 2000 dec — input  registers
static constexpr int     NUM_REGS   = 3;       // 3 × uint16_t = 6 bytes

// ── Calibration constants (from robotiq_2f_gripper_ctrl.py) ──────────────────
// gPO = 13  → fully open  (0.087 m)
// gPO = 230 → fully closed (0.000 m)
static constexpr double REG_OPEN    = 13.0;
static constexpr double REG_CLOSE   = 230.0;
static constexpr double MAX_POS_M   = 0.087;  // metres, physical stroke

// ── Velocity / force calibration ─────────────────────────────────────────────
static constexpr double VEL_MIN_MS  = 0.013;
static constexpr double VEL_MAX_MS  = 0.1;
static constexpr double FORCE_MIN_N = 30.0;
static constexpr double FORCE_MAX_N = 100.0;

// ─────────────────────────────────────────────────────────────────────────────

Robotiq2F85Driver::Robotiq2F85Driver(const std::string& port,
                                     int baudrate,
                                     int slave_id)
  : ctx_(nullptr)
  , port_(port)
  , baudrate_(baudrate)
  , slave_id_(slave_id)
{}

Robotiq2F85Driver::~Robotiq2F85Driver()
{
  disconnect();
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

bool Robotiq2F85Driver::connect()
{
  std::lock_guard<std::mutex> lock(mutex_);

  ctx_ = modbus_new_rtu(port_.c_str(), baudrate_, 'N', 8, 1);
  if (!ctx_)
  {
    ROS_ERROR("[Robotiq2F85] modbus_new_rtu failed: %s", modbus_strerror(errno));
    return false;
  }

  if (modbus_set_slave(ctx_, slave_id_) != 0)
  {
    ROS_ERROR("[Robotiq2F85] modbus_set_slave(%d) failed: %s",
              slave_id_, modbus_strerror(errno));
    modbus_free(ctx_);
    ctx_ = nullptr;
    return false;
  }

  // 500 ms response timeout — gives the gripper enough time to answer
  modbus_set_response_timeout(ctx_, 0, 500000);

  if (modbus_connect(ctx_) != 0)
  {
    ROS_ERROR("[Robotiq2F85] Could not open %s: %s",
              port_.c_str(), modbus_strerror(errno));
    modbus_free(ctx_);
    ctx_ = nullptr;
    return false;
  }

  ROS_INFO("[Robotiq2F85] Connected on %s (baud=%d, slave=%d)",
           port_.c_str(), baudrate_, slave_id_);
  return true;
}

void Robotiq2F85Driver::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (ctx_)
  {
    modbus_close(ctx_);
    modbus_free(ctx_);
    ctx_ = nullptr;
    ROS_INFO("[Robotiq2F85] Disconnected");
  }
}

bool Robotiq2F85Driver::isConnected() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return ctx_ != nullptr;
}

// ── Activation ───────────────────────────────────────────────────────────────

bool Robotiq2F85Driver::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return writeRegs(0, 0, 0, 0, 0, 0);
}

bool Robotiq2F85Driver::activate(int timeout_ms)
{
  // Step 1 — Reset (rACT=0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!writeRegs(0, 0, 0, 0, 0, 0))
    {
      ROS_ERROR("[Robotiq2F85] activate: reset write failed");
      return false;
    }
  }

  // Required pause between reset and activate
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Step 2 — Activate (rACT=1, everything else 0 per Robotiq spec)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!writeRegs(1, 0, 0, 0, 0, 0))
    {
      ROS_ERROR("[Robotiq2F85] activate: activation write failed");
      return false;
    }
  }

  ROS_INFO("[Robotiq2F85] Activation command sent — waiting for gSTA==3 …");

  // Step 3 — Poll gSTA until 3 (or timeout)
  const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(timeout_ms < 0
                                                    ? INT_MAX
                                                    : timeout_ms);
  while (true)
  {
    if (timeout_ms >= 0 && std::chrono::steady_clock::now() >= deadline)
    {
      ROS_ERROR("[Robotiq2F85] Activation timed out after %d ms", timeout_ms);
      return false;
    }

    GripperStatus st = readStatus();

    if (st.fault)
    {
      ROS_ERROR("[Robotiq2F85] Fault 0x%02X during activation", st.fault);
      return false;
    }
    if (st.activated)
    {
      ROS_INFO("[Robotiq2F85] Gripper activated (blue LED on)");
      return true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

// ── Motion commands ───────────────────────────────────────────────────────────

bool Robotiq2F85Driver::move(double pos_m, double vel_ms, double force_n)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return writeRegs(1, 1, 0,
                   posToReg(pos_m),
                   velToReg(vel_ms),
                   forceToReg(force_n));
}

bool Robotiq2F85Driver::open(double vel_ms, double force_n)
{
  return move(MAX_POS_M, vel_ms, force_n);
}

bool Robotiq2F85Driver::close(double vel_ms, double force_n)
{
  return move(0.0, vel_ms, force_n);
}

bool Robotiq2F85Driver::stop()
{
  // rGTO=0 halts motion; keep rACT=1 so the gripper stays active
  std::lock_guard<std::mutex> lock(mutex_);
  return writeRegs(1, 0, 0, 0, 0, 0);
}

// ── Status ────────────────────────────────────────────────────────────────────

GripperStatus Robotiq2F85Driver::readStatus()
{
  uint8_t b[6] = {0};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    readRegs(b);  // errors are logged inside; b stays zero on failure
  }

  // Decode byte 0 bit-fields (same as baseRobotiq2FGripper.py)
  const uint8_t gACT = (b[0] >> 0) & 0x01;
  const uint8_t gSTA = (b[0] >> 4) & 0x03;
  const uint8_t gOBJ = (b[0] >> 6) & 0x03;

  GripperStatus s;
  s.activated       = (gACT == 1 && gSTA == 3);
  s.moving          = (gOBJ == 0);
  s.object_detected = (gOBJ == 1 || gOBJ == 2);
  s.at_position     = (gOBJ == 3);
  s.fault           = b[2] & 0x0F;   // lower nibble of byte 2
  s.pos_raw         = b[4];           // gPO
  s.current_raw     = b[5];           // gCU

  // Convert position: gPO=13 → 0.087 m (open), gPO=230 → 0.0 m (closed)
  const double raw_pos = MAX_POS_M / (REG_OPEN - REG_CLOSE)
                         * (static_cast<double>(s.pos_raw) - REG_CLOSE);
  s.position_m = std::max(0.0, std::min(MAX_POS_M, raw_pos));
  s.current_ma = s.current_raw * 0.1;

  return s;
}

// ── Private helpers ───────────────────────────────────────────────────────────

bool Robotiq2F85Driver::writeRegs(uint8_t rACT, uint8_t rGTO, uint8_t rATR,
                                   uint8_t rPR,  uint8_t rSP,  uint8_t rFR)
{
  if (!ctx_)
  {
    ROS_ERROR("[Robotiq2F85] writeRegs called while not connected");
    return false;
  }

  // Pack 6 bytes into 3 × uint16_t (big-endian, per baseRobotiq2FGripper.py)
  //
  //  byte 0 = rACT | (rGTO<<3) | (rATR<<4)   → high byte of reg[0]
  //  byte 1 = 0 (reserved)                    → low  byte of reg[0]
  //  byte 2 = 0 (reserved)                    → high byte of reg[1]
  //  byte 3 = rPR                             → low  byte of reg[1]
  //  byte 4 = rSP                             → high byte of reg[2]
  //  byte 5 = rFR                             → low  byte of reg[2]

  const uint8_t b0 = (rACT & 0x01) | ((rGTO & 0x01) << 3) | ((rATR & 0x01) << 4);
  uint16_t regs[3];
  regs[0] = (static_cast<uint16_t>(b0)  << 8) | 0x00u;
  regs[1] = (static_cast<uint16_t>(0)   << 8) | static_cast<uint16_t>(rPR);
  regs[2] = (static_cast<uint16_t>(rSP) << 8) | static_cast<uint16_t>(rFR);

  const int rc = modbus_write_registers(ctx_, WRITE_ADDR, NUM_REGS, regs);
  if (rc != NUM_REGS)
  {
    ROS_ERROR("[Robotiq2F85] modbus_write_registers failed: %s",
              modbus_strerror(errno));
    return false;
  }
  return true;
}

bool Robotiq2F85Driver::readRegs(uint8_t* bytes_out)
{
  if (!ctx_)
  {
    ROS_ERROR("[Robotiq2F85] readRegs called while not connected");
    return false;
  }

  uint16_t regs[3] = {0};
  const int rc = modbus_read_registers(ctx_, READ_ADDR, NUM_REGS, regs);
  if (rc != NUM_REGS)
  {
    ROS_ERROR("[Robotiq2F85] modbus_read_registers failed: %s",
              modbus_strerror(errno));
    return false;
  }

  // Unpack each 16-bit register into two bytes (big-endian)
  for (int i = 0; i < 3; ++i)
  {
    bytes_out[2 * i]     = static_cast<uint8_t>((regs[i] >> 8) & 0xFF);
    bytes_out[2 * i + 1] = static_cast<uint8_t>( regs[i]       & 0xFF);
  }
  return true;
}

// ── Conversion helpers ────────────────────────────────────────────────────────

uint8_t Robotiq2F85Driver::posToReg(double pos_m)
{
  // rPR = (REG_OPEN - REG_CLOSE) / MAX_POS_M * pos_m + REG_CLOSE
  //     = (13 - 230) / 0.087 * pos_m + 230   (from robotiq_2f_gripper_ctrl.py)
  const double r = (REG_OPEN - REG_CLOSE) / MAX_POS_M * pos_m + REG_CLOSE;
  return static_cast<uint8_t>(std::max(0.0, std::min(255.0, r)));
}

uint8_t Robotiq2F85Driver::velToReg(double vel_ms)
{
  // rSP = 255 / (VEL_MAX - VEL_MIN) * (vel - VEL_MIN)
  const double r = 255.0 / (VEL_MAX_MS - VEL_MIN_MS) * (vel_ms - VEL_MIN_MS);
  return static_cast<uint8_t>(std::max(0.0, std::min(255.0, r)));
}

uint8_t Robotiq2F85Driver::forceToReg(double force_n)
{
  // rFR = 255 / (FORCE_MAX - FORCE_MIN) * (force - FORCE_MIN)
  const double r = 255.0 / (FORCE_MAX_N - FORCE_MIN_N) * (force_n - FORCE_MIN_N);
  return static_cast<uint8_t>(std::max(0.0, std::min(255.0, r)));
}

}  // namespace robotiq_2f85
