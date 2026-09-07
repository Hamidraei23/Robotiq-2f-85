#pragma once

#include <string>
#include <cstdint>
#include <mutex>
#include <modbus/modbus.h>

namespace robotiq_2f85
{

/**
 * Physical state reported by the gripper's input registers.
 * Populated by Robotiq2F85Driver::readStatus().
 */
struct GripperStatus
{
  // ── Activation ────────────────────────────────────────────────────────────
  bool activated;        ///< true when gACT==1 and gSTA==3 (blue LED, ready)

  // ── Motion state (mutually exclusive) ────────────────────────────────────
  bool moving;           ///< gOBJ==0: finger is in motion
  bool object_detected;  ///< gOBJ==1 or 2: finger stopped by contact before target
  bool at_position;      ///< gOBJ==3: finger reached requested position

  // ── Raw register values ───────────────────────────────────────────────────
  uint8_t fault;         ///< gFLT nibble; 0 == no fault
  uint8_t pos_raw;       ///< gPO: 13 = fully open, 230 = fully closed
  uint8_t current_raw;   ///< gCU: 0-255 (multiply by 0.1 for mA)

  // ── Converted values ──────────────────────────────────────────────────────
  double position_m;     ///< finger gap in metres [0.0, 0.087]
  double current_ma;     ///< current draw in milliamps
};


/**
 * Thin C++ wrapper around libmodbus RTU for the Robotiq 2F-85 gripper.
 *
 * All public methods are thread-safe: an internal mutex serialises every
 * Modbus transaction so the driver can be safely shared between a ROS timer
 * callback (status polling) and service callbacks (commands).
 *
 * Register map (from robotiq_2f_gripper_control / comModbusRtu.py):
 *   Output (write) base address : 0x03E8 (1000)
 *   Input  (read)  base address : 0x07D0 (2000)
 *   Modbus RTU slave ID         : 0x09   (9)
 */
class Robotiq2F85Driver
{
public:
  /**
   * @param port      Serial device, e.g. "/dev/ttyUSB0"
   * @param baudrate  Must match gripper DIP-switch setting (default 115200)
   * @param slave_id  Modbus RTU slave address (default 0x09 = 9)
   */
  explicit Robotiq2F85Driver(const std::string& port,
                              int baudrate = 115200,
                              int slave_id = 0x09);
  ~Robotiq2F85Driver();

  // Non-copyable — the Modbus context is a unique resource
  Robotiq2F85Driver(const Robotiq2F85Driver&)            = delete;
  Robotiq2F85Driver& operator=(const Robotiq2F85Driver&) = delete;

  // ── Lifecycle ─────────────────────────────────────────────────────────────

  /**
   * Open the serial port and configure Modbus RTU.
   * Must be called before any other method.
   * @return true on success
   */
  bool connect();

  /**
   * Close the serial port and free the Modbus context.
   * Safe to call even if connect() was never called.
   */
  void disconnect();

  /** @return true if the port is open */
  bool isConnected() const;

  // ── Activation (run once after power-on) ──────────────────────────────────

  /**
   * Send a reset command (rACT=0).  Does NOT block.
   */
  bool reset();

  /**
   * Full activation sequence: reset → wait 500 ms → activate → poll until
   * gSTA==3 (blue LED).  Blocks the calling thread.
   *
   * @param timeout_ms  Maximum time to wait for activation (-1 = forever).
   * @return true if the gripper reaches the activated state.
   */
  bool activate(int timeout_ms = 5000);

  // ── Motion commands (non-blocking — return after the Modbus write) ────────

  /**
   * Go to an arbitrary position.
   *
   * @param pos_m    Target gap in metres.  Clamped to [0.0, 0.087].
   *                 0.0   = fully closed
   *                 0.087 = fully open
   * @param vel_ms   Speed in m/s.  Clamped to [0.013, 0.1].
   * @param force_n  Maximum force in N.  Clamped to [10, 100].
   * @return true if the Modbus write succeeded.
   */
  bool move(double pos_m, double vel_ms = 0.05, double force_n = 50.0);

  /** Fully open at maximum speed and force. */
  bool open(double vel_ms = 0.1, double force_n = 100.0);

  /** Fully close at maximum speed and force. */
  bool close(double vel_ms = 0.1, double force_n = 100.0);

  /**
   * Stop any ongoing motion immediately (sets rGTO=0).
   * The gripper stays active (rACT=1) and can accept new commands afterwards.
   */
  bool stop();

  // ── Status ────────────────────────────────────────────────────────────────

  /**
   * Read the six input bytes from the gripper and decode them.
   * Typically called from a timer at ~50 Hz.
   */
  GripperStatus readStatus();

private:
  // ── Low-level Modbus helpers ──────────────────────────────────────────────

  /**
   * Pack six register fields into three uint16_t registers and write them to
   * the gripper output address 0x03E8.
   * Caller MUST hold mutex_.
   */
  bool writeRegs(uint8_t rACT, uint8_t rGTO, uint8_t rATR,
                 uint8_t rPR,  uint8_t rSP,  uint8_t rFR);

  /**
   * Read three holding registers from 0x07D0 and unpack them into six bytes.
   * Caller MUST hold mutex_.
   * @param bytes_out  Must point to at least 6 bytes.
   */
  bool readRegs(uint8_t* bytes_out);

  // ── Register-value conversions ────────────────────────────────────────────
  // (Formulae from robotiq_2f_gripper_ctrl.py)

  /** metres → rPR byte.  0.0 m → 230 (closed), 0.087 m → 13 (open). */
  static uint8_t posToReg(double pos_m);

  /** m/s   → rSP byte.  Range [0.013, 0.1] m/s → [0, 255]. */
  static uint8_t velToReg(double vel_ms);

  /** N     → rFR byte.  Range [10, 100] N → [0, 255]. */
  static uint8_t forceToReg(double force_n);

  // ── Members ───────────────────────────────────────────────────────────────
  modbus_t*         ctx_;
  const std::string port_;
  const int         baudrate_;
  const int         slave_id_;
  mutable std::mutex mutex_;
};

}  // namespace robotiq_2f85
