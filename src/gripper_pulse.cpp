/**
 * gripper_pulse.cpp  —  No-ROS pulse latency tester for the Robotiq 2F-85
 *
 * Press <Enter> to trigger one open → hold → close pulse.
 * A background thread polls the gripper at POLL_HZ and drives a state machine.
 * After each pulse the timing breakdown is printed.
 *
 * Build:  catkin build robotiq_2f85_driver
 * Run:    rosrun robotiq_2f85_driver gripper_pulse
 *   (the gripper node in robotiq_2f85.launch must NOT be running at the same
 *    time — both would fight over /dev/ttyUSB0)
 *
 * Alternatively run standalone (no roscore needed):
 *   ./devel/lib/robotiq_2f85_driver/gripper_pulse
 */

#include <cstdio>
#include <cstring>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <iostream>

#include "robotiq_2f85_driver/robotiq_2f85_driver.h"

using namespace robotiq_2f85;
using Clock = std::chrono::steady_clock;
using TP    = Clock::time_point;

// ── Pulse parameters (runtime-mutable) ──────────────────────────────────────
struct PulseParams
{
    double open_pos  = 0.0222; // m   [0.0, 0.085]
    double close_pos = 0.021;  // m   [0.0, 0.085]
    double speed     = 0.1;    // m/s [0.013, 0.1]  — clamped by driver
    double force     = 30.0;   // N   [30, 100]      — clamped by driver
    int    pulse_ms  = 50;     // ms  hold at open position before closing
};

static constexpr int POLL_HZ          = 1000;
static constexpr int PHASE_TIMEOUT_MS = 500;

// ── State machine ─────────────────────────────────────────────────────────────
enum class Phase { IDLE, OPEN_WAIT, HOLD, CLOSE_WAIT };

struct PulseRecord
{
    TP trigger;                          // Enter pressed
    TP open_write_start;                 // just before move(open)
    TP open_write_done;                  // just after  move(open) returns
    TP first_motion_open;
    bool first_motion_open_seen = false;
    TP open_reached;                     // at_position or object_detected
    TP close_write_start;
    TP close_write_done;
    TP first_motion_close;
    bool first_motion_close_seen = false;
    TP close_reached;
};

static std::atomic<bool>  g_running       {true};
static std::atomic<bool>  g_pulse_request {false};
static std::atomic<bool>  g_result_ready  {false};
static std::atomic<Phase> g_phase         {Phase::IDLE};
static std::mutex         g_rec_mutex;
static PulseRecord        g_shared_rec;
static PulseParams        g_params;          // read by poll thread, written by main
static std::mutex         g_params_mutex;

// ── Helpers ───────────────────────────────────────────────────────────────────
static double ms_between(TP a, TP b)
{
    return std::chrono::duration<double, std::milli>(b - a).count();
}

static void print_params(const PulseParams& p)
{
    printf("  open=%.4f m  close=%.4f m  speed=%.3f m/s  force=%.0f N  pulse=%d ms\n",
           p.open_pos, p.close_pos, p.speed, p.force, p.pulse_ms);
    fflush(stdout);
}

static void print_result(const PulseRecord& r, int pulse_ms_used)
{
    printf("\n══ Pulse Timing Report ════════════════════════════════════\n");
    printf("  Enter → open write start       : %8.3f ms\n",
           ms_between(r.trigger,            r.open_write_start));
    printf("  Modbus write (open cmd)        : %8.3f ms\n",
           ms_between(r.open_write_start,   r.open_write_done));
    if (r.first_motion_open_seen)
        printf("  Open write done → first motion : %8.3f ms\n",
               ms_between(r.open_write_done,    r.first_motion_open));
    else
        printf("  Open write done → first motion :   (not seen — move < 1 poll)\n");
    printf("  Open write done → at_pos/grip  : %8.3f ms\n",
           ms_between(r.open_write_done,    r.open_reached));
    printf("  ─────────────────────────────────────────────────────────\n");
    printf("  Hold window (target %d ms)     : %8.3f ms\n",
           pulse_ms_used,
           ms_between(r.open_reached,       r.close_write_start));
    printf("  ─────────────────────────────────────────────────────────\n");
    printf("  Modbus write (close cmd)       : %8.3f ms\n",
           ms_between(r.close_write_start,  r.close_write_done));
    if (r.first_motion_close_seen)
        printf("  Close write done → first motion: %8.3f ms\n",
               ms_between(r.close_write_done,   r.first_motion_close));
    else
        printf("  Close write done → first motion:   (not seen — move < 1 poll)\n");
    printf("  Close write done → at_pos/grip : %8.3f ms\n",
           ms_between(r.close_write_done,   r.close_reached));
    printf("  ─────────────────────────────────────────────────────────\n");
    printf("  TOTAL  Enter → close settled   : %8.3f ms\n",
           ms_between(r.trigger,            r.close_reached));
    printf("══════════════════════════════════════════════════════════\n\n");
    fflush(stdout);
}

// ── Poll / state-machine thread ───────────────────────────────────────────────
static void poll_thread(Robotiq2F85Driver* drv)
{
    using namespace std::chrono;
    const auto period = microseconds(1000000 / POLL_HZ);

    Phase       phase = Phase::IDLE;
    PulseRecord rec;
    PulseParams snap;   // snapshot of params taken at trigger time
    TP          phase_start;

    while (g_running.load(std::memory_order_relaxed))
    {
        const TP loop_start = Clock::now();

        GripperStatus s = drv->readStatus();

        switch (phase)
        {
        // ── IDLE: wait for trigger ─────────────────────────────────────────
        case Phase::IDLE:
            if (g_pulse_request.exchange(false))
            {
                // Snapshot current params atomically
                { std::lock_guard<std::mutex> lk(g_params_mutex); snap = g_params; }

                rec = PulseRecord{};
                rec.trigger          = Clock::now();
                rec.open_write_start = Clock::now();
                drv->move(snap.open_pos, snap.speed, snap.force);
                rec.open_write_done  = Clock::now();
                phase_start = rec.open_write_done;
                phase = Phase::OPEN_WAIT;
                g_phase.store(phase, std::memory_order_relaxed);
            }
            break;

        // ── OPEN_WAIT: monitor motion toward open position ─────────────────
        case Phase::OPEN_WAIT:
            if (!rec.first_motion_open_seen && s.moving)
            {
                rec.first_motion_open      = Clock::now();
                rec.first_motion_open_seen = true;
            }
            if (s.at_position || s.object_detected)
            {
                rec.open_reached = Clock::now();
                phase_start = rec.open_reached;
                phase = Phase::HOLD;
                g_phase.store(phase, std::memory_order_relaxed);
                break;
            }
            if (ms_between(phase_start, Clock::now()) > PHASE_TIMEOUT_MS)
            {
                rec.open_reached = Clock::now();
                phase_start = rec.open_reached;
                phase = Phase::HOLD;
                g_phase.store(phase, std::memory_order_relaxed);
                printf("  [warn] OPEN_WAIT timed out — position may not have been reached\n");
            }
            break;

        // ── HOLD: wait snap.pulse_ms then fire close ──────────────────────
        case Phase::HOLD:
            if (ms_between(phase_start, Clock::now()) >= snap.pulse_ms)
            {
                rec.close_write_start = Clock::now();
                drv->move(snap.close_pos, snap.speed, snap.force);
                rec.close_write_done  = Clock::now();
                phase_start = rec.close_write_done;
                phase = Phase::CLOSE_WAIT;
                g_phase.store(phase, std::memory_order_relaxed);
            }
            break;

        // ── CLOSE_WAIT: monitor motion toward close position ──────────────
        case Phase::CLOSE_WAIT:
            if (!rec.first_motion_close_seen && s.moving)
            {
                rec.first_motion_close      = Clock::now();
                rec.first_motion_close_seen = true;
            }
            if (s.at_position || s.object_detected)
            {
                rec.close_reached = Clock::now();
                {
                    std::lock_guard<std::mutex> lk(g_rec_mutex);
                    g_shared_rec = rec;
                }
                g_result_ready.store(true, std::memory_order_release);
                phase = Phase::IDLE;
                g_phase.store(phase, std::memory_order_relaxed);
                break;
            }
            if (ms_between(phase_start, Clock::now()) > PHASE_TIMEOUT_MS)
            {
                rec.close_reached = Clock::now();
                {
                    std::lock_guard<std::mutex> lk(g_rec_mutex);
                    g_shared_rec = rec;
                }
                g_result_ready.store(true, std::memory_order_release);
                phase = Phase::IDLE;
                g_phase.store(phase, std::memory_order_relaxed);
                printf("  [warn] CLOSE_WAIT timed out\n");
            }
            break;
        }

        const auto elapsed = Clock::now() - loop_start;
        if (elapsed < period)
            std::this_thread::sleep_for(period - elapsed);
    }
}

// ── Parameter parser ─────────────────────────────────────────────────────────
// Accepts:  set open=0.03 close=0.025 speed=0.05 force=40 pulse=100
// Any subset of keys is valid; unrecognised tokens are silently ignored.
static void parse_set(const std::string& line, PulseParams& p)
{
    // simple key=value scanner
    size_t pos = 0;
    while (pos < line.size())
    {
        // find next '='
        size_t eq = line.find('=', pos);
        if (eq == std::string::npos) break;

        // key is the word before '='
        size_t kstart = eq;
        while (kstart > 0 && line[kstart-1] != ' ' && line[kstart-1] != '\t') --kstart;
        std::string key = line.substr(kstart, eq - kstart);

        // value is the non-space chars after '='
        size_t vstart = eq + 1;
        size_t vend   = vstart;
        while (vend < line.size() && line[vend] != ' ' && line[vend] != '\t') ++vend;
        std::string val = line.substr(vstart, vend - vstart);

        try {
            if      (key == "open")  p.open_pos  = std::stod(val);
            else if (key == "close") p.close_pos = std::stod(val);
            else if (key == "speed") p.speed     = std::stod(val);
            else if (key == "force") p.force     = std::stod(val);
            else if (key == "pulse") p.pulse_ms  = std::stoi(val);
        } catch (...) {
            printf("  [warn] could not parse value for '%s': '%s'\n",
                   key.c_str(), val.c_str());
        }

        pos = vend;
    }
}

// ── main ──────────────────────────────────────────────────────────────────────
int main()
{
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║       Robotiq 2F-85  Pulse Latency Tester               ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n");
    printf("  poll=%d Hz  phase_timeout=%d ms\n\n", POLL_HZ, PHASE_TIMEOUT_MS);
    printf("Commands:\n");
    printf("  <Enter>                         — fire pulse with current params\n");
    printf("  set [open=X] [close=X] [speed=X] [force=X] [pulse=X]\n");
    printf("                                  — update params (any subset)\n");
    printf("  show                            — print current params\n");
    printf("  Ctrl-D / Ctrl-C                 — quit\n\n");

    Robotiq2F85Driver driver("/dev/ttyUSB0", 115200, 9);
    if (!driver.connect())
    {
        fprintf(stderr, "ERROR: cannot open /dev/ttyUSB0 — is the gripper plugged in?\n");
        return 1;
    }

    // Verify activation
    {
        GripperStatus s = driver.readStatus();
        if (!s.activated)
        {
            printf("Gripper not yet active — running activation sequence ...\n");
            if (!driver.activate(8000))
            {
                fprintf(stderr, "Activation failed. Check power and cable.\n");
                return 1;
            }
        }
        GripperStatus s2 = driver.readStatus();
        printf("Gripper ready.  Position: %.4f m  fault: %u\n\n",
               s2.position_m, s2.fault);
    }

    // Start poll thread
    std::thread poller(poll_thread, &driver);

    printf("Current params: "); print_params(g_params);
    printf("\nReady. Press <Enter> to fire, or type 'set ...' / 'show'.\n\n");

    std::string line;
    while (g_running.load())
    {
        printf("> "); fflush(stdout);

        if (!std::getline(std::cin, line))
        {
            g_running.store(false);
            break;
        }

        // ── Trim leading whitespace ────────────────────────────────────────
        size_t first = line.find_first_not_of(" \t");
        std::string cmd = (first == std::string::npos) ? "" : line.substr(first);

        // ── show ──────────────────────────────────────────────────────────
        if (cmd == "show")
        {
            std::lock_guard<std::mutex> lk(g_params_mutex);
            print_params(g_params);
            continue;
        }

        // ── set ... ───────────────────────────────────────────────────────
        if (cmd.substr(0, 3) == "set")
        {
            std::lock_guard<std::mutex> lk(g_params_mutex);
            parse_set(cmd, g_params);
            printf("  Updated: "); print_params(g_params);
            continue;
        }

        // ── blank Enter → fire pulse ──────────────────────────────────────
        if (!cmd.empty())
        {
            printf("  Unknown command. Use: set [open=X] [close=X] [speed=X] [force=X] [pulse=X]  |  show  |  <Enter>\n");
            continue;
        }

        if (g_phase.load(std::memory_order_relaxed) != Phase::IDLE)
        {
            printf("  [busy — pulse in progress, please wait]\n");
            continue;
        }

        // Print params being used for this pulse
        { std::lock_guard<std::mutex> lk(g_params_mutex); printf("  Firing: "); print_params(g_params); }

        // Snapshot pulse_ms for the result printer (poll thread snapshots the rest)
        int pulse_ms_snap;
        { std::lock_guard<std::mutex> lk(g_params_mutex); pulse_ms_snap = g_params.pulse_ms; }

        g_pulse_request.store(true, std::memory_order_release);

        // Block until the poll thread posts a result
        while (!g_result_ready.load(std::memory_order_acquire) &&
               g_running.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if (g_result_ready.exchange(false))
        {
            PulseRecord r;
            {
                std::lock_guard<std::mutex> lk(g_rec_mutex);
                r = g_shared_rec;
            }
            print_result(r, pulse_ms_snap);
        }
    }

    g_running.store(false);
    poller.join();
    driver.disconnect();
    printf("Bye.\n");
    return 0;
}
