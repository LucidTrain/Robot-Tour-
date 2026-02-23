#!/usr/bin/env python3
"""
calibrate_robot_tour.py

Guided calibration harness for diffdrive_blocks.py (Robot Tour).
- Menu-driven
- Pause prompts for repositioning & measurement
- In-memory parameter overrides for THIS session
- Prints suggested constants at the end (copy/paste)

Works with DiffDrive class in diffdrive_blocks.py.
"""

import time
import json
import math
from dataclasses import dataclass, asdict

import diffdrive_blocks as ddmod  # your existing file
from diffdrive_blocks import DiffDrive, clamp  # convenience


# ---------------------------
# Session parameters (overrides)
# ---------------------------

@dataclass
class Session:
    # motion speeds
    fwd_cm_s: float = 4.0
    turn_wheel_cm_s: float = 3.0

    # geometry / encoder
    wheel_diam_cm: float = ddmod.WHEEL_DIAMETER_CM
    track_width_cm: float = ddmod.TRACK_WIDTH_CM
    ticks_per_rev: int = ddmod.TICKS_PER_REV

    # filtering / watchdog
    edge_min_us: int = ddmod.EDGE_MIN_US
    stall_timeout_s: float = ddmod.STALL_TIMEOUT_S
    invalid_limit: int = ddmod.INVALID_LIMIT

    # control
    kp: float = ddmod.KP
    ki: float = ddmod.KI
    i_clamp: float = ddmod.I_CLAMP

    # derived
    ticks_per_cm: float = 0.0

    def recompute(self):
        self.ticks_per_cm = self.ticks_per_rev / (math.pi * self.wheel_diam_cm)


def apply_session_to_module(s: Session):
    """Override module-level constants used by DiffDrive internals."""
    ddmod.DEFAULT_FWD_CM_S = s.fwd_cm_s
    ddmod.DEFAULT_TURN_WHEEL_CM_S = s.turn_wheel_cm_s

    ddmod.WHEEL_DIAMETER_CM = s.wheel_diam_cm
    ddmod.TRACK_WIDTH_CM = s.track_width_cm
    ddmod.TICKS_PER_REV = s.ticks_per_rev

    ddmod.EDGE_MIN_US = s.edge_min_us
    ddmod.STALL_TIMEOUT_S = s.stall_timeout_s
    ddmod.INVALID_LIMIT = s.invalid_limit

    ddmod.KP = s.kp
    ddmod.KI = s.ki
    ddmod.I_CLAMP = s.i_clamp


def prompt(msg: str):
    input(f"\n{msg}\nPress ENTER when ready...")


def ticks_snapshot(robot: DiffDrive):
    tL, tR, invL, invR = robot._ticks_snapshot()
    return tL, tR, invL, invR


def run_and_capture_drive(robot: DiffDrive, dist_cmd_cm: float, speed_cm_s: float):
    tL0, tR0, invL0, invR0 = ticks_snapshot(robot)
    start = time.time()
    robot.drive_cm(dist_cmd_cm, speed_cm_s=speed_cm_s)
    end = time.time()
    tL1, tR1, invL1, invR1 = ticks_snapshot(robot)

    return {
        "cmd_cm": dist_cmd_cm,
        "speed_cm_s": speed_cm_s,
        "dt_s": end - start,
        "dL": tL1 - tL0,
        "dR": tR1 - tR0,
        "invL_delta": invL1 - invL0,
        "invR_delta": invR1 - invR0,
    }


def run_and_capture_turn(robot: DiffDrive, deg: float, wheel_cm_s: float):
    tL0, tR0, invL0, invR0 = ticks_snapshot(robot)
    start = time.time()
    robot.turn_deg(deg, wheel_cm_s=wheel_cm_s)
    end = time.time()
    tL1, tR1, invL1, invR1 = ticks_snapshot(robot)

    return {
        "cmd_deg": deg,
        "wheel_cm_s": wheel_cm_s,
        "dt_s": end - start,
        "dL": tL1 - tL0,
        "dR": tR1 - tR0,
        "invL_delta": invL1 - invL0,
        "invR_delta": invR1 - invR0,
    }


def suggest_from_straight(run, measured_cm: float):
    """
    Suggest ticks_per_cm and left/right scale ratio from a straight run.
    """
    dL = run["dL"]
    dR = run["dR"]
    avg_ticks = (dL + dR) / 2.0

    if measured_cm <= 1e-6:
        return None

    ticks_per_cm = avg_ticks / measured_cm

    # bias ratio: if L ticks > R ticks, left traveled "more" in encoder space.
    # use ratio to correct with per-wheel gain later if you add it.
    ratio_L_over_R = (abs(dL) / abs(dR)) if abs(dR) > 0 else None

    return {
        "ticks_per_cm_suggested": ticks_per_cm,
        "ratio_abs_L_over_R": ratio_L_over_R,
        "dL": dL, "dR": dR, "avg_ticks": avg_ticks,
    }


def suggest_track_width(old_track_width_cm: float, cmd_deg: float, actual_deg: float):
    """
    Turn angle is proportional to 1/track_width in this encoder-geometry model:
    target_wheel_cm = (track_width/2)*theta
    If actual_deg < cmd_deg, track_width is too big (commanded too few ticks) -> decrease.
    If actual_deg > cmd_deg, track_width is too small -> increase.
    Correction factor approx: new = old * (actual/cmd)
    """
    if abs(cmd_deg) < 1e-6:
        return None
    factor = (actual_deg / cmd_deg)
    return old_track_width_cm * factor


def menu():
    print("\n=== Robot Tour Calibration Harness ===")
    print("1) Selftest (encoders tick check)")
    print("2) Encoder sign sanity (lift wheels, slow forward/back)")
    print("3) Straight-line calibration (you measure distance)")
    print("4) Turn calibration (you measure angle error)")
    print("5) Slow-speed stability run (25cm)")
    print("6) Adjust session constants (in memory)")
    print("7) Print session constants (copy/paste block + JSON)")
    print("0) Exit\n")
    return input("Select> ").strip()


def adjust_session(s: Session):
    print("\n--- Current Session ---")
    for k, v in asdict(s).items():
        print(f"{k:18s} = {v}")
    print("")
    key = input("Enter field to change (or blank to cancel): ").strip()
    if not key:
        return
    if not hasattr(s, key):
        print("Unknown field.")
        return
    val = input(f"New value for {key}: ").strip()
    cur = getattr(s, key)
    try:
        if isinstance(cur, int):
            setattr(s, key, int(val))
        elif isinstance(cur, float):
            setattr(s, key, float(val))
        else:
            setattr(s, key, val)
        s.recompute()
        print("Updated.")
    except Exception as e:
        print(f"Failed to set: {e}")


def print_constants(s: Session, suggestions: dict):
    print("\n=== Suggested Constants (Session) ===\n")

    # Copy/paste block for diffdrive_blocks.py
    print("# ---- CALIBRATION OUTPUT ----")
    print(f"WHEEL_DIAMETER_CM = {s.wheel_diam_cm:.4f}")
    print(f"TRACK_WIDTH_CM    = {s.track_width_cm:.4f}")
    print(f"TICKS_PER_REV     = {s.ticks_per_rev:d}")
    print(f"EDGE_MIN_US       = {s.edge_min_us:d}")
    print(f"STALL_TIMEOUT_S   = {s.stall_timeout_s:.3f}")
    print(f"INVALID_LIMIT     = {s.invalid_limit:d}")
    print(f"KP                = {s.kp:.4f}")
    print(f"KI                = {s.ki:.4f}")
    print(f"I_CLAMP           = {s.i_clamp:.4f}")
    print(f"DEFAULT_FWD_CM_S        = {s.fwd_cm_s:.2f}")
    print(f"DEFAULT_TURN_WHEEL_CM_S = {s.turn_wheel_cm_s:.2f}")
    if suggestions.get("ticks_per_cm_suggested") is not None:
        tpcm = suggestions["ticks_per_cm_suggested"]
        # Convert back to implied wheel diameter if user prefers to keep TICKS_PER_REV fixed
        implied_wheel_diam = s.ticks_per_rev / (math.pi * tpcm)
        print(f"# ticks_per_cm suggested = {tpcm:.4f}")
        print(f"# implied WHEEL_DIAMETER_CM (if TICKS_PER_REV fixed) = {implied_wheel_diam:.4f}")
    if suggestions.get("ratio_abs_L_over_R") is not None:
        print(f"# abs(L/R ticks) ratio (straight bias) = {suggestions['ratio_abs_L_over_R']:.4f}")
    print("# ----------------------------\n")

    print("=== Raw Session JSON ===")
    blob = {"session": asdict(s), "suggestions": suggestions}
    print(json.dumps(blob, indent=2))


def main():
    s = Session()
    s.recompute()
    suggestions = {}

    # Apply once at start
    apply_session_to_module(s)

    print("Starting DiffDrive…")
    robot = DiffDrive()
    try:
        robot.wait_interlock()

        while True:
            choice = menu()

            # keep module constants synced
            apply_session_to_module(s)

            if choice == "0":
                break

            elif choice == "1":
                robot.selftest()

            elif choice == "2":
                prompt("Lift robot so wheels spin freely. We'll do slow forward for 1s.")
                t0 = ticks_snapshot(robot)
                robot.mot.set(0.22, 0.22)
                time.sleep(1.0)
                robot.mot.stop()
                t1 = ticks_snapshot(robot)
                print(f"Forward: dL={t1[0]-t0[0]:+d} dR={t1[1]-t0[1]:+d} invΔL={t1[2]-t0[2]} invΔR={t1[3]-t0[3]}")
                prompt("Now slow reverse for 1s.")
                t0 = ticks_snapshot(robot)
                robot.mot.set(-0.22, -0.22)
                time.sleep(1.0)
                robot.mot.stop()
                t1 = ticks_snapshot(robot)
                print(f"Reverse: dL={t1[0]-t0[0]:+d} dR={t1[1]-t0[1]:+d} invΔL={t1[2]-t0[2]} invΔR={t1[3]-t0[3]}")
                print("Expectation: forward should make BOTH dL and dR positive (or both negative consistently).")

            elif choice == "3":
                prompt("Place robot on floor at a start line with room to go straight. We'll command a long run.")
                dist_cmd = float(input("Command distance in cm (recommend 100): ").strip() or "100")
                run = run_and_capture_drive(robot, dist_cmd, s.fwd_cm_s)
                print("\n--- Straight Run Telemetry ---")
                print(json.dumps(run, indent=2))
                measured = float(input("\nEnter measured actual distance traveled in cm (tape measure): ").strip())
                sug = suggest_from_straight(run, measured)
                if sug:
                    suggestions.update(sug)
                    print("\n--- Suggested ---")
                    print(json.dumps(sug, indent=2))

            elif choice == "4":
                prompt("Place robot on floor with a clear heading reference line. We'll do an in-place turn.")
                cmd_deg = float(input("Command turn degrees (recommend 360 or 4x90=360): ").strip() or "360")
                run = run_and_capture_turn(robot, cmd_deg, s.turn_wheel_cm_s)
                print("\n--- Turn Telemetry ---")
                print(json.dumps(run, indent=2))
                actual = float(input("\nEnter actual angle turned in degrees (estimate with protractor/marks): ").strip())
                new_tw = suggest_track_width(s.track_width_cm, cmd_deg, actual)
                if new_tw:
                    print(f"\nSuggested TRACK_WIDTH_CM: {new_tw:.4f} (was {s.track_width_cm:.4f})")
                    accept = input("Apply to session? (y/N): ").strip().lower()
                    if accept == "y":
                        s.track_width_cm = float(new_tw)
                        s.recompute()

            elif choice == "5":
                prompt("Slow 25cm stability test. Clear path.")
                run = run_and_capture_drive(robot, 25.0, s.fwd_cm_s)
                print("\n--- 25cm Telemetry ---")
                print(json.dumps(run, indent=2))
                print("If invalid transitions spike on one side, lower EDGE_MIN_US slightly or improve wiring/pullups.")

            elif choice == "6":
                adjust_session(s)

            elif choice == "7":
                print_constants(s, suggestions)

            else:
                print("Unknown selection.")

    except KeyboardInterrupt:
        print("\nCtrl+C: stopping.")
    finally:
        try:
            robot.mot.stop()
        except Exception:
            pass
        robot.close()


if __name__ == "__main__":
    main()
