#!/usr/bin/env python3
"""
diffdrive_blocks.py
Encoder-only (gyro-independent) differential drive that can:
  - drive fixed blocks: 25cm / 50cm
  - turn in-place: +/- 90 degrees
Includes:
  - robust quadrature decoder (state-table) with invalid-transition rejection
  - per-channel microsecond edge filter (debounce)
  - per-wheel stall watchdog
  - start interlock on GPIO13 (BCM) like your earlier scripts
  - self-test that spins wheels briefly and verifies encoder ticks

IMPORTANT:
You asked "use values from previous files". I cannot see your previous file contents here.
So the CONFIG section below is arranged to be a direct copy/paste from your prior scripts:
  - GPIO pins for motors and encoders
  - wheel geometry and encoder constants
Just replace the CONFIG constants with the same numbers you used in testdrive12-16.

Run:
  python3 diffdrive_blocks.py

Keys:
  - Press Ctrl+C to stop.
"""

import time
import sys
import math
from dataclasses import dataclass

# =========================
# CONFIG (COPY FROM YOUR PREVIOUS FILES)
# =========================

# Start interlock (same as your logs)
INTERLOCK_GPIO = 13  # BCM

# ---- Encoder GPIOs (BCM) ----
# Set these to the SAME pins you used previously.
ENC_LA = 5    # TODO: replace with your left encoder A pin
ENC_LB = 6    # TODO: replace with your left encoder B pin
ENC_RA = 16   # TODO: replace with your right encoder A pin
ENC_RB = 20   # TODO: replace with your right encoder B pin

# ---- Motor driver GPIOs (BCM) ----
# Generic H-bridge: PWM + two direction pins per side.
# Set these to your real pins from earlier scripts.
MOT_L_PWM = 12   # TODO
MOT_L_IN1 = 23   # TODO
MOT_L_IN2 = 24   # TODO

MOT_R_PWM = 13   # TODO (NOTE: don't collide with INTERLOCK_GPIO; change one if needed)
MOT_R_IN1 = 27   # TODO
MOT_R_IN2 = 22   # TODO

PWM_HZ = 20000

# ---- Robot geometry ----
WHEEL_DIAMETER_CM = 6.5   # TODO: copy from your prior files
TRACK_WIDTH_CM    = 13.0  # TODO: wheel center-to-center

# ---- Encoder resolution ----
# ticks_per_rev should be the number of quadrature counts per wheel revolution
# in YOUR software units. If you count every valid transition (x4), use CPR*4.
TICKS_PER_REV = 360  # TODO

# ---- Safety / filtering ----
EDGE_MIN_US = 80     # per-channel minimum edge spacing; start 50-150us
STALL_TIMEOUT_S = 0.35  # if commanded and no ticks for this long -> fault
INVALID_LIMIT = 200     # if too many invalid transitions in a short run -> fault

# ---- Control tuning (simple PI on wheel speed) ----
KP = 0.35
KI = 1.20
I_CLAMP = 0.8

# speed targets in cm/s for "blocks"
DEFAULT_FWD_CM_S = 8.0

# Turn speed target (wheel linear speed in cm/s during in-place turn)
DEFAULT_TURN_WHEEL_CM_S = 6.0

# =========================
# END CONFIG
# =========================


# Try pigpio first (best for callbacks + microsecond timing)
_USE_PIGPIO = False
try:
    import pigpio  # type: ignore
    _USE_PIGPIO = True
except Exception:
    _USE_PIGPIO = False

if not _USE_PIGPIO:
    import RPi.GPIO as GPIO  # type: ignore


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


@dataclass
class EncoderStats:
    ticks: int = 0
    invalid: int = 0
    last_us: int = 0
    last_state: int = 0  # 2-bit AB state
    last_tick_time: float = 0.0


class QuadDecoder:
    """
    Robust quadrature decoder using a 4x4 transition table.
    Valid transitions change by 1 bit. Invalid transitions (00->11 etc.) are ignored and counted.
    """
    # state = (A<<1)|B
    # delta table indexed by (old_state<<2)|new_state
    _DELTA = [
        0,  +1,  -1,  0,
        -1,  0,   0, +1,
        +1,  0,   0, -1,
        0,  -1,  +1,  0
    ]

    def __init__(self, name: str, min_us: int):
        self.name = name
        self.min_us = min_us
        self.s = EncoderStats()

    def update(self, a: int, b: int, now_us: int):
        # Edge spacing debounce
        if self.s.last_us and (now_us - self.s.last_us) < self.min_us:
            return  # ignore too-fast edge
        self.s.last_us = now_us

        new_state = ((a & 1) << 1) | (b & 1)
        idx = (self.s.last_state << 2) | new_state
        d = self._DELTA[idx]
        if d == 0 and new_state != self.s.last_state:
            # invalid transition
            self.s.invalid += 1
        else:
            self.s.ticks += d
            self.s.last_tick_time = time.time()

        self.s.last_state = new_state


class MotorDriver:
    """
    Generic dual H-bridge with PWM and IN1/IN2 per side.
    command in [-1, 1]
    """
    def __init__(self):
        if _USE_PIGPIO:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("pigpio daemon not running. Start with: sudo pigpiod")
            for pin in [MOT_L_IN1, MOT_L_IN2, MOT_R_IN1, MOT_R_IN2, MOT_L_PWM, MOT_R_PWM]:
                self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(MOT_L_PWM, PWM_HZ)
            self.pi.set_PWM_frequency(MOT_R_PWM, PWM_HZ)
            self._set_pwm(MOT_L_PWM, 0)
            self._set_pwm(MOT_R_PWM, 0)
        else:
            GPIO.setmode(GPIO.BCM)
            for pin in [MOT_L_IN1, MOT_L_IN2, MOT_R_IN1, MOT_R_IN2, MOT_L_PWM, MOT_R_PWM]:
                GPIO.setup(pin, GPIO.OUT)
            self.pwmL = GPIO.PWM(MOT_L_PWM, PWM_HZ)
            self.pwmR = GPIO.PWM(MOT_R_PWM, PWM_HZ)
            self.pwmL.start(0)
            self.pwmR.start(0)

    def _set_dir(self, in1, in2, fwd: bool):
        if _USE_PIGPIO:
            self.pi.write(in1, 1 if fwd else 0)
            self.pi.write(in2, 0 if fwd else 1)
        else:
            GPIO.output(in1, GPIO.HIGH if fwd else GPIO.LOW)
            GPIO.output(in2, GPIO.LOW if fwd else GPIO.HIGH)

    def _set_pwm(self, pin, duty_0_255: int):
        self.pi.set_PWM_dutycycle(pin, duty_0_255)

    def set(self, uL: float, uR: float):
        uL = clamp(uL, -1.0, 1.0)
        uR = clamp(uR, -1.0, 1.0)

        self._set_dir(MOT_L_IN1, MOT_L_IN2, fwd=(uL >= 0))
        self._set_dir(MOT_R_IN1, MOT_R_IN2, fwd=(uR >= 0))

        dL = int(abs(uL) * 255)
        dR = int(abs(uR) * 255)

        if _USE_PIGPIO:
            self._set_pwm(MOT_L_PWM, dL)
            self._set_pwm(MOT_R_PWM, dR)
        else:
            self.pwmL.ChangeDutyCycle(abs(uL) * 100.0)
            self.pwmR.ChangeDutyCycle(abs(uR) * 100.0)

    def stop(self):
        self.set(0.0, 0.0)

    def close(self):
        self.stop()
        if _USE_PIGPIO:
            self.pi.stop()
        else:
            self.pwmL.stop()
            self.pwmR.stop()
            GPIO.cleanup()


class EncoderIO:
    """
    Sets up callbacks for both encoders and provides tick snapshots.
    """
    def __init__(self, left: QuadDecoder, right: QuadDecoder):
        self.left = left
        self.right = right

        if _USE_PIGPIO:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("pigpio daemon not running. Start with: sudo pigpiod")
            for pin in [ENC_LA, ENC_LB, ENC_RA, ENC_RB]:
                self.pi.set_mode(pin, pigpio.INPUT)
                # prefer external pullups; internal can be enabled as a fallback:
                self.pi.set_pull_up_down(pin, pigpio.PUD_UP)
            # initialize states
            self.left.s.last_state  = ((self.pi.read(ENC_LA) << 1) | self.pi.read(ENC_LB))
            self.right.s.last_state = ((self.pi.read(ENC_RA) << 1) | self.pi.read(ENC_RB))

            # register callbacks on BOTH pins
            self.cb_la = self.pi.callback(ENC_LA, pigpio.EITHER_EDGE, self._cb_left)
            self.cb_lb = self.pi.callback(ENC_LB, pigpio.EITHER_EDGE, self._cb_left)
            self.cb_ra = self.pi.callback(ENC_RA, pigpio.EITHER_EDGE, self._cb_right)
            self.cb_rb = self.pi.callback(ENC_RB, pigpio.EITHER_EDGE, self._cb_right)
        else:
            GPIO.setmode(GPIO.BCM)
            for pin in [ENC_LA, ENC_LB, ENC_RA, ENC_RB]:
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self.left.s.last_state  = ((GPIO.input(ENC_LA) << 1) | GPIO.input(ENC_LB))
            self.right.s.last_state = ((GPIO.input(ENC_RA) << 1) | GPIO.input(ENC_RB))

            GPIO.add_event_detect(ENC_LA, GPIO.BOTH, callback=self._cb_left_gpio)
            GPIO.add_event_detect(ENC_LB, GPIO.BOTH, callback=self._cb_left_gpio)
            GPIO.add_event_detect(ENC_RA, GPIO.BOTH, callback=self._cb_right_gpio)
            GPIO.add_event_detect(ENC_RB, GPIO.BOTH, callback=self._cb_right_gpio)

    def _cb_left(self, gpio, level, tick):
        a = self.pi.read(ENC_LA)
        b = self.pi.read(ENC_LB)
        self.left.update(a, b, tick)

    def _cb_right(self, gpio, level, tick):
        a = self.pi.read(ENC_RA)
        b = self.pi.read(ENC_RB)
        self.right.update(a, b, tick)

    def _cb_left_gpio(self, channel):
        now_us = int(time.time() * 1e6)
        a = GPIO.input(ENC_LA)
        b = GPIO.input(ENC_LB)
        self.left.update(a, b, now_us)

    def _cb_right_gpio(self, channel):
        now_us = int(time.time() * 1e6)
        a = GPIO.input(ENC_RA)
        b = GPIO.input(ENC_RB)
        self.right.update(a, b, now_us)

    def close(self):
        if _USE_PIGPIO:
            self.cb_la.cancel(); self.cb_lb.cancel()
            self.cb_ra.cancel(); self.cb_rb.cancel()
            self.pi.stop()
        else:
            GPIO.cleanup()


class DiffDrive:
    def __init__(self):
        self.encL = QuadDecoder("L", EDGE_MIN_US)
        self.encR = QuadDecoder("R", EDGE_MIN_US)
        self.encio = EncoderIO(self.encL, self.encR)
        self.mot = MotorDriver()

        self.ticks_per_cm = TICKS_PER_REV / (math.pi * WHEEL_DIAMETER_CM)

        self.iL = 0.0
        self.iR = 0.0
        self.last_ctrl = time.time()

        self.last_ticks_L = 0
        self.last_ticks_R = 0
        self.last_time = time.time()

    def wait_interlock(self):
        print(f"WAIT: Start interlock on GPIO{INTERLOCK_GPIO} (BCM). Waiting for level=1 ...")
        if _USE_PIGPIO:
            pi = pigpio.pi()
            pi.set_mode(INTERLOCK_GPIO, pigpio.INPUT)
            pi.set_pull_up_down(INTERLOCK_GPIO, pigpio.PUD_DOWN)
            while pi.read(INTERLOCK_GPIO) == 0:
                time.sleep(0.02)
            pi.stop()
        else:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(INTERLOCK_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            while GPIO.input(INTERLOCK_GPIO) == 0:
                time.sleep(0.02)
        print("START signal detected. Robot enabled.\n")

    def _ticks_snapshot(self):
        return self.encL.s.ticks, self.encR.s.ticks, self.encL.s.invalid, self.encR.s.invalid

    def _ticks_to_cm(self, ticks):
        return ticks / self.ticks_per_cm

    def _wheel_speed_ticks_s(self, ticks_now, ticks_prev, dt):
        return (ticks_now - ticks_prev) / dt if dt > 1e-6 else 0.0

    def _pi_speed_control(self, sp_ticks_s_L, sp_ticks_s_R, dt):
        # measured
        tL, tR, invL, invR = self._ticks_snapshot()
        vL = self._wheel_speed_ticks_s(tL, self.last_ticks_L, dt)
        vR = self._wheel_speed_ticks_s(tR, self.last_ticks_R, dt)
        self.last_ticks_L, self.last_ticks_R = tL, tR

        # PI
        eL = sp_ticks_s_L - vL
        eR = sp_ticks_s_R - vR
        self.iL = clamp(self.iL + eL * dt, -I_CLAMP, I_CLAMP)
        self.iR = clamp(self.iR + eR * dt, -I_CLAMP, I_CLAMP)

        uL = KP * eL + KI * self.iL
        uR = KP * eR + KI * self.iR

        # normalize to [-1,1] conservatively
        uL = clamp(uL, -1.0, 1.0)
        uR = clamp(uR, -1.0, 1.0)

        return uL, uR, vL, vR, invL, invR

    def _watchdogs(self, uL, uR):
        now = time.time()
        # stall watchdog: if commanded significantly and no tick for too long
        if abs(uL) > 0.2 and (now - self.encL.s.last_tick_time) > STALL_TIMEOUT_S:
            raise RuntimeError("FAULT: Left encoder stalled while commanded (check wiring/noise/glitch filter).")
        if abs(uR) > 0.2 and (now - self.encR.s.last_tick_time) > STALL_TIMEOUT_S:
            raise RuntimeError("FAULT: Right encoder stalled while commanded (check wiring/noise/glitch filter).")

        # invalid transition watchdog (noise)
        if self.encL.s.invalid > INVALID_LIMIT or self.encR.s.invalid > INVALID_LIMIT:
            raise RuntimeError("FAULT: Too many invalid quadrature transitions (noise).")

    def selftest(self):
        print("SELFTEST: Spinning each wheel briefly to verify encoders tick...")
        # snapshot
        tL0, tR0, _, _ = self._ticks_snapshot()

        # spin left only
        self.mot.set(0.35, 0.0)
        time.sleep(0.35)
        self.mot.stop()
        time.sleep(0.15)
        tL1, tR1, _, _ = self._ticks_snapshot()

        # spin right only
        self.mot.set(0.0, 0.35)
        time.sleep(0.35)
        self.mot.stop()
        time.sleep(0.15)
        tL2, tR2, _, _ = self._ticks_snapshot()

        dL = tL1 - tL0
        dR = tR2 - tR1

        ok = True
        if abs(dL) < 3:
            ok = False
            print(f"  FAIL: Left encoder barely moved (Δ={dL}).")
            print("        -> Lower EDGE_MIN_US, add pullups/buffering, check A/B/GND, route away from motors.")
        if abs(dR) < 3:
            ok = False
            print(f"  FAIL: Right encoder barely moved (Δ={dR}).")
            print("        -> Lower EDGE_MIN_US, add pullups/buffering, check A/B/GND, route away from motors.")
        print("SELFTEST complete.\n")
        return ok

    def drive_cm(self, dist_cm: float, speed_cm_s: float = DEFAULT_FWD_CM_S):
        # distance target in ticks (average of wheels)
        target_ticks = int(dist_cm * self.ticks_per_cm)
        tL0, tR0, _, _ = self._ticks_snapshot()
        start = time.time()
        self.last_ticks_L, self.last_ticks_R = tL0, tR0
        self.encL.s.last_tick_time = time.time()
        self.encR.s.last_tick_time = time.time()

        # convert speed to ticks/s
        sp = speed_cm_s * self.ticks_per_cm

        print(f"DRIVE: {dist_cm:.1f} cm target at {speed_cm_s:.1f} cm/s (target_ticks≈{target_ticks})")
        while True:
            now = time.time()
            dt = now - self.last_time
            self.last_time = now

            tL, tR, invL, invR = self._ticks_snapshot()
            dL = tL - tL0
            dR = tR - tR0
            prog = int((dL + dR) / 2)

            # simple taper near target
            remaining = target_ticks - prog
            if remaining <= 0:
                break
            taper = clamp(remaining / (0.25 * target_ticks + 1), 0.2, 1.0)
            sp_now = sp * taper

            uL, uR, vL, vR, invL, invR = self._pi_speed_control(sp_now, sp_now, dt)
            self._watchdogs(uL, uR)
            self.mot.set(uL, uR)

            if int((now - start) * 10) % 5 == 0:  # occasional print
                cm_prog = self._ticks_to_cm(prog)
                print(f"  prog={cm_prog:6.1f}cm | ticks L={dL:+6d} R={dR:+6d} | vL={vL:+7.1f} vR={vR:+7.1f} | inv L={invL} R={invR} | uL={uL:+.2f} uR={uR:+.2f}")

            time.sleep(0.02)

        self.mot.stop()
        print("DRIVE done.\n")

    def turn_deg(self, deg: float, wheel_cm_s: float = DEFAULT_TURN_WHEEL_CM_S):
        """
        In-place turn using encoder-only geometry:
          wheel travel per rad = (TRACK_WIDTH/2)
          target_wheel_cm = (TRACK_WIDTH/2) * theta_rad
        Left forward, right backward for +deg (CCW), reverse for -deg.
        """
        theta = math.radians(deg)
        target_wheel_cm = (TRACK_WIDTH_CM / 2.0) * abs(theta)
        target_ticks = int(target_wheel_cm * self.ticks_per_cm)

        tL0, tR0, _, _ = self._ticks_snapshot()
        self.last_ticks_L, self.last_ticks_R = tL0, tR0
        self.encL.s.last_tick_time = time.time()
        self.encR.s.last_tick_time = time.time()

        sp = wheel_cm_s * self.ticks_per_cm
        # direction
        sgn = 1.0 if deg >= 0 else -1.0

        print(f"TURN: {deg:+.1f} deg target (wheel_cm≈{target_wheel_cm:.2f}, ticks≈{target_ticks})")
        start = time.time()
        while True:
            now = time.time()
            dt = now - self.last_time
            self.last_time = now

            tL, tR, invL, invR = self._ticks_snapshot()
            dL = tL - tL0
            dR = tR - tR0

            # progress is average absolute wheel ticks (since one side reverses)
            prog = int((abs(dL) + abs(dR)) / 2)
            remaining = target_ticks - prog
            if remaining <= 0:
                break
            taper = clamp(remaining / (0.25 * target_ticks + 1), 0.2, 1.0)
            sp_now = sp * taper

            # left and right setpoints opposite
            uL, uR, vL, vR, invL, invR = self._pi_speed_control(sgn * sp_now, -sgn * sp_now, dt)
            self._watchdogs(uL, uR)
            self.mot.set(uL, uR)

            if int((now - start) * 10) % 5 == 0:
                print(f"  prog≈{self._ticks_to_cm(prog):5.1f}cm-wheel | dL={dL:+6d} dR={dR:+6d} | inv L={invL} R={invR} | uL={uL:+.2f} uR={uR:+.2f}")

            time.sleep(0.02)

        self.mot.stop()
        print("TURN done.\n")

    def close(self):
        self.mot.close()
        try:
            self.encio.close()
        except Exception:
            pass


def main():
    print("DiffDrive: Encoder-only blocks + 90° turns (gyro-independent).")
    if _USE_PIGPIO:
        print("Using pigpio callbacks (recommended).")
    else:
        print("Using RPi.GPIO events (works, but pigpio is more robust).")
    print("")

    dd = DiffDrive()
    try:
        dd.wait_interlock()
        dd.selftest()

        # Demo sequence: 25cm forward, right 90, 25cm forward, left 90, 50cm forward
        dd.drive_cm(25.0)
        dd.turn_deg(-90.0)
        dd.drive_cm(25.0)
        dd.turn_deg(+90.0)
        dd.drive_cm(50.0)

        print("Done.")
    except KeyboardInterrupt:
        print("\nCtrl+C: stopping.")
    except Exception as e:
        print(f"\n{e}\nStopping motors.")
    finally:
        try:
            dd.mot.stop()
        except Exception:
            pass
        dd.close()


if __name__ == "__main__":
    main()
