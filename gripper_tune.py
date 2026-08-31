#!/usr/bin/env python3
"""Find working gripper numbers on the real hardware. Runs ON THE PI, no ROS.

    python3 gripper_tune.py

Everything arm_pi_node.py does to the gripper is guesswork until it is measured
against an actual cube, because the numbers that matter -- where the pads
first touch, how much over-travel grips without hunting, whether the linkage
holds with the signal off -- are all mechanical.

Commands:
    o              open
    c <deg>        ramp closed to <deg> of over-travel past contact (default 15)
    h <deg>        set the sustained hold over-travel and stay there
    cut            stop the pulse train, leaving the servos limp
    touch          walk the jaws in 1 deg at a time to find first contact
    trim a|b <deg> nudge one servo's contact angle to fix an asymmetric gripper
    sweep          try each over-travel in turn, pausing for you to tug the cube
    show           print the constants to paste into arm_pi_node.py
    q              open the jaws and quit
"""
import sys, time

try:
    import lgpio
except ImportError:
    sys.exit("lgpio not found -- this script must run on the Pi.")

PINS = (23, 24)
OPEN_ANGLES = (0.0, 185.0)
touch_a, touch_b = 80.0, 100.0
hold_deg = 8.0
RAMP_DEG, RAMP_DT = 1.5, 0.02

h = lgpio.gpiochip_open(4)
for p in PINS:
    lgpio.gpio_claim_output(h, p)


def duty(angle):
    return angle / 180.0 * 10.0 + 2.5


def apply(duties):
    for pin, d in zip(PINS, duties):
        lgpio.tx_pwm(h, pin, 50, d)


def squeeze(deg):
    return (duty(touch_a + deg), duty(touch_b - deg))


def ramp(target, start=0.0):
    step = RAMP_DEG if target >= start else -RAMP_DEG
    for i in range(1, max(1, int(abs(target - start) / RAMP_DEG)) + 1):
        apply(squeeze(start + step * i)); time.sleep(RAMP_DT)
    apply(squeeze(target))


def cut():
    for pin in PINS:
        lgpio.tx_pwm(h, pin, 50, 0)


print(__doc__)
try:
    while True:
        try:
            parts = input("gripper> ").strip().split()
        except EOFError:
            break
        if not parts:
            continue
        cmd, args = parts[0], parts[1:]

        if cmd == "q":
            break
        elif cmd == "o":
            apply(tuple(duty(a) for a in OPEN_ANGLES)); print("  open")
        elif cmd == "c":
            d = float(args[0]) if args else 15.0
            ramp(d); print(f"  ramped to {d:+.1f} deg over-travel")
        elif cmd == "h":
            hold_deg = float(args[0]) if args else hold_deg
            apply(squeeze(hold_deg)); print(f"  holding at {hold_deg:+.1f} deg")
        elif cmd == "cut":
            cut()
            print("  signal off -- servos limp. Does the cube stay put?")
            print("  If yes, set GRIPPER_CUT_SIGNAL_WHEN_HELD = True. That is")
            print("  the best outcome: a servo with no pulses cannot jitter.")
        elif cmd == "touch":
            print("  walking in 1 deg at a time; watch for the pads meeting the cube")
            for d in range(0, 26):
                apply(squeeze(float(d))); time.sleep(0.25)
                print(f"    over-travel {d:3d} deg", end="\r")
            print("\n  note the angle where they FIRST touched; that is your")
            print("  contact point, and over-travel should be measured from it.")
        elif cmd == "trim":
            if len(args) == 2 and args[0] in "ab":
                if args[0] == "a":
                    touch_a += float(args[1])
                else:
                    touch_b -= float(args[1])
                apply(squeeze(hold_deg))
                print(f"  touch angles now a={touch_a:.1f} b={touch_b:.1f}")
            else:
                print("  usage: trim a|b <deg>")
        elif cmd == "sweep":
            for d in (4, 6, 8, 10, 12, 15, 20):
                ramp(float(d)); print(f"  over-travel {d:2d} deg -- tug the cube, "
                                      "then press Enter"); input()
        elif cmd == "show":
            print(f"\n  GRIPPER_TOUCH_A = {touch_a}\n  GRIPPER_TOUCH_B = {touch_b}")
            print(f"  GRIPPER_HOLD_DEG = {hold_deg}\n")
        else:
            print("  ?")
finally:
    apply(tuple(duty(a) for a in OPEN_ANGLES))
    time.sleep(0.5)
    cut()
    lgpio.gpiochip_close(h)
    print("jaws opened, signal off, chip closed.")
