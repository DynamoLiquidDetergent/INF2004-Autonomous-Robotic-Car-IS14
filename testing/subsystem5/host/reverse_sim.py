#!/usr/bin/env python3
"""
S5-H80  Design check: should the car follow the line while backing away?

Model: differential-drive car, line sensors a distance L AHEAD of the wheel axle
(L = 50, 80, 110 mm tried), wheel track W = 130 mm, back-away of 150 mm at 40 mm/s
(3.75 s), line sensor read in whole zones of 8 mm limited to +/-3 zones, one control
step of delay.  Start offsets -16..+16 mm and heading errors -0.1..+0.1 rad.

Compared: (a) straight reverse, (b) the forward line controller with the SAME steering
sign, at a quarter of the forward gain (3 mm/s per zone).  The sensor offset at the end
of the reverse is the score.

Run:  python3 testing/subsystem5/host/reverse_sim.py
Exit status 0 when the result supports the decision (no line feedback in reverse).
"""
import math, sys

def sim(v, k, sign, L, W=130.0, zone=8.0, y0=0.0, th0=0.0, T=3.75, dt=0.02, lim=12, delay=1):
    y = y0 - L * math.sin(th0)           # axle centre; the line is y = 0
    th, prev, buf, mx = th0, 0, [0.0] * delay, 0.0
    t = 0.0
    while t < T:
        ys = y + L * math.sin(th)        # sensor offset from the line
        e = max(-3, min(3, round(ys / zone)))
        s = max(-lim, min(lim, sign * k * e))
        buf.append(s); s = buf.pop(0)
        th += (-2.0 * s / W) * dt        # steer s: left wheel v+s, right wheel v-s
        y += v * math.sin(th) * dt
        mx = max(mx, abs(ys)); t += dt
    return abs(ys), mx

def grid(k, sign):
    better = worse = 0
    worst_ctl = worst_str = 0.0
    for y0 in range(-16, 17, 4):
        for th in (-0.1, -0.05, 0.0, 0.05, 0.1):
            for L in (50.0, 80.0, 110.0):
                a, _ = sim(-40, k, sign, L, y0=y0, th0=th)
                b, _ = sim(-40, 0, sign, L, y0=y0, th0=th)
                worst_ctl, worst_str = max(worst_ctl, a), max(worst_str, b)
                if a <= b + 0.5: better += 1
                else: worse += 1
    return better, worse, worst_ctl, worst_str

if __name__ == "__main__":
    b, w, wc, ws = grid(3, +1)
    print("same sign, 3 mm/s per zone: as good or better %d, worse %d of %d; worst final offset %.1f mm "
          "with the control, %.1f mm straight" % (b, w, b + w, wc, ws))
    b2, w2, wc2, ws2 = grid(3, -1)
    print("flipped sign: as good or better %d, worse %d; worst final offset %.1f mm" % (b2, w2, wc2))
    ok = (w > 20) and (wc > ws) and (w2 > w)
    print("S5-H80 %s  line feedback in reverse is not better than straight (sensors ahead of the axle)"
          % ("PASS" if ok else "FAIL"))
    sys.exit(0 if ok else 1)
