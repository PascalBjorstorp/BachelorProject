#!/usr/bin/env python3
"""Pick the best spawn waypoint from a 9-column MPCC trajectory CSV.

Selects the waypoint with the highest look-ahead speed limit (max speed before
any corner in the next 7m), breaking ties by minimum wall clearance.  Waypoints
with clearance < 0.5m on either side are excluded.  This avoids spawning inside
or immediately before a tight hairpin where the MPCC speed limit is already low.
"""
import sys
import math


def main():
    if len(sys.argv) < 2:
        sys.exit(1)

    rows = []
    with open(sys.argv[1]) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = [x.strip() for x in line.split(',')]
            if len(p) >= 9:
                try:
                    rows.append({
                        's': float(p[0]), 'x': float(p[1]), 'y': float(p[2]),
                        'psi': float(p[3]), 'kappa': float(p[4]),
                        'left': float(p[7]), 'right': float(p[8]),
                    })
                except ValueError:
                    pass

    if not rows:
        sys.exit(1)

    total_s = rows[-1]['s']

    def interp_kappa(s):
        s_mod = s % total_s
        lo, hi = 0, len(rows) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if rows[mid]['s'] <= s_mod:
                lo = mid
            else:
                hi = mid
        denom = max(rows[hi]['s'] - rows[lo]['s'], 1e-9)
        t = (s_mod - rows[lo]['s']) / denom
        return rows[lo]['kappa'] + t * (rows[hi]['kappa'] - rows[lo]['kappa'])

    def speed_limit(s, lookahead=7.0, n=20):
        ax_b = 9.0 * 0.75
        cs = 0.85
        mu_g = 0.745 * 9.81
        thresh = 0.20
        v = 20.0
        ds = lookahead / n
        for i in range(1, n + 1):
            d = ds * i
            k = abs(interp_kappa(s + d))
            if k > thresh:
                vc = cs * math.sqrt(mu_g / k)
                vb = math.sqrt(vc * vc + 2.0 * ax_b * d)
                if vb < v:
                    v = vb
        return v

    best = None
    for r in rows:
        cl = min(r['left'], r['right'])
        if cl < 0.5:
            continue
        sl = speed_limit(r['s'])
        if best is None or sl > best[0] or (sl == best[0] and cl > best[1]):
            best = (sl, cl, r['x'], r['y'], r['psi'])

    if best:
        print(f"{best[2]} {best[3]} {best[4]}")
    else:
        r = rows[0]
        print(f"{r['x']} {r['y']} {r['psi']}")


if __name__ == '__main__':
    main()
