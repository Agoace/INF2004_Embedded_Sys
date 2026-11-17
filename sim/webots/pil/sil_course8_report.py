"""Figure-8 SIL run report from sil-result/samples.csv rows (dicts).

Straights / deviation / flips / SEARCH use the definitions of Main_Worker's
feel.py on 20 ms robot rows, so SIL and robot numbers compare:
  straights = LINE rows whose 0.5 s mean yaw rate is below 10 deg/s;
  deviation = |yaw rate - 0.5 s mean|; flips = sign changes of the asked
  wheel difference on straights; SEARCH = LINE -> SEARCH entries.
Here the yaw rate is the simulated body yaw (not wheel speeds).
"""
import math

LINE, SEARCH = 1, 2
LOOSE_DIST_MM = 60.0
LIFT_ON_RAMP_MM = 5.0


def _pct(values, q):
    if not values:
        return None
    v = sorted(values)
    k = (len(v) - 1) * q
    lo, hi = math.floor(k), math.ceil(k)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def straights(rows, step_ms=20, window_ms=500):
    """feel.py style on 20 ms rows: deviations (deg/s) and flips per s."""
    sub = rows[step_ms - 1::step_ms]
    rate = []
    for a, b in zip(sub, sub[1:]):
        d = (b['yaw_rad'] - a['yaw_rad'] + math.pi) % (2 * math.pi) - math.pi
        rate.append((b, math.degrees(d) / (step_ms / 1000)))
    half = window_ms // step_ms // 2
    dev, flips, sign, n = [], 0, 0, 0
    for i, (row, r) in enumerate(rate):
        win = [x for _, x in rate[max(0, i - half):i + half + 1]]
        mean = sum(win) / len(win)
        if row['module'] != LINE or abs(mean) >= 10:
            sign = 0
            continue
        n += 1
        dev.append(abs(r - mean))
        diff = row['desired_right_mm_s'] - row['desired_left_mm_s']
        s = (diff > 0) - (diff < 0)
        if s and sign and s != sign:
            flips += 1
        if s:
            sign = s
    secs = n * step_ms / 1000
    return dict(straight_seconds=secs, deviation_median=_pct(dev, .5),
                deviation_p90=_pct(dev, .9), deviation_p99=_pct(dev, .99),
                flips_per_s=flips / secs if secs else None)


def report(rows, *, loop_mm, crossing_tile, loose_ids, wall_seconds=None):
    out = dict(samples=len(rows), sim_seconds=len(rows) / 1000,
               wall_seconds=wall_seconds, loop_mm=loop_mm)
    if not rows:
        return out
    laps_done = rows[-1]['progress_mm'] / loop_mm
    out['laps_progress'] = laps_done
    out['laps_completed'] = int(laps_done)
    # SEARCH entries, per lap, with the tile under the sensor pair.
    entries = []
    for a, b in zip(rows, rows[1:]):
        if a['module'] == LINE and b['module'] == SEARCH:
            entries.append(dict(ms=b['sim_ms'], lap=int(b['progress_mm'] // loop_mm) + 1,
                                tile=b['tile'], error_mm=round(b['error_mm'], 1)))
    out['search_entries'] = entries
    out['search_per_lap'] = {lap: sum(e['lap'] == lap for e in entries)
                             for lap in range(1, max(2, out['laps_completed'] + 1) + 1)}
    minutes = len(rows) / 60000
    out['search_per_min'] = len(entries) / minutes if minutes else None
    out['searches_counter'] = rows[-1]['searches']
    # Crossing passes: contiguous runs on the crossing tile.
    passes, cur = [], None
    for i, r in enumerate(rows):
        on = r['tile'] == crossing_tile
        if on and cur is None:
            cur = i
        if not on and cur is not None:
            passes.append((cur, i - 1))
            cur = None
    if cur is not None:
        passes.append((cur, len(rows) - 1))
    cross = []
    for a, b in passes:
        seg = rows[a:b + 1]
        after = rows[b + 1:b + 1 + 1500]
        d = (rows[b]['yaw_rad'] - rows[a]['yaw_rad'] + math.pi) % (2 * math.pi) - math.pi
        cross.append(dict(start_ms=rows[a]['sim_ms'], end_ms=rows[b]['sim_ms'],
                          heading_change_deg=round(math.degrees(d), 1),
                          modules=sorted({r['module'] for r in seg}),
                          ja_phases=sorted({r['ja_phase'] for r in seg}),
                          max_error_mm=round(max(abs(r['error_mm']) for r in seg + after), 1),
                          straight=all(r['module'] == LINE for r in seg) and abs(math.degrees(d)) < 15
                          and all(abs(r['error_mm']) < 30 for r in seg + after)))
    out['crossing_passes'] = cross
    # Ramp.
    tilted = [r for r in rows if r['tilted']]
    out['ramp'] = dict(
        tilted_ms=len(tilted),
        roll_peak_deg=round(max(abs(r['roll_deg']) for r in rows), 2),
        right_wheel_lift_peak_mm=round(max(r['right_wheel_lift_mm'] for r in rows), 2),
        left_wheel_lift_peak_mm=round(max(r['left_wheel_lift_mm'] for r in rows), 2))
    lifted = [r for r in rows if r['right_wheel_lift_mm'] >= LIFT_ON_RAMP_MM]
    out['ramp']['right_tyre_on_ramp'] = bool(lifted) and (
        out['ramp']['right_wheel_lift_peak_mm'] > out['ramp']['left_wheel_lift_peak_mm'])
    out['ramp']['right_lift_ms'] = len(lifted)
    out['ramp']['first_lift_ms'] = lifted[0]['sim_ms'] if lifted else None
    # Loose arcs while in LINE.
    loose = [r for r in rows if r['module'] == LINE and
             (r['track_dist_mm'] > LOOSE_DIST_MM or r['under_tile'] in loose_ids)]
    out['loose_arc'] = dict(line_ms_off_track=len(loose),
                            tiles=sorted({r['under_tile'] for r in loose if r['under_tile']}),
                            max_track_dist_mm=round(max(r['track_dist_mm'] for r in rows), 1),
                            followed=bool(loose))
    out['straights'] = straights(rows)
    return out
