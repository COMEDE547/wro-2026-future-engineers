"""Summarise the car's run logs.

    python run_metrics.py <run_logs folder> [summary.csv]

Reads every round2_run_*.csv written by main.py (ENABLE_RUN_CSV_LOGGING = True) and
every parking_run_*.csv written by parallel_parking.py (ENABLE_RUN_LOGGING = True),
writes one summary row per main.py run, and prints the per-day table of full runs
from the parking lot and the parking attempts.
"""
import csv, os, sys, statistics, collections

def num(x):
    try: return float(x)
    except (TypeError, ValueError): return None

def summarise_round2(path):
    rows = list(csv.DictReader(open(path, encoding='utf-8', errors='replace')))
    if not rows: return None
    name = os.path.basename(path)
    ts = [num(r.get('elapsed_s')) for r in rows]; st = [r.get('state', '') for r in rows]
    corners = [num(r.get('corner_counter')) for r in rows]
    seq = [s for i, s in enumerate(st) if s and (i == 0 or s != st[i - 1])]
    t_start = next((t for t, r in zip(ts, rows) if (num(r.get('applied_speed')) or 0) > 0 and t is not None), None)
    t_12 = next((t for t, c in zip(ts, corners) if c is not None and c >= 12 and t is not None), None)
    faults = [((r.get('detail') or '').split('FAULT:', 1)[-1]).strip() for r in rows if 'FAULT' in (r.get('detail') or '')]
    return {'file': name, 'date': name[11:19], 'time': name[20:26], 'start_state': seq[0] if seq else '', 'final_state': st[-1],
            'max_corner': max([c for c in corners if c is not None], default=None), 't_start_s': t_start,
            'time_to_12th_corner_s': round(t_12 - (t_start or 0), 1) if t_12 is not None else None, 'duration_s': ts[-1],
            'pillar_passes': seq.count('CONFIRM_PASSED'), 'fault_reason': faults[-1] if faults else '', 'states': ' > '.join(seq)}

def summarise_parking(path):
    rows = list(csv.DictReader(open(path, encoding='utf-8', errors='replace')))
    if not rows: return None
    return {'file': os.path.basename(path), 'final_state': rows[-1].get('state', ''), 'duration_s': num(rows[-1].get('elapsed_s'))}

def main(folder, out=None):
    names = sorted(os.listdir(folder))
    runs = [x for x in (summarise_round2(os.path.join(folder, f)) for f in names if f.startswith('round2_run_') and f.endswith('.csv')) if x]
    parks = [x for x in (summarise_parking(os.path.join(folder, f)) for f in names if f.startswith('parking_run_') and f.endswith('.csv')) if x]
    if out:
        with open(out, 'w', newline='', encoding='utf-8') as fh:
            w = csv.DictWriter(fh, fieldnames=list(runs[0].keys())); w.writeheader(); w.writerows(runs)
    print('main.py runs: %d; parking attempts: %d' % (len(runs), len(parks)))
    full = [r for r in runs if r['start_state'] == 'PARKING_EXIT_DIRECTION']
    print('Full runs from the parking lot (reached 12 corners; time to the 12th corner, s):')
    for day in sorted({r['date'] for r in full}):
        rs = [r for r in full if r['date'] == day]; done = sorted(r['time_to_12th_corner_s'] for r in rs if r['time_to_12th_corner_s'] is not None)
        print('  %s  runs %3d  reached %2d  %s' % (day, len(rs), len(done), ('median %.1f, %.1f-%.1f' % (statistics.median(done), done[0], done[-1])) if done else '-'))
    if parks:
        ok = sorted(p['duration_s'] for p in parks if p['final_state'] == 'COMPLETE')
        print('Parking: %d of %d COMPLETE; time to COMPLETE median %.1f s (%.1f-%.1f)' % (len(ok), len(parks), statistics.median(ok), ok[0], ok[-1]) if ok else 'Parking: none complete')
    return runs, parks

if __name__ == '__main__':
    if len(sys.argv) < 2: sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
