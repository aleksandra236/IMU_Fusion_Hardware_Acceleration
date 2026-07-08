#!/usr/bin/env python3
"""
spec_vs_vp.py  –  Poređenje C speca (golden model) vs VP (sc_fixed)

Ulazi:
  spec_csv  : orientation_1504.csv  (time,qw,qx,qy,qz,roll,pitch,yaw)
  vp_csv    : vp_output.csv         (sample,qw,qx,qy,qz)

Oba fajla moraju imati isti broj uzoraka (1918).
"""

import csv, math, sys

SPEC_CSV = "../spec/orientation_1504.csv"
VP_CSV   = "vp_output.csv"

def load_spec(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append({
                'qw': float(r['qw']), 'qx': float(r['qx']),
                'qy': float(r['qy']), 'qz': float(r['qz']),
                'roll': float(r['roll']), 'pitch': float(r['pitch']),
                'yaw': float(r['yaw']),
            })
    return rows

def load_vp(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append({
                'qw': float(r['qw']), 'qx': float(r['qx']),
                'qy': float(r['qy']), 'qz': float(r['qz']),
            })
    return rows

def quat_to_euler(qw, qx, qy, qz):
    roll  = math.degrees(math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy)))
    sp    = 2*(qw*qy - qz*qx)
    sp    = max(-1.0, min(1.0, sp))
    pitch = math.degrees(math.asin(sp))
    yaw   = math.degrees(math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz)))
    return roll, pitch, yaw

def angle_err(a, b):
    e = abs(a - b)
    return min(e, 360 - e)

def l2(s, v):
    return math.sqrt((s['qw']-v['qw'])**2 + (s['qx']-v['qx'])**2 +
                     (s['qy']-v['qy'])**2 + (s['qz']-v['qz'])**2)

# ── Load ─────────────────────────────────────────────────────────────────────
spec = load_spec(SPEC_CSV)
vp   = load_vp(VP_CSV)

n = min(len(spec), len(vp))
print(f"Ucitano: spec={len(spec)} uzoraka, vp={len(vp)} uzoraka → poredimo {n}\n")

# ── Compare ──────────────────────────────────────────────────────────────────
sum_l2 = sum_roll = sum_pitch = sum_yaw = 0.0
max_l2 = max_roll = max_pitch = max_yaw = 0.0

print(f"{'Uzor':>5}  {'Roll_S':>10}{'Roll_VP':>10}  {'Pitch_S':>10}{'Pitch_VP':>10}  "
      f"{'Yaw_S':>10}{'Yaw_VP':>10}  {'dRoll':>8}{'dPitch':>8}{'dYaw':>8}")
print("-" * 102)

results = []
for i in range(n):
    s = spec[i]
    v = vp[i]
    sr, sp_a, sy = s['roll'], s['pitch'], s['yaw']
    vr, vp_a, vy = quat_to_euler(v['qw'], v['qx'], v['qy'], v['qz'])

    eq = l2(s, v)
    er = angle_err(sr, vr)
    ep = angle_err(sp_a, vp_a)
    ey = angle_err(sy, vy)

    sum_l2   += eq; max_l2   = max(max_l2, eq)
    sum_roll += er; max_roll = max(max_roll, er)
    sum_pitch+= ep; max_pitch= max(max_pitch, ep)
    sum_yaw  += ey; max_yaw  = max(max_yaw, ey)

    results.append((i, sr,sp_a,sy, vr,vp_a,vy, er,ep,ey, eq))
    if i % 100 == 99:
        print(f"{i:>5}  {sr:+10.3f}{vr:+10.3f}  {sp_a:+10.3f}{vp_a:+10.3f}  "
              f"{sy:+10.3f}{vy:+10.3f}  {er:8.4f}{ep:8.4f}{ey:8.4f}")

# ── Summary ──────────────────────────────────────────────────────────────────
print()
print("=" * 62)
print(f"  Rezime  (C spec golden vs VP sc_fixed,  {n} uzoraka)")
print("=" * 62)
print(f"  Kvaternion L2 greska:  srednja = {sum_l2/n:.6f}   maks = {max_l2:.6f}")
print(f"  Roll  greska (stepen): srednja = {sum_roll/n:.4f}    maks = {max_roll:.4f}")
print(f"  Pitch greska (stepen): srednja = {sum_pitch/n:.4f}    maks = {max_pitch:.4f}")
print(f"  Yaw   greska (stepen): srednja = {sum_yaw/n:.4f}    maks = {max_yaw:.4f}")

last = results[-1]
print(f"\n  Finalna orijentacija (uzorak {n-1}):")
print(f"    {'':10}  {'Roll':>10}  {'Pitch':>10}  {'Yaw':>10}")
print(f"    {'C spec':10}  {last[1]:+10.3f}  {last[2]:+10.3f}  {last[3]:+10.3f}")
print(f"    {'VP sc_fixed':10}  {last[4]:+10.3f}  {last[5]:+10.3f}  {last[6]:+10.3f}")
print(f"    {'greska':10}  {last[4]-last[1]:+10.3f}  {last[5]-last[2]:+10.3f}  {last[6]-last[3]:+10.3f}")
print("=" * 62)

# ── Write CSV ─────────────────────────────────────────────────────────────────
out_path = "spec_vs_vp_results.csv"
with open(out_path, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['sample','spec_roll','spec_pitch','spec_yaw',
                'vp_roll','vp_pitch','vp_yaw',
                'err_roll','err_pitch','err_yaw','q_l2'])
    for r in results:
        w.writerow([r[0], f"{r[1]:.4f}", f"{r[2]:.4f}", f"{r[3]:.4f}",
                    f"{r[4]:.4f}", f"{r[5]:.4f}", f"{r[6]:.4f}",
                    f"{r[7]:.4f}", f"{r[8]:.4f}", f"{r[9]:.4f}", f"{r[10]:.6f}"])
print(f"\nDetaljni rezultati upisani u {out_path}")
