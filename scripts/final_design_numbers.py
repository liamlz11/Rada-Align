"""Numbers for docs/FINAL_DESIGN.md (Raster Scan + PSD + KLT).

Run: python scripts/final_design_numbers.py

Every input is tagged: [datasheet] (Hamamatsu S5990/S5991 datasheet, read
2026-11-16), [repo] (config.yaml / docs), [handbook] (textbook constant) or
ASSUMPTION. Outputs are [arith] or [sim] (numerical model below).
numpy only.
"""
import math
import numpy as np

DEG = math.pi / 180.0

def h(title):
    print("\n" + "=" * 78 + "\n" + title + "\n" + "=" * 78)

# ---------------------------------------------------------------- inputs
L_PSD_MM = 9.0            # [datasheet] S5991-01 photosensitive area 9 x 9 mm
PSD_RES_UM = 1.5          # [datasheet] position resolution, Io = 1 uA, B = 1 kHz
PSD_ERR_UM = 150.0        # [datasheet] position detection error typ (max 250), 0.2 mm spot
PSD_SENS = 0.6            # [datasheet] A/W at 960 nm
PSD_ISAT_UA = 500.0       # [datasheet] saturation photocurrent, RL = 1 kOhm
PSD_RISE_US = 2.0         # [datasheet]
LAMBDA = 960e-9           # D11: match the PSD peak [datasheet 960 nm]
SIGMA_HEAD_DEG = math.hypot(0.0106, 0.0106)  # [repo] wind jitter + sway per head
DISTANCES = [2.0, 10.0, 12.0, 15.0]

# ---------------------------------------------------------------- 1. spot
h("1. Spot size vs PSD (S5991-01, 9 x 9 mm)")
print(f"head pointing sigma (wind, [repo] assumption) = {SIGMA_HEAD_DEG:.4f} deg")
print("Gaussian beam, best possible collimation for each distance (w0 = sqrt(lambda z / pi)):")
print(f"{'d m':>5} {'w0 mm':>7} {'D1/e2 mm':>9} {'D99 mm':>8} {'lin +/- mm':>10} {'lin +/- deg':>11} {'3sig wind mm':>12}")
for d in DISTANCES:
    w0 = math.sqrt(LAMBDA * d / math.pi)
    wz = math.sqrt(2) * w0
    d_e2 = 2 * wz * 1e3
    d99 = 2 * wz * math.sqrt(-math.log(0.01) / 2) * 1e3
    lin = (L_PSD_MM - d99) / 2
    lin_deg = math.degrees(math.atan(lin / (d * 1e3))) if lin > 0 else float("nan")
    wind3 = d * 1e3 * math.tan(3 * SIGMA_HEAD_DEG * DEG)
    print(f"{d:5.0f} {w0*1e3:7.2f} {d_e2:9.2f} {d99:8.2f} {lin:10.2f} {lin_deg:11.4f} {wind3:12.2f}")
print("Fixed launch beam 3 mm (w0 = 1.5 mm):")
w0 = 1.5e-3
zr = math.pi * w0**2 / LAMBDA
print(f"  Rayleigh range {zr:.2f} m, full divergence {2*LAMBDA/(math.pi*w0)*1e3:.3f} mrad")
for d in DISTANCES:
    wz = w0 * math.sqrt(1 + (d / zr) ** 2)
    print(f"  d {d:4.0f} m: D1/e2 {2*wz*1e3:5.2f} mm, D99 {2*wz*1.517*1e3:5.2f} mm")
print("Geometric model D = D0 + theta*d, D0 = 2 mm (ASSUMPTION), spot must leave")
print("linear range r on the 9 mm PSD: D <= 9 - 2r")
for d in [2.0, 15.0]:
    for label, r in [("fit only, r=0", 0.0), ("r = 3 sigma wind", d * 1e3 * math.tan(3 * SIGMA_HEAD_DEG * DEG))]:
        dmax = L_PSD_MM - 2 * r
        th = (dmax - 2.0) / (d * 1e3) * 1e3
        print(f"  d {d:4.0f} m, {label:18s}: D <= {dmax:6.2f} mm -> theta_full <= {th:7.3f} mrad")
print("Beam FOCUSED at the PSD distance (waist on the PSD), launch radius W at the horn:")
def focused_waist(W, z):
    lo, hi = 1e-7, W
    for _ in range(200):
        w0 = 0.5 * (lo + hi); zr = math.pi * w0**2 / LAMBDA
        if w0 * math.sqrt(1 + (z / zr) ** 2) > W: lo = w0
        else: hi = w0
    return w0
for W in [2.5e-3, 5e-3, 10e-3]:
    for d in DISTANCES:
        w0 = focused_waist(W, d); zr = math.pi * w0**2 / LAMBDA
        d99 = 2 * w0 * 1.517 * 1e3
        wind3 = d * 1e3 * math.tan(3 * SIGMA_HEAD_DEG * DEG)
        print(f"  W {W*1e3:4.1f} mm, d {d:4.0f} m: spot D99 {d99:5.2f} mm, depth of focus +/-{zr:5.2f} m,"
              f" lin +/-{(L_PSD_MM-d99)/2:5.2f} mm = {math.degrees(math.atan((L_PSD_MM-d99)/2/(d*1e3))):.4f} deg,"
              f" PSD for 3 sigma wind >= {2*wind3+d99:5.1f} mm")
print("PSD size needed for +/- 3 sigma wind with the diffraction-limited spot:")
for d in DISTANCES:
    w0 = math.sqrt(LAMBDA * d / math.pi); d99 = 2 * math.sqrt(2) * w0 * 1.517 * 1e3
    wind3 = d * 1e3 * math.tan(3 * SIGMA_HEAD_DEG * DEG)
    print(f"  d {d:4.0f} m: L >= {2*wind3 + d99:6.1f} mm")

# ---------------------------------------------------------------- 2. model
h("2. Observability: 3D model, Jacobian, rank  [sim]")

def Rx(a):
    c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def Ry(a):
    c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def Rz(a):
    c, s = math.cos(a), math.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

GEO = dict(
    horn_lever=np.array([0.10, 0.0, 0.0]),   # [repo] horn_pivot_to_center_m
    radar_lever=np.array([0.10, 0.0, 0.30]), # ASSUMPTION: radar reference 0.3 m above its tilt pivot
    lasers={"A": (-0.15, 0.10), "B": (0.15, 0.10), "C": (0.0, -0.10)},  # ASSUMPTION, horn body [left, up] m
    f_b=0.050,                               # ASSUMPTION PSD-B focal length 50 mm
)
STATE = ["pan_h", "tilt_h", "roll_h", "pan_r", "tilt_r", "roll_r", "ty_h", "tz_h", "ty_r", "tz_r"]

def poses(s, d):
    pan_h, tilt_h, roll_h, pan_r, tilt_r, roll_r, ty_h, tz_h, ty_r, tz_r = s
    Rh = Rz(pan_h) @ Ry(-tilt_h) @ Rx(roll_h)
    Rr = Rz(math.pi + pan_r) @ Ry(-tilt_r) @ Rx(roll_r)
    ph = np.array([0.0, ty_h, tz_h])
    pr = np.array([d, ty_r, tz_r - GEO["radar_lever"][2]])
    hc = ph + Rh @ GEO["horn_lever"]
    ref = pr + Rr @ GEO["radar_lever"]
    return Rh, Rr, ph, hc, ref

def measure(s, d, mis=None):
    """PSD readings [A_y, A_z (mm), B_y, B_z (mm at focal plane), C_y, C_z (mm)]."""
    mis = mis or {}
    Rh, Rr, ph, hc, ref = poses(s, d)
    n = Rr @ np.array([1.0, 0, 0])
    out = []
    for k in ["A", "B", "C"]:
        y, z = GEO["lasers"][k]
        origin = ph + Rh @ np.array([GEO["horn_lever"][0], y, z])
        dy, dz = mis.get(k, (0.0, 0.0))
        dirb = Rh @ np.array([1.0, dy, dz]); dirb /= np.linalg.norm(dirb)
        centre = ref + Rr @ np.array([0.0, -y, z])   # matched pattern, mirrored in radar body
        if k == "B":
            v = Rr.T @ (-dirb)
            out += [GEO["f_b"] * v[1] / v[0] * 1e3, GEO["f_b"] * v[2] / v[0] * 1e3]
        else:
            t = (centre - origin) @ n / (dirb @ n)
            hit = origin + t * dirb - centre
            out += [hit @ (Rr[:, 1]) * 1e3, hit @ (Rr[:, 2]) * 1e3]
    return np.array(out)

def errors(s, d):
    """True [eh_az, eh_el, er_az, er_el, roll_rel] in deg."""
    Rh, Rr, ph, hc, ref = poses(s, d)
    u = (ref - hc) / np.linalg.norm(ref - hc)
    w = Rh.T @ u
    q = Rr.T @ (-u)
    zh = Rh[:, 2] - (Rh[:, 2] @ u) * u
    zr = Rr[:, 2] - (Rr[:, 2] @ u) * u
    roll = math.atan2(np.cross(zr, zh) @ u, zr @ zh)
    return np.degrees([math.atan2(w[1], w[0]), math.atan2(w[2], w[0]),
                       math.atan2(q[1], q[0]), math.atan2(q[2], q[0]), roll])

def jac(fun, s, eps=1e-7):
    f0 = fun(s); J = np.zeros((len(f0), len(s)))
    for i in range(len(s)):
        sp = s.copy(); sm = s.copy(); sp[i] += eps; sm[i] -= eps
        J[:, i] = (fun(sp) - fun(sm)) / (2 * eps)
    return J

for d in [2.0, 12.0]:
    s0 = np.zeros(10)
    print(f"\n-- d = {d} m, nominal aligned state: readings {np.round(measure(s0, d), 9)}, errors {np.round(errors(s0, d), 9)}")
    Jm = jac(lambda s: measure(s, d), s0)
    Je = jac(lambda s: errors(s, d), s0)
    sv = np.linalg.svd(Jm, compute_uv=False)
    rank = int((sv > sv[0] * 1e-9).sum())
    print(f"rank of d(readings)/d(10 states) = {rank}; singular values {np.round(sv, 4)}")
    sv6 = np.linalg.svd(Jm[:, :6], compute_uv=False)
    print(f"rank vs the 6 servo angles only = {int((sv6 > sv6[0]*1e-9).sum())}")
    # errors observable <=> rows of Je in row space of Jm
    X0, *_ = np.linalg.lstsq(Jm.T, Je.T, rcond=None)
    resid = np.abs(Jm.T @ X0 - Je.T).max()
    print(f"max residual of errors projected on readings = {resid:.2e} (0 => all 5 errors observable)")
    G = Jm @ np.linalg.pinv(Je)          # readings per unit error (mm per deg)
    X = np.linalg.pinv(G).T              # least-squares estimator over the 6 readings
    names = ["A_y", "A_z", "B_y", "B_z", "C_y", "C_z"]
    print("  readings per error, mm per deg (columns eh_az, eh_el, er_az, er_el, roll):")
    for i in range(6):
        print(f"    {names[i]}: " + " ".join(f"{G[i, j]:+10.4f}" for j in range(5)))
    for j, en in enumerate(["eh_az", "eh_el", "er_az", "er_el", "roll"]):
        terms = " ".join(f"{X[i, j]:+.6f}*{names[i]}" for i in range(6) if abs(X[i, j]) > 1e-9)
        print(f"  LS estimator {en:6s} [deg] = {terms}")
    _, _, Vt = np.linalg.svd(Jm)
    N = Vt[rank:].T
    print(f"null space dim = {N.shape[1]}; max |d errors| along it = {np.abs(Je @ N).max():.2e}")
    for k in range(N.shape[1]):
        v = N[:, k]; top = np.argsort(-np.abs(v))[:4]
        print("  null vector: " + ", ".join(f"{STATE[i]} {v[i]:+.3f}" for i in top))
    for sub, rows in [("A only", [0, 1]), ("B only", [2, 3]), ("A+B", [0, 1, 2, 3])]:
        Xs, *_ = np.linalg.lstsq(Jm[rows].T, Je.T, rcond=None)
        r = np.abs(Jm[rows].T @ Xs - Je.T).max(axis=0)
        print(f"  {sub:7s}: residual per error (eh_az, eh_el, er_az, er_el, roll) = {np.round(r, 4)}")
    # nonlinear check of the linear estimator near alignment and at setup size
    rng = np.random.default_rng(0)
    for lab, ang, tr in [("near aligned +/-0.1 deg, +/-10 mm", 0.1, 0.010), ("setup +/-1 deg, +/-250 mm", 1.0, 0.25)]:
        worst = np.zeros(5)
        for _ in range(2000):
            s = np.concatenate([rng.uniform(-ang, ang, 6) * DEG, rng.uniform(-tr, tr, 4)])
            if lab.startswith("near"):
                s[6:] = rng.uniform(-tr, tr, 4)
            est = X.T @ measure(s, d)
            worst = np.maximum(worst, np.abs(est - errors(s, d)))
        print(f"  linear estimator, {lab}: worst |est - true| deg = {np.round(worst, 5)}")

# ---- alignment by rotations only (conflict 2)
print("\n-- Rotations only: solve the 6 servo angles for errors = 0 and radar roll level")

def level_roll(s, d):
    Rh, Rr, *_ = poses(s, d)
    return math.degrees(math.asin(Rr[2, 1]))  # radar body y axis vs horizontal

def goal(j6, rest, d):
    s = np.concatenate([j6, rest]); return np.concatenate([errors(s, d), [level_roll(s, d)]])

rng = np.random.default_rng(1)
for d, off in [(2.0, 0.05), (2.0, 0.25), (10.0, 0.25), (12.0, 0.25), (15.0, 0.25)]:
    worst_j = 0.0; worst_tilt = 0.0; worst_read = 0.0; worst_err = 0.0
    for _ in range(300):
        j = rng.uniform(-1, 1, 6) * DEG; j[2] = rng.uniform(-0.5, 0.5) * DEG; j[5] = rng.uniform(-0.5, 0.5) * DEG
        rest = np.concatenate([rng.uniform(-off, off, 2), [0.0, 0.0]])  # relative position error on the horn side
        for _it in range(20):
            g = goal(j, rest, d)
            if np.abs(g).max() < 1e-10: break
            J = jac(lambda x: goal(x, rest, d), j)
            j = j - np.linalg.solve(J, g) * 1.0
        s = np.concatenate([j, rest])
        worst_j = max(worst_j, np.abs(np.degrees(j)).max())
        Rh, Rr, *_ = poses(s, d)
        worst_tilt = max(worst_tilt, abs(math.degrees(math.asin(Rr[2, 0]))))
        worst_read = max(worst_read, np.abs(measure(s, d)).max())
        worst_err = max(worst_err, np.abs(errors(s, d)).max())
    print(f"  d {d:4.0f} m, offset +/-{off*1e3:.0f} mm: max |servo angle| {worst_j:5.2f} deg, radar boresight off level up to {worst_tilt:4.2f} deg,"
          f" max |PSD reading| at solution {worst_read:.1e} mm, max |error| {worst_err:.1e} deg")
print("  range needed (arith): 1 deg setup + atan(0.25/d):",
      ", ".join(f"{d:.0f} m {1+math.degrees(math.atan(0.25/d)):.2f} deg" for d in [10, 12, 15]))

# ---- simple decoupled loop with lever arms
print("\n-- Decoupled loop (gain 1, estimator above, inverse of d(errors)/d(servos) at nominal)")
for d in [2.0, 12.0]:
    s0 = np.zeros(10)
    Jm = jac(lambda s: measure(s, d), s0); Je = jac(lambda s: errors(s, d), s0)
    X = np.linalg.pinv(Jm @ np.linalg.pinv(Je)).T
    Jj = np.vstack([Je[:, :6], jac(lambda s: np.array([level_roll(s, d)]), s0)[:, :6]])
    its = []
    for _ in range(200):
        s = np.concatenate([rng.uniform(-1, 1, 6) * DEG, rng.uniform(-0.25, 0.25, 2), [0.0, 0.0]])
        for it in range(1, 50):
            e = np.concatenate([X.T @ measure(s, d), [level_roll(s, d)]])
            if np.abs(errors(s, d)[:4]).max() < 0.001: break
            s[:6] -= np.linalg.solve(Jj, e)
        its.append(it)
    print(f"  d {d:4.0f} m: iterations to |errors| < 0.001 deg: median {int(np.median(its))}, max {max(its)}")

# ---- laser misalignment propagation
print("\n-- Laser misalignment 0.01 deg on one laser at a time (aligned state, d = 12 m)")
d = 12.0; s0 = np.zeros(10)
Jm = jac(lambda s: measure(s, d), s0); Je = jac(lambda s: errors(s, d), s0)
X = np.linalg.pinv(Jm @ np.linalg.pinv(Je)).T
for k in ["A", "B", "C"]:
    for ax, mis in [("az", (0.01 * DEG, 0.0)), ("el", (0.0, 0.01 * DEG))]:
        est = X.T @ measure(s0, d, {k: mis})
        print(f"  laser {k} {ax}: estimated errors (eh_az, eh_el, er_az, er_el, roll) = {np.round(est, 4)}")
est = X.T @ measure(s0, d, {k: (0.01 * DEG, 0.0) for k in "ABC"})
print(f"  whole board 0.01 deg az (common mode): {np.round(est, 4)}")

# ---------------------------------------------------------------- 3. raster
h("3. Raster scan with the horn Pan/Tilt")
SCAN_SPEED_MAX = 2.0     # [repo] pan/tilt max_speed 2 deg/s (ASSUMPTION for servos)
F_OUT = 1000.0           # ASSUMPTION lock-in output rate 1 kHz
N_CROSS = 10             # DESIGN: samples while the spot crosses one PSD width
TURN_S = 0.2             # ASSUMPTION line turnaround
print(f"step = PSD width L = 9 mm at the PSD (spot centre always passes over the active area)")
print(f"{'d m':>4} {'step deg':>9} {'speed deg/s':>11} " + " ".join(f"{'R='+str(r):>10}" for r in [0.05, 0.1, 0.25, 0.5, 1.0, 2.43]))
for d in DISTANCES:
    step = math.degrees(math.atan(L_PSD_MM / (d * 1e3)))
    speed = min(SCAN_SPEED_MAX, step * F_OUT / N_CROSS)
    row = []
    for R in [0.05, 0.1, 0.25, 0.5, 1.0, 2.43]:
        lines = math.ceil(2 * R / step) + 1
        t = (lines * 2 * R + (lines - 1) * step) / speed + (lines - 1) * TURN_S
        row.append(f"{t:9.1f}s")
    print(f"{d:4.0f} {step:9.4f} {speed:11.2f} " + " ".join(f"{x:>10}" for x in row))
print("Capture of the other two spots once A is on its PSD:")
for d in DISTANCES:
    w0 = math.sqrt(LAMBDA * d / math.pi); d99 = 2 * math.sqrt(2) * w0 * 1.517 * 1e3
    free = max(0.0, (L_PSD_MM - d99) / 2)
    b = math.hypot(0.15, 0.20) * 1e3
    print(f"  d {d:4.0f} m: C on its PSD if |relative roll| < {math.degrees(free / b):.3f} deg (baseline {b:.0f} mm, spot D99 {d99:.1f} mm)")
print(f"  PSD-B angular field (f 50 mm, ASSUMPTION): +/- {math.degrees(math.atan(L_PSD_MM/2/50)):.2f} deg;"
      f" spatial capture = lens aperture 25 mm (ASSUMPTION) = +/- {math.degrees(math.atan(12.5/12000)):.3f} deg at 12 m")

# ---------------------------------------------------------------- 4. thermal
h("4. Laser board thermal stability  [arith]")
CTE = {"aluminium": 23e-6, "steel": 12e-6, "invar": 1.2e-6}  # [handbook]
print("board 200 mm long, 10 mm thick (ASSUMPTION); through-thickness gradient bends it:")
print("theta = CTE * dT * L / t")
for m, a in CTE.items():
    for dT in [0.5, 1.0, 2.0]:
        th = a * dT * 0.200 / 0.010
        print(f"  {m:9s} dT {dT:3.1f} K: {math.degrees(th):.4f} deg")
print("uniform +20 K, aluminium: spacing change "
      f"{23e-6*20*0.2*1e3:.3f} mm -> {math.degrees(math.atan(23e-6*20*0.2/12)):.5f} deg at 12 m")

# ---------------------------------------------------------------- 5. servo
h("5. Encoder, gearing, torque  [arith]")
for res in [0.005, 0.001]:
    c = 360 / res
    print(f"  {res} deg per count at the output: {c:,.0f} counts/rev = {math.log2(c):.1f} bit")
for bits, ratio in [(17, 1), (17, 50), (20, 1), (23, 1)]:
    print(f"  {bits}-bit motor encoder, gear {ratio}:1 -> {360/(2**bits)/ratio:.6f} deg per count at the output")
print(f"  one 0.005 deg count = {12000*math.tan(0.005*DEG):.2f} mm on PSD-A at 12 m; PSD resolves {PSD_RES_UM} um")
for r in [0.03, 0.05, 0.10]:
    T = 35 * 9.81 * r
    print(f"  35 kg [mentor], CG {r*100:.0f} cm off the tilt/roll axis: {T:5.1f} N m; via 100:1 at 35% (ASSUMPTION) {T/100/0.35:.2f} N m at the motor")
I = 35 * (0.5**2 + 0.3**2) / 12  # ASSUMPTION 0.5 x 0.5 x 0.3 m box
print(f"  inertia (box 0.5x0.5x0.3 m, ASSUMPTION) {I:.2f} kg m^2; 1 deg/s^2 needs {I*DEG:.3f} N m")

# ---------------------------------------------------------------- 6. PSD noise
h("6. PSD noise, ADC, modulation  [arith from datasheet]")
for d in [2.0, 12.0, 15.0]:
    print(f"  PSD-A at {d:4.0f} m: 1.5 um = {math.degrees(PSD_RES_UM*1e-6/d):.2e} deg;"
          f" detection error 150 um = {math.degrees(PSD_ERR_UM*1e-6/d):.4f} deg")
for f in [0.050, 0.100]:
    print(f"  PSD-B f {f*1e3:.0f} mm: 0.05 deg = {f*math.tan(0.05*DEG)*1e6:.1f} um; 1.5 um = {math.degrees(1.5e-6/f):.5f} deg;"
          f" detection error 150 um = {math.degrees(150e-6/f):.3f} deg (cancels at a calibrated null point)")
for I_ua, B in [(1, 1000), (50, 100), (100, 100)]:
    res = PSD_RES_UM * (1 / I_ua) * math.sqrt(B / 1000)
    print(f"  Io {I_ua:3d} uA, B {B:4d} Hz: resolution ~ {res:.4f} um (scaled 1/Io, sqrt(B); ASSUMPTION scaling) ->"
          f" PSD-B f 50 mm {math.degrees(res*1e-6/0.05):.2e} deg")
for bits in [12, 16]:
    q = L_PSD_MM / 2 / 2**(bits - 1) * 1e3
    print(f"  {bits}-bit ADC on the electrode currents: ~{q:.3f} um -> PSD-B f 50 mm {math.degrees(q*1e-6/0.05):.5f} deg,"
          f" PSD-A 12 m {math.degrees(q*1e-6/12):.2e} deg")
P_mw, T_filter = 1.0, 0.9  # ASSUMPTION
I_sig = P_mw * 1e-3 * T_filter * PSD_SENS * 1e6
print(f"  laser 1.0 mW (ASSUMPTION), filter 90%: photocurrent {I_sig:.0f} uA vs saturation {PSD_ISAT_UA:.0f} uA [datasheet]")
E_sun = 0.6  # ASSUMPTION W/m^2/nm direct sun near 960 nm
for bw in [10, 3]:
    I_bg = E_sun * bw * (L_PSD_MM * 1e-3) ** 2 * PSD_SENS * 1e6
    print(f"  direct sun into the PSD, {bw:2d} nm filter: {I_bg:.0f} uA DC (needs hood + AC-coupled front end)")
print(f"  PSD bandwidth ~ 0.35/tr = {0.35/(PSD_RISE_US*1e-6)/1e3:.0f} kHz [arith from datasheet rise time];"
      " modulation 10 kHz (ASSUMPTION), 4 samples per cycle -> 40 kS/s x 12 channels = 480 kS/s")
C4 = 10 ** (0.002 * (960 - 700))
print(f"  Class 1 AEL at 960 nm ~ 0.39 mW x C4, C4 = 10^(0.002(lambda-700)) = {C4:.2f} -> {0.39*C4:.2f} mW [unverified recall of IEC 60825-1]")

# ---------------------------------------------------------------- 7. budget
h("7. Error budget per error, 1 sigma, RSS  [ASSUMPTION rows]")
radar = {
    "PSD-B optical axis to radar datum (calibration)": 0.015,
    "PSD unit seat repeatability on the datum": 0.010,
    "wind and tripod settling (real motion)": 0.015,
    "laser A vs laser B parallelism (differential)": 0.005,
    "board bending, differential (invar, 1 K)": 0.0014,
    "turbulence / heat shimmer": 0.003,
    "servo residual inside deadband": 0.005,
    "PSD-B noise (f 50 mm, 1 uA, 1 kHz)": 0.0017,
}
horn = {
    "laser A to horn axis parallelism (lab CALIB)": 0.010,
    "PSD-A position vs radar reference, 0.5 mm at 12 m": math.degrees(math.atan(0.5 / 12000)),
    "wind and tripod settling (real motion)": 0.015,
    "board bending, common (invar, 1 K)": 0.0014,
    "turbulence / heat shimmer": 0.003,
    "servo residual inside deadband": 0.005,
    "PSD-A noise and detection error at the null": 0.0001,
}
for name, rows in [("radar error", radar), ("horn error", horn)]:
    print(f"-- {name}")
    for k, v in rows.items():
        print(f"  {k:52s} {v:.4f}")
    rss = math.sqrt(sum(v * v for v in rows.values()))
    print(f"  {'RSS':52s} {rss:.4f}  -> 2 sigma {2*rss:.4f} vs 0.05")
old = [0.015, 0.015, 0.010, 0.005, 0.005, 0.003, 0.002, 0.002]
print(f"previous camera budget RSS [repo CHALLENGE.md rows] = {math.sqrt(sum(v*v for v in old)):.4f}")

# ---------------------------------------------------------------- BOM sums
h("BOM sums (D10 ranges)  [arith]")
lo = [1800, 2000, 1000, 1500, 300, 60, 300, 600, 150, 50, 500, 300, 300]
hi = [9000, 8000, 4000, 5000, 2700, 2400, 3000, 3000, 1000, 500, 2000, 2000, 1500]
print(f"  low {sum(lo):,}  high {sum(hi):,}  high without item 7 (scan = horn servos) {sum(hi)-3000:,}")

# ---------------------------------------------------------------- extras
h("Extra numbers quoted in docs/FINAL_DESIGN.md  [arith]")
for L, d in [(9.0, 12.0), (9.0, 15.0), (20.0, 12.0), (20.0, 15.0)]:
    w0 = focused_waist(5e-3, d); d99 = 2 * w0 * 1.517 * 1e3
    free = (L - d99) / 2
    ang = math.degrees(math.atan(free / (d * 1e3)))
    roll = math.degrees(math.atan(free / (math.hypot(0.15, 0.20) * 1e3)))
    print(f"  PSD {L:4.0f} mm, d {d:4.0f} m, focused W 5 mm: linear +/-{free:.2f} mm = +/-{ang:.4f} deg"
          f" = {ang/SIGMA_HEAD_DEG:.2f} sigma wind; C captured if |relative roll| < {roll:.2f} deg")
print(f"  camera 1.55 um / 25 mm at 12 m: {1.55e-6/0.025*12.0*1e3:.3f} mm per px")
print(f"  ACQUIRE window +/-3 x 0.05 deg = +/-{3*0.05:.2f} deg")
print(f"  BOM items 1+2+4 high: {9000+8000+5000:,}; high end over the 30k budget by {sum(hi)-30000:,}")
print(f"  PSD-A scale at 12 m: {(12.0-0.2)*1e3*math.pi/180:.1f} mm/deg (11.8 m laser-to-PSD path, lever ASSUMPTION);"
      f" d*pi/180 at 12.0 m = {12000*math.pi/180:.1f} mm/deg")
