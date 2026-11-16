"""3D geometry of the final design (docs/FINAL_DESIGN.md sections 2-4, 7.2).

Same model as scripts/final_design_numbers.py, parameterised from config.

Frames: world x from the horn towards the radar, y left, z up. Each side has a
body frame [forward, left, up]; the radar's forward points back at the horn.
Attitude vector (6, rad): horn pan, tilt, roll, radar pan, tilt, roll.
Translation vector (4, m): horn y, z, radar y, z (manual setup position).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .config import PsdConfig

DEG = math.pi / 180.0
PSDS = ("A", "B", "C")
READING_NAMES = ("A_y", "A_z", "B_y", "B_z", "C_y", "C_z")
ERROR_NAMES = ("eh_az", "eh_el", "er_az", "er_el", "roll")
SERVO_NAMES = ("horn_pan", "horn_tilt", "horn_roll", "radar_pan", "radar_tilt", "radar_roll")


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def focused_spot_radius99_mm(launch_radius_mm: float, distance_m: float, wavelength_nm: float) -> float:
    """99 % power radius of a Gaussian beam focused at `distance_m` (FINAL_DESIGN 7.1)."""
    lam = wavelength_nm * 1e-9
    W = launch_radius_mm * 1e-3
    lo, hi = 1e-7, W
    for _ in range(100):
        w0 = 0.5 * (lo + hi)
        zr = math.pi * w0 * w0 / lam
        if w0 * math.sqrt(1 + (distance_m / zr) ** 2) > W:
            lo = w0
        else:
            hi = w0
    return 1.517 * w0 * 1e3


def unit_sigmas_deg(cfg: PsdConfig, emitter: str) -> tuple[float, float]:
    """1-sigma calibration residual of (laser board, PSD-B null), by mounting point.

    The unit on the radar datum carries the datum transfer and the seat; the unit
    on the horn carries its own axis error (config.yaml psd.calib)."""
    c = cfg.calib
    radar_unit = math.hypot(c.radar_datum_sigma_deg, c.radar_seat_sigma_deg)
    horn_unit = c.horn_axis_sigma_deg
    return (radar_unit, horn_unit) if emitter == "radar" else (horn_unit, radar_unit)


@dataclass
class Biases:
    """Hidden truth the controller does not know (calibration residuals)."""

    laser_dir: dict = field(default_factory=lambda: {k: (0.0, 0.0) for k in PSDS})  # rad, horn body (left, up)
    b_null: tuple = (0.0, 0.0)  # rad, PSD-B optical axis vs radar datum
    psd_pos_mm: dict = field(default_factory=lambda: {k: (0.0, 0.0) for k in PSDS})


class Geometry:
    def __init__(self, cfg: PsdConfig, distance_m: float, height_m: float = 0.0):
        """`height_m`: radar reference above the horn centre (nominal). The truth
        gets it from the scenario; the controller's nominal model from its sensors."""
        g = cfg.geometry
        self.cfg = cfg
        self.d = float(distance_m)
        self.h = float(height_m)
        self.horn_lever = np.array(g.horn_lever_m, dtype=float)
        self.radar_lever = np.array(g.radar_lever_m, dtype=float)
        self.lasers = {"A": g.laser_a_m, "B": g.laser_b_m, "C": g.laser_c_m}
        self.f_b = cfg.sensor.b_focal_mm * 1e-3
        if g.emitter not in ("radar", "horn"):
            raise ValueError(f"psd.geometry.emitter must be 'radar' or 'horn', got {g.emitter!r}")
        self.emitter = g.emitter
        self.spot_r99_mm = focused_spot_radius99_mm(
            cfg.laser.launch_radius_mm, self.d, cfg.laser.wavelength_nm
        )
        self.att0 = self._aligned_attitude()

    def _aligned_attitude(self) -> np.ndarray:
        """Attitude (rad) at which both boresights lie on the line of sight for
        zero setup translation: horn tilted up by the line elevation, radar down."""
        att = np.zeros(6)
        if self.h == 0.0:
            return att
        z4 = np.zeros(4)
        for _ in range(40):
            e = self.errors(att, z4) * DEG
            att[1] += e[1]
            att[4] += e[3]
        return att

    # ------------------------------------------------------------ poses
    def poses(self, att, trans):
        ph, th, rh, pr, tr, rr = att
        Rh = Rz(ph) @ Ry(-th) @ Rx(rh)
        Rr = Rz(math.pi + pr) @ Ry(-tr) @ Rx(rr)
        piv_h = np.array([0.0, trans[0], trans[1]])
        piv_r = np.array([self.d, trans[2], self.h + trans[3] - self.radar_lever[2]])
        hc = piv_h + Rh @ self.horn_lever
        ref = piv_r + Rr @ self.radar_lever
        return Rh, Rr, piv_h, hc, ref

    # ------------------------------------------------------------ readings
    def beams(self, att, trans, bias: Biases | None = None) -> dict:
        """Where each beam meets its PSD.

        A, C: hit position on the PSD plane, PSD local [y, z] in mm.
        B: focal-plane position in mm, plus `B_lens` = hit position on the lens (mm).
        """
        bias = bias or Biases()
        Rh, Rr, ph, hc, ref = self.poses(att, trans)
        if self.emitter == "radar":
            RE, refE, RT, refT = Rr, ref, Rh, hc
        else:
            RE, refE, RT, refT = Rh, hc, Rr, ref
        n = RT @ np.array([1.0, 0, 0])
        out = {}
        for k in PSDS:
            y, z = self.lasers[k]
            origin = refE + RE @ np.array([0.0, y, z])
            dy, dz = bias.laser_dir[k]
            dirb = RE @ np.array([1.0, dy, dz])
            dirb /= np.linalg.norm(dirb)
            centre = refT + RT @ np.array([0.0, -y, z])
            t = (centre - origin) @ n / (dirb @ n)
            hit = origin + t * dirb - centre
            pos = np.array([hit @ RT[:, 1], hit @ RT[:, 2]]) * 1e3 - np.array(bias.psd_pos_mm[k])
            if k == "B":
                v = RT.T @ (-dirb)
                ay = v[1] / v[0] - math.tan(bias.b_null[0])
                az = v[2] / v[0] - math.tan(bias.b_null[1])
                out["B"] = np.array([self.f_b * ay, self.f_b * az]) * 1e3
                out["B_lens"] = pos
            else:
                out[k] = pos
        return out

    def readings_vector(self, att, trans, bias=None) -> np.ndarray:
        b = self.beams(att, trans, bias)
        return np.concatenate([b["A"], b["B"], b["C"]])

    # ------------------------------------------------------------ truth
    def errors(self, att, trans) -> np.ndarray:
        """True [eh_az, eh_el, er_az, er_el, relative roll] in degrees."""
        Rh, Rr, ph, hc, ref = self.poses(att, trans)
        u = (ref - hc) / np.linalg.norm(ref - hc)
        w = Rh.T @ u
        q = Rr.T @ (-u)
        zh = Rh[:, 2] - (Rh[:, 2] @ u) * u
        zr = Rr[:, 2] - (Rr[:, 2] @ u) * u
        roll = math.atan2(np.cross(zr, zh) @ u, zr @ zh)
        return np.degrees([
            math.atan2(w[1], w[0]), math.atan2(w[2], w[0]),
            math.atan2(q[1], q[0]), math.atan2(q[2], q[0]), roll,
        ])

    def gravity(self, att) -> dict:
        """What ideal inclinometers see, in degrees: tilt of each boresight vs the
        horizon and roll of each body y axis vs the horizon."""
        Rh, Rr, *_ = self.poses(att, np.zeros(4))
        return {
            "horn_tilt": math.degrees(math.asin(Rh[2, 0])),
            "horn_roll": math.degrees(math.asin(Rh[2, 1])),
            "radar_tilt": math.degrees(math.asin(Rr[2, 0])),
            "radar_roll": math.degrees(math.asin(Rr[2, 1])),
        }

    def line_elevation_deg(self, trans, att=None) -> float:
        """Elevation of the horn-centre -> radar-reference line (R3)."""
        _, _, _, hc, ref = self.poses(self.att0 if att is None else att, trans)
        v = ref - hc
        return math.degrees(math.atan2(v[2], math.hypot(v[0], v[1])))

    def height_mm(self, trans, att=None) -> float:
        """Radar reference above the horn centre, in mm (R3 truth)."""
        _, _, _, hc, ref = self.poses(self.att0 if att is None else att, trans)
        return float(ref[2] - hc[2]) * 1e3

    # ------------------------------------------------------------ linearisation
    def jacobians(self, eps: float = 1e-7) -> dict:
        """Linear maps at the nominal aligned state (att0), used by the estimator/controller.

        G:  readings (mm) per error (deg), 6 x 5
        E:  least-squares estimator, errors = E @ readings, 5 x 6
        Jj: d[errors, radar level roll]/d(servo angles in deg), 6 x 6
        """
        s0 = np.concatenate([self.att0, np.zeros(4)])

        def rd(s):
            return self.readings_vector(s[:6], s[6:])

        def er(s):
            return self.errors(s[:6], s[6:])

        def jac(fun, s):
            f0 = fun(s)
            J = np.zeros((len(f0), len(s)))
            for i in range(len(s)):
                sp, sm = s.copy(), s.copy()
                sp[i] += eps
                sm[i] -= eps
                J[:, i] = (fun(sp) - fun(sm)) / (2 * eps)
            return J

        Jm = jac(rd, s0)
        Je = jac(er, s0)
        G = Jm @ np.linalg.pinv(Je)
        Jlev = jac(lambda s: np.array([self.gravity(s[:6])["radar_roll"]]), s0)
        Jj = np.vstack([Je[:, :6], Jlev[:, :6]]) * DEG  # per degree of servo angle
        # relative roll (error 4) from the two inclinometer rolls:
        # roll = c_h * horn_roll + c_r * radar_roll, by the roll servo of each side
        Jhr = jac(lambda s: np.array([self.gravity(s[:6])["horn_roll"]]), s0)
        roll_incl = (Je[4, 2] / Jhr[0, 2], Je[4, 5] / Jlev[0, 5])
        return {"Jm": Jm, "Je": Je, "G": G, "E": np.linalg.pinv(G), "Jj": Jj, "roll_incl": roll_incl}
