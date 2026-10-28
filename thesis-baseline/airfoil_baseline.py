#!/usr/bin/env python3
"""Deterministic baseline for fitting one airfoil side (PS or SS).

Fits a cubic B-spline to the side's coordinates by constrained least squares
plus a fairness term, instead of interpolating every point:

    min_P  sum_k |C(u_k; P) - Q_k|^2  +  lam * integral |C'''(u)|^2 du

subject to G1 continuity with the LE/TE arcs:

    P[0]  = start point (tangency with the first edge arc)
    P[1]  = P[0]  + a * t_start,   a > 0
    P[-2] = P[-1] - b * t_end,     b > 0
    P[-1] = end point (tangency with the second edge arc)

Tangent vectors point in the direction of travel along the side. The problem is
linear in (a, b, interior control points), so each fit is one bounded linear
least-squares solve.

The fit-to-tolerance driver tries increasing numbers of control points and, for
each one that meets the deviation tolerance, raises lam as far as the tolerance
allows. It returns the candidate with the fewest inflections.

Only numpy and scipy are required (matplotlib for --plot). No data leaves the
machine.
"""

import argparse
import json
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from scipy.interpolate import BSpline, make_interp_spline
from scipy.optimize import lsq_linear, minimize, minimize_scalar

DEGREE = 3


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def cross2(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n == 0:
        raise ValueError("zero-length tangent vector")
    return v / n


def centripetal_params(pts):
    d = np.sqrt(np.linalg.norm(np.diff(pts, axis=0), axis=1))
    u = np.concatenate([[0.0], np.cumsum(d)])
    return u / u[-1]


def approx_knots(u, n_ctrl, p=DEGREE):
    """Clamped knot vector for approximation (Piegl & Tiller, eq. 9.68-9.69).

    `u` holds the parameters of all points including both ends (0 and 1).
    """
    r = len(u) - 1
    n = n_ctrl - 1
    d = (r + 1) / (n - p + 1)
    interior = []
    for j in range(1, n - p + 1):
        i = int(j * d)
        alpha = j * d - i
        interior.append((1 - alpha) * u[i - 1] + alpha * u[i])
    return np.concatenate([np.zeros(p + 1), interior, np.ones(p + 1)])


def basis(t, x, nu=0):
    """Matrix of B-spline basis functions (or their nu-th derivative) at x."""
    n_ctrl = len(t) - DEGREE - 1
    spl = BSpline(t, np.eye(n_ctrl), DEGREE, extrapolate=False)
    if nu:
        spl = spl.derivative(nu)
    return np.nan_to_num(spl(np.clip(x, 0.0, 1.0)))


def count_sign_changes(values, eps):
    s = np.sign(values[np.abs(values) > eps])
    return int(np.count_nonzero(s[1:] != s[:-1]))


# ---------------------------------------------------------------------------
# Fit result
# ---------------------------------------------------------------------------

@dataclass
class SideFit:
    knots: np.ndarray
    poles: np.ndarray
    lam: float
    deviations: np.ndarray = field(repr=False)
    params: np.ndarray = field(repr=False)

    @property
    def spline(self):
        return BSpline(self.knots, self.poles, DEGREE)

    @property
    def n_ctrl(self):
        return len(self.poles)

    @property
    def max_dev(self):
        return float(self.deviations.max()) if len(self.deviations) else 0.0

    def curvature(self, u):
        spl = self.spline
        d1, d2 = spl.derivative(1)(u), spl.derivative(2)(u)
        speed = np.linalg.norm(d1, axis=1)
        return cross2(d1, d2) / speed ** 3

    def sample_params(self, per_span=200):
        breaks = np.unique(self.knots)
        return np.unique(np.concatenate([
            np.linspace(a, b, per_span) for a, b in zip(breaks[:-1], breaks[1:])
        ]))

    def inflections(self, kappa_eps=1e-9):
        return count_sign_changes(self.curvature(self.sample_params()), kappa_eps)

    def polygon_inflections(self, eps=1e-15):
        e = np.diff(self.poles, axis=0)
        return count_sign_changes(cross2(e[:-1], e[1:]), eps)

    def end_curvatures(self):
        k = self.curvature(np.array([0.0, 1.0]))
        return float(k[0]), float(k[1])

    def to_dict(self, kappa_eps=1e-9):
        breaks, mults = np.unique(self.knots, return_counts=True)
        k0, k1 = self.end_curvatures()
        return {
            "degree": DEGREE,
            "poles": self.poles.tolist(),
            "knots": breaks.tolist(),
            "multiplicities": mults.tolist(),
            "lambda": self.lam,
            "n_ctrl": self.n_ctrl,
            "max_deviation": self.max_dev,
            "deviations": self.deviations.tolist(),
            "inflections": self.inflections(kappa_eps),
            "control_polygon_inflections": self.polygon_inflections(),
            "curvature_at_start": k0,
            "curvature_at_end": k1,
        }


# ---------------------------------------------------------------------------
# Projection (true point-to-curve distance)
# ---------------------------------------------------------------------------

def project(spl, pts, u_guess=None, n_samples=4000, newton_iters=10):
    """Closest parameter on `spl` for each point, and the distance to it."""
    if u_guess is None:
        us = np.linspace(0.0, 1.0, n_samples)
        cs = spl(us)
        d2 = ((pts[:, None, :] - cs[None, :, :]) ** 2).sum(axis=2)
        u = us[np.argmin(d2, axis=1)]
    else:
        u = np.asarray(u_guess, dtype=float).copy()
    d1, d2s = spl.derivative(1), spl.derivative(2)
    for _ in range(newton_iters):
        r = spl(u) - pts
        c1, c2 = d1(u), d2s(u)
        g = (r * c1).sum(axis=1)
        h = (c1 * c1).sum(axis=1) + (r * c2).sum(axis=1)
        step = np.where(h > 0, g / np.where(h > 0, h, 1.0), 0.0)
        u = np.clip(u - step, 0.0, 1.0)
    dist = np.linalg.norm(spl(u) - pts, axis=1)
    return u, dist


# ---------------------------------------------------------------------------
# Constrained least-squares fit
# ---------------------------------------------------------------------------

def rot90(v):
    return np.array([-v[1], v[0]])


def _pole_map(n_ctrl, p_start, p_end, t_start, t_end, len_start=None, len_end=None,
              h_start=None, h_end=None):
    """Affine map from unknowns z to flattened poles: P_flat = G @ z + h.

    Returns (G, h, pos), where pos lists the columns of z that must be positive.

    G1 ends: P1 = P0 + a t0 and P(n-2) = P(n-1) - b t1, with a, b > 0 unknown.
    G2 ends: the handle length is given (len_start / len_end), and the next
    pole sits at the height above the tangent line that gives the required
    end curvature (h_start / h_end, from _g2_height):
        P2     = P0     + s0 t0 + h_start rot90(t0)
        P(n-3) = P(n-1) + s1 d1 + h_end   rot90(d1),   d1 = -t1
    """
    g2_start, g2_end = len_start is not None, len_end is not None
    i_start = 2 if g2_start else None
    i_end = n_ctrl - 3 if g2_end else None
    free = [i for i in range(2, n_ctrl - 2) if i not in (i_start, i_end)]
    G = np.zeros((2 * n_ctrl, 2 + 2 * len(free)))
    h = np.zeros(2 * n_ctrl)
    pos = []

    def rows(i):
        return slice(2 * i, 2 * i + 2)

    h[rows(0)] = p_start
    if g2_start:
        h[rows(1)] = p_start + len_start * t_start
        h[rows(i_start)] = p_start + h_start * rot90(t_start)
        G[rows(i_start), 0] = t_start
    else:
        h[rows(1)] = p_start
        G[rows(1), 0] = t_start
        pos.append(0)

    h[rows(n_ctrl - 1)] = p_end
    if g2_end:
        d1 = -t_end
        h[rows(n_ctrl - 2)] = p_end + len_end * d1
        h[rows(i_end)] = p_end + h_end * rot90(d1)
        G[rows(i_end), 1] = d1
    else:
        h[rows(n_ctrl - 2)] = p_end
        G[rows(n_ctrl - 2), 1] = -t_end
        pos.append(1)

    col = 2
    for i in free:
        G[2 * i, col] = 1.0
        G[2 * i + 1, col + 1] = 1.0
        col += 2
    return G, h, pos


def _g2_height(kappa, length, k1, k2, p=DEGREE):
    """Height of the third pole above the end tangent that gives end curvature
    `kappa` for an end handle of `length` (clamped B-spline; k1, k2 are the
    first two interior knot distances from that end)."""
    return kappa * length ** 2 * p * k2 / ((p - 1) * k1)


def _solve(Q, u, knots, lam, G, h, pos, min_len):
    """Bounded linear least squares. Returns (poles, cost)."""
    n_ctrl = len(knots) - DEGREE - 1
    I2 = np.eye(2)

    B = np.kron(basis(knots, u), I2)
    A_fit = B @ G
    b_fit = Q.ravel() - B @ h

    # C''' of a cubic is constant on each knot span, so the integral is exact.
    breaks = np.unique(knots)
    mids = 0.5 * (breaks[:-1] + breaks[1:])
    lens = np.diff(breaks)
    W = np.kron(np.sqrt(lens)[:, None] * basis(knots, mids, nu=3), I2)
    A_fair = W @ G
    b_fair = -W @ h

    # Make lam dimensionless: lam = 1 weighs both terms about equally.
    nf = np.linalg.norm(A_fair)
    scale = np.linalg.norm(A_fit) / nf if nf > 0 else 0.0
    w = math.sqrt(lam) * scale

    A = np.vstack([A_fit, w * A_fair])
    b = np.concatenate([b_fit, w * b_fair])
    lb = np.full(A.shape[1], -np.inf)
    lb[pos] = min_len
    x = lsq_linear(A, b, bounds=(lb, np.inf), method="bvls").x
    r = A @ x - b
    return (G @ x + h).reshape(n_ctrl, 2), float(r @ r)


def fit_side(Q, p_start, p_end, t_start, t_end, n_ctrl, lam=0.0,
             param_iters=3, k_start=None, k_end=None):
    """Fit one side with `n_ctrl` control points and fairness weight `lam`.

    Q: (K, 2) side points strictly between the two end points, in order of
    travel from p_start to p_end.

    k_start / k_end: optional signed end curvatures (counter-clockwise
    positive, in the direction of travel) for G2 joints. End curvature is
    nonlinear in the end handle's length but linear in the next pole once that
    length is fixed. So each G2 handle length is searched directly (bounded
    1-D search, or Nelder-Mead for two), with one linear solve per trial.
    """
    g2 = (k_start is not None) + (k_end is not None)
    if n_ctrl < 4 + g2 or (g2 == 2 and n_ctrl < 6):
        raise ValueError(f"n_ctrl={n_ctrl} too small for {g2} G2 end(s)")
    Q = np.asarray(Q, dtype=float)
    p_start, p_end = np.asarray(p_start, float), np.asarray(p_end, float)
    t_start, t_end = unit(t_start), unit(t_end)

    u_all = centripetal_params(np.vstack([p_start, Q, p_end]))
    if n_ctrl > len(u_all):
        raise ValueError(f"n_ctrl={n_ctrl} exceeds the number of points ({len(u_all)})")
    knots = approx_knots(u_all, n_ctrl)
    u = u_all[1:-1]
    chord = np.linalg.norm(p_end - p_start)
    min_len = 1e-6 * chord
    p = DEGREE
    k1s, k2s = knots[p + 1], knots[p + 2]
    k1e, k2e = 1.0 - knots[-(p + 2)], 1.0 - knots[-(p + 3)]

    def solve_g1(u):
        G, h, pos = _pole_map(n_ctrl, p_start, p_end, t_start, t_end)
        return _solve(Q, u, knots, lam, G, h, pos, min_len)[0]

    def solve_g2(u, lengths):
        it = iter(lengths)
        ls = next(it) if k_start is not None else None
        le = next(it) if k_end is not None else None
        hs = None if ls is None else _g2_height(k_start, ls, k1s, k2s)
        # Reversing the curve flips the sign of its curvature.
        he = None if le is None else _g2_height(-k_end, le, k1e, k2e)
        G, h, pos = _pole_map(n_ctrl, p_start, p_end, t_start, t_end, ls, le, hs, he)
        return _solve(Q, u, knots, lam, G, h, pos, min_len)

    lo, hi = math.log(1e-4 * chord), math.log(chord)

    def solve(u, guess):
        if not g2:
            return solve_g1(u), None
        cost = lambda x: solve_g2(u, np.exp(np.clip(x, lo, hi)))[1]
        if g2 == 1:
            r = minimize_scalar(lambda x: cost([x]), bounds=(lo, hi), method="bounded",
                                options={"xatol": 1e-4})
            x = np.array([r.x])
        else:
            x0 = np.clip(np.log(guess), lo, hi)
            x = minimize(cost, x0, method="Nelder-Mead",
                         options={"xatol": 1e-4, "fatol": 1e-18, "maxiter": 400}).x
        lengths = np.exp(np.clip(x, lo, hi))
        return solve_g2(u, lengths)[0], lengths

    poles = solve_g1(u)
    guess = []
    if k_start is not None:
        guess.append(max(np.linalg.norm(poles[1] - poles[0]), 1e-3 * chord))
    if k_end is not None:
        guess.append(max(np.linalg.norm(poles[-1] - poles[-2]), 1e-3 * chord))
    poles, lengths = solve(u, np.array(guess))
    # Parameter correction: re-project the points and refit with knots fixed.
    for _ in range(param_iters):
        u, _ = project(BSpline(knots, poles, DEGREE), Q, u_guess=u)
        poles, lengths = solve(u, lengths)

    spl = BSpline(knots, poles, DEGREE)
    u, dev = project(spl, Q)
    return SideFit(knots=knots, poles=poles, lam=lam, deviations=dev, params=u)


def fit_to_tolerance(Q, p_start, p_end, t_start, t_end, tol, m_min=4,
                     m_max=None, m_extra=4, lam_range=(1e-8, 1e4),
                     lam_iters=30, kappa_eps=1e-9, log=None, k_start=None,
                     k_end=None):
    """Fewest-inflection fit that keeps every point within `tol`.

    For each control-point count from m_min upward, fits with lam = 0; if that
    meets the tolerance, raises lam (bisection in log space) as far as the
    tolerance allows. Stops m_extra counts after the first feasible one.

    Returns (best_fit, feasible). If no count meets the tolerance, returns the
    fit with the smallest maximum deviation and feasible = False.
    """
    Q = np.asarray(Q, dtype=float)
    if m_max is None:
        m_max = len(Q) + 2
    g2 = (k_start is not None) + (k_end is not None)
    m_min = max(m_min, {0: 4, 1: 5, 2: 6}[g2])
    log = log or (lambda *_: None)
    curv = {"k_start": k_start, "k_end": k_end}

    candidates, closest, first_ok = [], None, None
    for m in range(m_min, m_max + 1):
        base = fit_side(Q, p_start, p_end, t_start, t_end, m, 0.0, **curv)
        if closest is None or base.max_dev < closest.max_dev:
            closest = base
        if base.max_dev > tol:
            log(f"  m={m:2d}  lam=0         max_dev={base.max_dev:.6f}  over tolerance")
            continue

        best = base
        lo, hi = math.log10(lam_range[0]), math.log10(lam_range[1])
        top = fit_side(Q, p_start, p_end, t_start, t_end, m, 10 ** hi, **curv)
        if top.max_dev <= tol:
            best = top
        else:
            for _ in range(lam_iters):
                mid = 0.5 * (lo + hi)
                trial = fit_side(Q, p_start, p_end, t_start, t_end, m, 10 ** mid, **curv)
                if trial.max_dev <= tol:
                    best, lo = trial, mid
                else:
                    hi = mid
        log(f"  m={m:2d}  lam={best.lam:<9.3g} max_dev={best.max_dev:.6f}  "
            f"inflections={best.inflections(kappa_eps)}")
        candidates.append(best)
        first_ok = m if first_ok is None else first_ok
        if m >= first_ok + m_extra:
            break

    if not candidates:
        return closest, False
    best = min(candidates, key=lambda f: (f.inflections(kappa_eps), f.n_ctrl, f.max_dev))
    return best, True


# ---------------------------------------------------------------------------
# Comparison: plain interpolation (what Algorithm 4 does today)
# ---------------------------------------------------------------------------

def interpolate_side(Q, p_start, p_end, t_start, t_end):
    """Cubic interpolation through every point with clamped end tangents."""
    pts = np.vstack([p_start, Q, p_end])
    u = centripetal_params(pts)
    length = np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()
    bc = ([(1, unit(t_start) * length)], [(1, unit(t_end) * length)])
    spl = make_interp_spline(u, pts, k=DEGREE, bc_type=bc)
    return SideFit(knots=spl.t, poles=spl.c, lam=0.0,
                   deviations=np.zeros(len(Q)), params=u[1:-1])


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def read_points(path):
    rows = []
    with open(path, newline="") as f:
        for line in f:
            parts = [p.strip() for p in line.replace(";", ",").split(",") if p.strip()]
            if len(parts) < 2:
                continue
            try:
                rows.append((float(parts[0]), float(parts[1])))
            except ValueError:
                continue  # header or comment
    if len(rows) < 3:
        raise ValueError(f"{path}: need at least 3 points, found {len(rows)}")
    return np.array(rows)


def estimate_tangent(pts):
    """Tangent at pts[0] from a quadratic fit through the first few points."""
    k = min(4, len(pts))
    u = centripetal_params(pts[:k])
    cx = np.polyfit(u, pts[:k, 0], min(2, k - 1))
    cy = np.polyfit(u, pts[:k, 1], min(2, k - 1))
    return unit([np.polyval(np.polyder(cx), 0.0), np.polyval(np.polyder(cy), 0.0)])


def plot(fit, Q, path, interp=None, tol=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 11))
    us = fit.sample_params()
    c = fit.spline(us)
    ax1.plot(c[:, 0], c[:, 1], "-", lw=1.5, label="Baseline fit")
    ax1.plot(fit.poles[:, 0], fit.poles[:, 1], "o--", ms=4, lw=0.8, label="Control polygon")
    ax1.plot(Q[:, 0], Q[:, 1], "k.", ms=5, label="Side points")
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.set_title(f"m = {fit.n_ctrl} control points, lambda = {fit.lam:.3g}")
    ax1.legend(loc="best", fontsize=8)

    ax2.axhline(0, color="0.6", lw=0.8)
    if interp is not None:
        ui = interp.sample_params()
        ax2.plot(ui, interp.curvature(ui), lw=1, alpha=0.7,
                 label=f"Interpolation ({interp.inflections()} inflections)")
    ax2.plot(us, fit.curvature(us), lw=1.5,
             label=f"Baseline ({fit.inflections()} inflections)")
    ax2.set_xlabel("curve parameter u")
    ax2.set_ylabel("signed curvature (1/length unit)")
    ax2.legend(loc="best", fontsize=8)

    ax3.bar(np.arange(1, len(Q) + 1), fit.deviations)
    if tol is not None:
        ax3.axhline(tol, color="r", lw=1, label=f"tolerance {tol:g}")
        ax3.legend(loc="best", fontsize=8)
    ax3.set_xlabel("side point index")
    ax3.set_ylabel("deviation (length unit)")

    fig.tight_layout()
    fig.savefig(path, dpi=130)


# ---------------------------------------------------------------------------
# Demo data (synthetic; no real blade data)
# ---------------------------------------------------------------------------

def demo_side(n=25, decimals=4):
    """A smooth side with no inflections and a nearly flat stretch, sampled at
    n interior points and rounded like drawing coordinates.

    y = 0.032 (1.6 - x)^2.5 has y'' > 0 everywhere, shrinking toward x = 1.6,
    so rounding noise is enough to flip the sign of an interpolant's curvature.
    The end tangents are exact, as the edge arcs would provide them.
    """
    x = np.linspace(0.0, 1.5, n + 2)
    y = 0.032 * (1.6 - x) ** 2.5
    pts = np.round(np.column_stack([x, y]), decimals)

    def tangent(xi):
        return unit([1.0, -0.08 * (1.6 - xi) ** 1.5])

    return pts[1:-1], pts[0], pts[-1], tangent(x[0]), tangent(x[-1])


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Least-squares + fairness baseline for one airfoil side.")
    ap.add_argument("csv", nargs="?", help="ordered side points: x,y per line")
    ap.add_argument("--tol", type=float, default=0.00015,
                    help="max point deviation (default 0.00015)")
    ap.add_argument("--start-point", nargs=2, type=float, metavar=("X", "Y"),
                    help="tangency point at the start (default: first CSV point)")
    ap.add_argument("--end-point", nargs=2, type=float, metavar=("X", "Y"),
                    help="tangency point at the end (default: last CSV point)")
    ap.add_argument("--start-tangent", nargs=2, type=float, metavar=("TX", "TY"),
                    help="edge-arc tangent at the start, in the direction of travel")
    ap.add_argument("--end-tangent", nargs=2, type=float, metavar=("TX", "TY"),
                    help="edge-arc tangent at the end, in the direction of travel")
    ap.add_argument("--m-min", type=int, default=4, help="fewest control points to try")
    ap.add_argument("--m-max", type=int, help="most control points to try")
    ap.add_argument("--m-extra", type=int, default=4,
                    help="control-point counts to try after the first feasible one")
    ap.add_argument("--kappa-eps", type=float, default=1e-9,
                    help="ignore |curvature| below this when counting inflections")
    ap.add_argument("--out", help="write the fit as JSON")
    ap.add_argument("--plot", help="write a PNG with geometry, curvature and deviations")
    ap.add_argument("--demo", action="store_true",
                    help="run on synthetic data instead of a CSV")
    args = ap.parse_args(argv)

    if args.demo:
        Q, ps, pe, t0, t1 = demo_side()
    elif args.csv:
        pts = read_points(args.csv)
        ps = np.array(args.start_point) if args.start_point else pts[0]
        pe = np.array(args.end_point) if args.end_point else pts[-1]
        Q = pts[(0 if args.start_point else 1):(len(pts) if args.end_point else -1)]
        t0 = unit(args.start_tangent) if args.start_tangent else estimate_tangent(
            np.vstack([ps, Q]))
        t1 = unit(args.end_tangent) if args.end_tangent else -estimate_tangent(
            np.vstack([pe, Q[::-1]]))
        if not (args.start_tangent and args.end_tangent):
            print("warning: end tangent(s) estimated from the points; pass the "
                  "edge-arc tangents for G1 continuity with the arcs", file=sys.stderr)
    else:
        ap.error("give a CSV file or --demo")

    print(f"{len(Q)} side points, tolerance {args.tol:g}")
    print("Searching control-point counts:")
    fit, feasible = fit_to_tolerance(
        Q, ps, pe, t0, t1, args.tol, m_min=args.m_min, m_max=args.m_max,
        m_extra=args.m_extra, kappa_eps=args.kappa_eps, log=print)
    interp = interpolate_side(Q, ps, pe, t0, t1)

    print()
    print(f"Interpolation through every point: {interp.inflections(args.kappa_eps)} inflections")
    status = "meets" if feasible else "DOES NOT meet"
    print(f"Baseline fit: {fit.inflections(args.kappa_eps)} inflections, "
          f"max deviation {fit.max_dev:.6f} ({status} tolerance), "
          f"{fit.n_ctrl} control points, lambda {fit.lam:.3g}")
    print(f"Control polygon inflections: {fit.polygon_inflections()}")
    k0, k1 = fit.end_curvatures()
    print(f"Curvature at start/end: {k0:+.4g} / {k1:+.4g} "
          "(compare signs with the adjacent edge arcs to catch junction inflections)")
    over = int(np.count_nonzero(fit.deviations > args.tol))
    print(f"Points over tolerance: {over}")

    if args.out:
        with open(args.out, "w") as f:
            json.dump(fit.to_dict(args.kappa_eps), f, indent=2)
        print(f"Wrote {args.out}")
    if args.plot:
        plot(fit, Q, args.plot, interp=interp, tol=args.tol)
        print(f"Wrote {args.plot}")
    return 0 if feasible else 1


if __name__ == "__main__":
    sys.exit(main())
