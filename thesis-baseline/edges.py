#!/usr/bin/env python3
"""LE/TE detection, edge-circle fitting and tangency selection for one section.

A replacement for Algorithms 2-3 of the thesis:

1. Edges from curvature. A lightly smoothed periodic spline through the
   ordered contour; the LE and TE are its two strongest convex curvature peaks.
2. Edge circles by geometric least squares. Starting from a few points around
   each peak, neighbours are added while every window point stays within the
   deviation tolerance of the fitted circle.
3. Optional fixed radius. When the drawing gives LE_R / TE_R, only the center
   is fitted.
4. Tangency by fit quality. Each tangency point is an angle on its circle,
   searched between the last window point and the first side point. For each
   candidate pair of angles, the side is fitted with the baseline
   (airfoil_baseline.fit_side) and scored by: points over tolerance, then
   inflections (including at the arc-side joints), then maximum deviation.

Input: contour points ordered around the section (Algorithm 1's output), as an
(N, 2) array or a CSV of x,y rows. Only numpy and scipy are required.
"""

import argparse
import json
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from scipy.interpolate import splev, splprep
from scipy.optimize import least_squares

import airfoil_baseline as ab

# Side curvature (1/length unit) below which a joint's sign is not judged.
JUNCTION_EPS = 1e-2

# Edge windows grow while every point is within GROW_FRAC * tol of the circle.
# Below 1, side points that happen to lie within tolerance near a tangency are
# less likely to join the window; rounding to 4 decimals moves true arc points
# by at most ~0.00007, so 0.5 * 0.00015 still admits them.
GROW_FRAC = 0.5


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def signed_area(pts):
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def circle_point(center, radius, theta):
    return center + radius * np.array([math.cos(theta), math.sin(theta)])


def circle_tangent(theta, orient):
    """Unit tangent at angle theta, in the contour's direction of travel."""
    return orient * np.array([-math.sin(theta), math.cos(theta)])


# ---------------------------------------------------------------------------
# Step 1: edges from curvature
# ---------------------------------------------------------------------------

def curvature_peaks(pts, tol, min_separation=0.25, oversample=20):
    """Contour indices of the two strongest convex curvature peaks.

    Returns (indices, orient), where orient is +1 for a counter-clockwise
    contour and -1 for clockwise.
    """
    n = len(pts)
    orient = 1.0 if signed_area(pts) > 0 else -1.0
    closed = np.vstack([pts, pts[:1]])
    smoothing = n * (0.5 * tol) ** 2
    tck, u_data = splprep(closed.T, s=smoothing, per=1, quiet=2)

    us = np.linspace(0.0, 1.0, oversample * n, endpoint=False)
    dx, dy = splev(us, tck, der=1)
    ddx, ddy = splev(us, tck, der=2)
    kappa = orient * (dx * ddy - dy * ddx) / np.hypot(dx, dy) ** 3

    first = int(np.argmax(kappa))
    sep = np.abs(us - us[first])
    sep = np.minimum(sep, 1.0 - sep)
    masked = np.where(sep > min_separation, kappa, -np.inf)
    second = int(np.argmax(masked))

    peaks = []
    for k in (first, second):
        x, y = splev(us[k], tck)
        peaks.append(int(np.argmin(np.hypot(pts[:, 0] - x, pts[:, 1] - y))))
    return peaks, orient


# ---------------------------------------------------------------------------
# Step 2-3: edge circles
# ---------------------------------------------------------------------------

def fit_circle(points, radius=None, init=None):
    """Geometric least-squares circle. Returns (center, radius, max_residual).

    With `radius` given, only the center is fitted.
    """
    p = np.asarray(points, dtype=float)
    if init is None:
        # Algebraic (Kasa) fit as the starting point.
        A = np.column_stack([2 * p, np.ones(len(p))])
        b = (p ** 2).sum(axis=1)
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        c0 = sol[:2]
        r0 = math.sqrt(max(sol[2] + c0 @ c0, 1e-30))
    else:
        c0, r0 = np.asarray(init[0], float), float(init[1])

    if radius is None:
        res = least_squares(
            lambda z: np.hypot(*(p - z[:2]).T) - z[2], np.r_[c0, r0], method="lm"
            if len(p) >= 3 else "trf")
        center, r = res.x[:2], float(abs(res.x[2]))
    else:
        r = float(radius)
        res = least_squares(lambda z: np.hypot(*(p - z).T) - r, c0,
                            method="lm" if len(p) >= 2 else "trf")
        center = res.x
    resid = np.abs(np.hypot(*(p - center).T) - r)
    return center, r, float(resid.max())


@dataclass
class Edge:
    seed: int
    window: list          # contour indices on the arc, in travel order
    center: np.ndarray
    radius: float
    max_residual: float
    radius_fixed: bool = False
    label: str = ""

    def to_dict(self):
        return {
            "label": self.label,
            "center": self.center.tolist(),
            "radius": self.radius,
            "radius_fixed": self.radius_fixed,
            "window": [int(i) for i in self.window],
            "max_residual": self.max_residual,
        }


def grow_edge(pts, seed, tol, radius=None, grow_tol=None, start_half=2,
              max_frac=0.25, max_passes=4):
    """Fit a circle around `seed`, growing the window while it stays within tol.

    The window must stay within `tol`, and each point added to it must itself
    lie within `grow_tol` (default `tol`) of the refitted circle.

    The curvature peak is often off-center on the arc, so a starting window
    around it can include side points. After each pass the seed moves to the
    middle of the window and the window is regrown, until it stops changing.
    """
    n = len(pts)
    edge = None
    for _ in range(max_passes):
        edge = _grow_once(pts, seed, tol, grow_tol if grow_tol is not None else tol,
                          radius, start_half, max_frac)
        middle = edge.window[len(edge.window) // 2]
        if middle == seed % n:
            break
        seed = middle
    return edge


def _grow_once(pts, seed, tol, grow_tol, radius, start_half, max_frac):
    n = len(pts)
    max_len = max(3, int(max_frac * n))

    half = start_half
    while True:
        lo, hi = seed - half, seed + half
        idx = [i % n for i in range(lo, hi + 1)]
        center, r, res = fit_circle(pts[idx], radius)
        if res <= tol or half == 1:
            break
        half -= 1

    grow_left = grow_right = True
    while (grow_left or grow_right) and (hi - lo + 1) < max_len:
        for side in ("left", "right"):
            if side == "left" and not grow_left:
                continue
            if side == "right" and not grow_right:
                continue
            nlo, nhi = (lo - 1, hi) if side == "left" else (lo, hi + 1)
            idx = [i % n for i in range(nlo, nhi + 1)]
            c2, r2, res2 = fit_circle(pts[idx], radius, init=(center, r))
            new = pts[(nlo if side == "left" else nhi) % n]
            new_res = abs(np.hypot(*(new - c2)) - r2)
            if res2 <= tol and new_res <= grow_tol:
                lo, hi, center, r, res = nlo, nhi, c2, r2, res2
            elif side == "left":
                grow_left = False
            else:
                grow_right = False

    window = [i % n for i in range(lo, hi + 1)]
    return Edge(seed=seed % n, window=window, center=center, radius=r,
                max_residual=res, radius_fixed=radius is not None)


def detect_edges(pts, tol, le_radius=None, te_radius=None, grow_frac=GROW_FRAC):
    """Find, fit and label the two edges. Returns (edges, orient).

    Edges are labelled by radius: with drawing radii, by the closer match;
    otherwise the larger circle is taken as the LE (typical for turbine
    blades; check this for your parts).
    """
    peaks, orient = curvature_peaks(pts, tol)
    grow_tol = grow_frac * tol
    free = [grow_edge(pts, s, tol, grow_tol=grow_tol) for s in peaks]

    if le_radius is not None and te_radius is not None:
        direct = abs(free[0].radius - le_radius) + abs(free[1].radius - te_radius)
        swapped = abs(free[0].radius - te_radius) + abs(free[1].radius - le_radius)
        labels = ("LE", "TE") if direct <= swapped else ("TE", "LE")
    else:
        labels = ("LE", "TE") if free[0].radius >= free[1].radius else ("TE", "LE")

    radii = {"LE": le_radius, "TE": te_radius}
    edges = []
    for e, label in zip(free, labels):
        if radii[label] is not None:
            e = grow_edge(pts, e.seed, tol, radius=radii[label], grow_tol=grow_tol)
        e.label = label
        edges.append(e)
    return edges, orient


# ---------------------------------------------------------------------------
# Step 4: tangency by fit quality
# ---------------------------------------------------------------------------

def _angle(edge, p):
    d = p - edge.center
    return math.atan2(d[1], d[0])


@dataclass
class _End:
    """Search range for one tangency, and the window points it may hand to the side.

    The window can hold side points that happen to lie within tolerance of the
    circle, so the range starts `reach` points inside the window and ends at
    the first point outside it. Window points past the chosen tangency (in the
    direction away from the arc) are fitted as side points.
    """
    lo: float           # angle range, in travel order
    hi: float
    idx: list           # window points inside the range, in travel order
    ang: np.ndarray     # their angles, unwrapped consistently with lo/hi
    end: str


def _end_range(edge, pts, end, orient, n, reach=2):
    reach = max(0, min(reach, len(edge.window) - 2))
    if end == "out":
        idx = edge.window[len(edge.window) - 1 - reach:]
        outside = (edge.window[-1] + 1) % n
        ref = _angle(edge, pts[idx[0]])
        ang = np.array([ref + wrap_angle(_angle(edge, pts[i]) - ref) for i in idx])
        a_out = ref + wrap_angle(_angle(edge, pts[outside]) - ref)
        return _End(lo=ref, hi=a_out, idx=list(idx), ang=ang, end=end)
    idx = edge.window[:reach + 1]
    outside = (edge.window[0] - 1) % n
    ref = _angle(edge, pts[idx[-1]])
    ang = np.array([ref + wrap_angle(_angle(edge, pts[i]) - ref) for i in idx])
    a_out = ref + wrap_angle(_angle(edge, pts[outside]) - ref)
    return _End(lo=a_out, hi=ref, idx=list(idx), ang=ang, end=end)


def _side_points(pts, outside_idx, start, end, th0, th1, orient):
    """Side points for tangency angles th0 (leaving start edge) and th1
    (arriving at end edge): outside points plus window points past each tangency."""
    head = [i for i, a in zip(start.idx, start.ang) if orient * (a - th0) > 0]
    tail = [i for i, a in zip(end.idx, end.ang) if orient * (a - th1) < 0]
    return pts[head + list(outside_idx) + tail]


def side_indices(n, start_edge, end_edge):
    """Contour indices strictly between two edges' windows, in travel order."""
    i = (start_edge.window[-1] + 1) % n
    stop = start_edge.window[0] if end_edge is start_edge else end_edge.window[0]
    out = []
    while i != stop:
        out.append(i)
        i = (i + 1) % n
    return out


@dataclass
class SideResult:
    start_edge: str
    end_edge: str
    theta_start: float
    theta_end: float
    p_start: np.ndarray
    p_end: np.ndarray
    fit: ab.SideFit = field(repr=False)
    junction_inflections: int
    curvature_jump_start: float
    curvature_jump_end: float
    tol: float

    @property
    def points_over(self):
        return int(np.count_nonzero(self.fit.deviations > self.tol))

    def to_dict(self):
        d = self.fit.to_dict()
        d.update({
            "start_edge": self.start_edge,
            "end_edge": self.end_edge,
            "theta_start": self.theta_start,
            "theta_end": self.theta_end,
            "tangency_start": self.p_start.tolist(),
            "tangency_end": self.p_end.tolist(),
            "junction_inflections": self.junction_inflections,
            "curvature_jump_start": self.curvature_jump_start,
            "curvature_jump_end": self.curvature_jump_end,
            "points_over_tolerance": self.points_over,
        })
        return d


def _evaluate_side(Q, e0, e1, th0, th1, orient, m, lam, tol, kappa_eps,
                   param_iters=1, fit=None):
    p0, p1 = circle_point(e0.center, e0.radius, th0), circle_point(e1.center, e1.radius, th1)
    t0, t1 = circle_tangent(th0, orient), circle_tangent(th1, orient)
    if fit is None:
        m = min(m, len(Q) + 2)
        fit = ab.fit_side(Q, p0, p1, t0, t1, m, lam, param_iters=param_iters)
    k_start, k_end = fit.end_curvatures()
    k0, k1 = orient / e0.radius, orient / e1.radius
    # A joint is an inflection when the side turns the other way from the arc.
    # Near-zero side curvature is ambiguous, so it must exceed JUNCTION_EPS.
    junction = int(orient * k_start < -JUNCTION_EPS) + int(orient * k_end < -JUNCTION_EPS)
    over = int(np.count_nonzero(fit.deviations > tol))
    score = (over, fit.inflections(kappa_eps) + junction, fit.max_dev)
    return score, fit, p0, p1, junction, abs(k_start - k0) * e0.radius, abs(k_end - k1) * e1.radius


def choose_side_tangency(pts, Q_idx, e0, e1, orient, tol, grid=9,
                         kappa_eps=1e-9):
    """Search both tangency angles of one side on a grid over their ranges.

    The control-point count and smoothing are chosen once at the middle of the
    ranges; the chosen tangencies then get a full fit-to-tolerance search.
    """
    n = len(pts)
    r0 = _end_range(e0, pts, "out", orient, n)
    r1 = _end_range(e1, pts, "in", orient, n)

    def side_fit_to_tol(th0, th1):
        Q = _side_points(pts, Q_idx, r0, r1, th0, th1, orient)
        p0 = circle_point(e0.center, e0.radius, th0)
        p1 = circle_point(e1.center, e1.radius, th1)
        fit, _ = ab.fit_to_tolerance(Q, p0, p1, circle_tangent(th0, orient),
                                     circle_tangent(th1, orient), tol,
                                     kappa_eps=kappa_eps)
        return Q, fit

    _, ref = side_fit_to_tol(0.5 * (r0.lo + r0.hi), 0.5 * (r1.lo + r1.hi))
    m, lam = ref.n_ctrl, ref.lam

    best = None
    for th0 in np.linspace(r0.lo, r0.hi, grid):
        for th1 in np.linspace(r1.lo, r1.hi, grid):
            Q = _side_points(pts, Q_idx, r0, r1, th0, th1, orient)
            score = _evaluate_side(Q, e0, e1, th0, th1, orient, m, lam, tol, kappa_eps)[0]
            if best is None or score < best[0]:
                best = (score, th0, th1)
    _, th0, th1 = best
    Q, fit = side_fit_to_tol(th0, th1)
    _, fit, p0, p1, junction, jump0, jump1 = _evaluate_side(
        Q, e0, e1, th0, th1, orient, None, None, tol, kappa_eps, fit=fit)
    return SideResult(start_edge=e0.label, end_edge=e1.label,
                      theta_start=float(th0), theta_end=float(th1),
                      p_start=p0, p_end=p1, fit=fit,
                      junction_inflections=junction,
                      curvature_jump_start=float(jump0),
                      curvature_jump_end=float(jump1), tol=tol)


# ---------------------------------------------------------------------------
# Whole section
# ---------------------------------------------------------------------------

@dataclass
class SectionResult:
    edges: list
    sides: list
    orient: float

    def to_dict(self):
        return {
            "orientation": "ccw" if self.orient > 0 else "cw",
            "edges": [e.to_dict() for e in self.edges],
            "sides": [s.to_dict() for s in self.sides],
        }


def reconstruct_section(pts, tol=0.00015, le_radius=None, te_radius=None,
                        grid=9, kappa_eps=1e-9, grow_frac=GROW_FRAC):
    pts = np.asarray(pts, dtype=float)
    edges, orient = detect_edges(pts, tol, le_radius, te_radius, grow_frac)
    a, b = edges
    n = len(pts)
    sides = [
        choose_side_tangency(pts, side_indices(n, a, b), a, b, orient, tol, grid, kappa_eps),
        choose_side_tangency(pts, side_indices(n, b, a), b, a, orient, tol, grid, kappa_eps),
    ]
    return SectionResult(edges=edges, sides=sides, orient=orient)


# ---------------------------------------------------------------------------
# Thesis method, for comparison (Algorithm 2's circle and classification)
# ---------------------------------------------------------------------------

def three_point_circle(pts, cusp):
    """Circle through the cusp and its two neighbours, as in Algorithm 2."""
    n = len(pts)
    return fit_circle(pts[[(cusp - 1) % n, cusp, (cusp + 1) % n]])[:2]


def threshold_classify(pts, center, radius, threshold=0.001):
    """Indices within `threshold` of the circle, as in Algorithm 2."""
    d = np.abs(np.hypot(*(pts - center).T) - radius)
    return np.flatnonzero(d < threshold)


# ---------------------------------------------------------------------------
# Synthetic section with known answer
# ---------------------------------------------------------------------------

@dataclass
class SyntheticSection:
    pts: np.ndarray
    centers: dict
    radii: dict
    tangency: dict      # label -> list of true tangency points
    on_arc: dict        # label -> contour indices that truly lie on the arc
    true_inflections: dict = field(default_factory=dict)  # "LE->TE" -> count


def _bezier(P, s, der=0):
    P = np.asarray(P, dtype=float)
    n = len(P) - 1
    for _ in range(der):
        P = n * np.diff(P, axis=0)
        n -= 1
    B = np.stack([math.comb(n, i) * s ** i * (1 - s) ** (n - i) for i in range(n + 1)], 1)
    return B @ P


def synthetic_section(le_r=0.05, te_r=0.02, edge_spacing=0.01, side_points=24,
                      decimals=4, seed=None):
    """A cambered section with a known answer, sampled and rounded like a drawing.

    Two circles joined by Bezier sides that are G1 at the tangency points by
    construction. The suction side is convex throughout (curvature >= 0.5).
    The pressure side is convex where it meets both arcs and concave in the
    middle, so it has exactly 2 inflections and none at the joints.

    Arc points sit half a spacing in from the tangency points, so no tangency
    point is a data point. Side points cluster toward the edges (cosine
    spacing), as drawing tables usually do.

    Contour runs counter-clockwise: LE arc, pressure side, TE arc, suction side.
    """
    c_le, c_te = np.array([0.0, 0.0]), np.array([1.5, 0.25])
    # Tangency angles on each circle, counter-clockwise travel.
    le_in, le_out = math.radians(125.0), math.radians(245.0)    # SS -> LE -> PS
    te_in, te_out = math.radians(-79.0), math.radians(65.0)     # PS -> TE -> SS

    def arc(c, r, a0, a1):
        k = max(2, int(round(r * (a1 - a0) / edge_spacing)))
        th = a0 + (np.arange(k) + 0.5) * (a1 - a0) / k
        return np.column_stack([c[0] + r * np.cos(th), c[1] + r * np.sin(th)])

    def side(P, k):
        s = np.linspace(0, 1, 8001)
        c = _bezier(P, s)
        L = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(c, axis=0), axis=1))])
        targets = L[-1] * 0.5 * (1 - np.cos(np.pi * np.linspace(0, 1, k + 2)[1:-1]))
        return np.column_stack([np.interp(targets, L, c[:, 0]), np.interp(targets, L, c[:, 1])])

    tan = lambda th: np.array([-math.sin(th), math.cos(th)])
    p_le_in, p_le_out = circle_point(c_le, le_r, le_in), circle_point(c_le, le_r, le_out)
    p_te_in, p_te_out = circle_point(c_te, te_r, te_in), circle_point(c_te, te_r, te_out)

    ps_poles = [p_le_out, p_le_out + 0.28 * tan(le_out), [0.579, -0.093], [1.006, 0.189],
                p_te_in - 0.14 * tan(te_in), p_te_in]
    ss_poles = [p_te_out, p_te_out + 0.2 * tan(te_out), p_le_in - 0.8 * tan(le_in), p_le_in]

    le_arc = arc(c_le, le_r, le_in, le_out)
    ps = side(ps_poles, side_points)
    te_arc = arc(c_te, te_r, te_in, te_out)
    ss = side(ss_poles, side_points)

    pts = np.vstack([le_arc, ps, te_arc, ss])
    if seed is not None:
        rng = np.random.default_rng(seed)
        pts = pts + rng.uniform(-0.5, 0.5, pts.shape) * 10.0 ** -decimals
    pts = np.round(pts, decimals)

    n_le, n_ps, n_te = len(le_arc), len(ps), len(te_arc)
    on_arc = {"LE": list(range(0, n_le)),
              "TE": list(range(n_le + n_ps, n_le + n_ps + n_te))}
    return SyntheticSection(
        pts=pts, centers={"LE": c_le, "TE": c_te}, radii={"LE": le_r, "TE": te_r},
        tangency={"LE": [p_le_in, p_le_out], "TE": [p_te_in, p_te_out]},
        on_arc=on_arc, true_inflections={"LE->TE": 2, "TE->LE": 0})


def compare_on_synthetic(sec, tol=0.00015, use_radii=False, grid=9):
    """Errors of the proposed method and of Algorithm 2's circle on `sec`."""
    radii = sec.radii if use_radii else {"LE": None, "TE": None}
    result = reconstruct_section(sec.pts, tol, radii["LE"], radii["TE"], grid=grid)
    rows = []
    for e in result.edges:
        truth_c, truth_r = sec.centers[e.label], sec.radii[e.label]
        cusp_c, cusp_r = three_point_circle(sec.pts, e.seed)
        old_class = set(threshold_classify(sec.pts, cusp_c, cusp_r).tolist())
        true_arc = set(sec.on_arc[e.label])
        rows.append({
            "edge": e.label,
            "radius_true": truth_r,
            "radius_three_point": cusp_r,
            "radius_lsq": e.radius,
            "center_err_lsq": float(np.linalg.norm(e.center - truth_c)),
            "window_points": len(e.window),
            "window_false": len(set(e.window) - true_arc),
            "threshold_false": len(old_class - true_arc),
        })
    tangency_err = []
    for s in result.sides:
        for label, p in ((s.start_edge, s.p_start), (s.end_edge, s.p_end)):
            tangency_err.append(min(np.linalg.norm(p - t) for t in sec.tangency[label]))
    return result, rows, tangency_err


def trials(n_trials, tol=0.00015, use_radii=False, grid=9, seed=0):
    """Repeat compare_on_synthetic over random spacing and rounding offsets."""
    rng = np.random.default_rng(seed)
    rows = []
    for t in range(n_trials):
        sec = synthetic_section(edge_spacing=float(rng.uniform(0.006, 0.012)),
                                side_points=int(rng.integers(18, 31)),
                                seed=int(rng.integers(1 << 30)))
        result, edge_rows, terr = compare_on_synthetic(sec, tol, use_radii, grid)
        sides = {f"{s.start_edge}->{s.end_edge}": s for s in result.sides}
        rows.append({
            "edges": edge_rows,
            "tangency_err": terr,
            "inflections_correct": all(
                s.fit.inflections() + s.junction_inflections == sec.true_inflections[k]
                for k, s in sides.items()),
            "points_over": sum(s.points_over for s in result.sides),
        })
    return rows


def summarize_trials(rows):
    def pct(a, q):
        return float(np.percentile(a, q))

    out = {"trials": len(rows)}
    for label in ("LE", "TE"):
        e = [r for row in rows for r in row["edges"] if r["edge"] == label]
        three = np.array([abs(r["radius_three_point"] - r["radius_true"]) for r in e])
        lsq = np.array([abs(r["radius_lsq"] - r["radius_true"]) for r in e])
        out[label] = {
            "radius_err_three_point_median": pct(three, 50),
            "radius_err_three_point_max": float(three.max()),
            "radius_err_lsq_median": pct(lsq, 50),
            "radius_err_lsq_max": float(lsq.max()),
            "threshold_misclassified_trials": int(sum(r["threshold_false"] > 0 for r in e)),
            "window_side_points_trials": int(sum(r["window_false"] > 0 for r in e)),
        }
    terr = np.array([x for row in rows for x in row["tangency_err"]])
    out["tangency_err_median"] = pct(terr, 50)
    out["tangency_err_p90"] = pct(terr, 90)
    out["tangency_err_max"] = float(terr.max())
    out["inflections_correct_trials"] = int(sum(r["inflections_correct"] for r in rows))
    out["trials_with_points_over"] = int(sum(r["points_over"] > 0 for r in rows))
    return out


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description="LE/TE detection and tangency selection.")
    ap.add_argument("csv", nargs="?", help="contour points ordered around the section: x,y")
    ap.add_argument("--tol", type=float, default=0.00015)
    ap.add_argument("--le-radius", type=float, help="LE_R from the drawing (fixes the LE radius)")
    ap.add_argument("--te-radius", type=float, help="TE_R from the drawing (fixes the TE radius)")
    ap.add_argument("--grid", type=int, default=9, help="tangency search grid per angle")
    ap.add_argument("--out", help="write the result as JSON")
    ap.add_argument("--demo", action="store_true", help="run on a synthetic section with known answer")
    ap.add_argument("--trials", type=int,
                    help="repeat the synthetic comparison over random spacing and rounding")
    args = ap.parse_args(argv)

    if args.trials:
        for use_radii in (False, True):
            summary = summarize_trials(trials(args.trials, args.tol, use_radii, args.grid))
            print(f"\nRadius {'fixed from drawing' if use_radii else 'fitted'}:")
            print(json.dumps(summary, indent=2))
        return 0

    if args.demo:
        sec = synthetic_section()
        for use_radii in (False, True):
            result, rows, terr = compare_on_synthetic(sec, args.tol, use_radii, args.grid)
            print(f"\nSynthetic section, radius {'fixed from drawing' if use_radii else 'fitted'}:")
            for r in rows:
                print(f"  {r['edge']}: true R {r['radius_true']:.4f}  "
                      f"3-point R {r['radius_three_point']:.4f}  LSQ R {r['radius_lsq']:.4f}  "
                      f"center err {r['center_err_lsq']:.5f}  "
                      f"window {r['window_points']} pts ({r['window_false']} side pts)  "
                      f"0.001 threshold: {r['threshold_false']} side pts misclassified")
            print("  tangency position errors: " + ", ".join(f"{e:.4f}" for e in terr))
            for s in result.sides:
                print(f"  side {s.start_edge}->{s.end_edge}: {s.fit.inflections()} inflections, "
                      f"{s.junction_inflections} junction inflections, "
                      f"max dev {s.fit.max_dev:.6f}, {s.points_over} points over")
        return 0

    if not args.csv:
        ap.error("give a CSV file or --demo")
    pts = ab.read_points(args.csv)
    result = reconstruct_section(pts, args.tol, args.le_radius, args.te_radius, args.grid)
    for e in result.edges:
        print(f"{e.label}: center ({e.center[0]:.5f}, {e.center[1]:.5f})  R {e.radius:.5f}"
              f"{' (fixed)' if e.radius_fixed else ''}  window {len(e.window)} pts  "
              f"max residual {e.max_residual:.6f}")
    for s in result.sides:
        print(f"side {s.start_edge}->{s.end_edge}: tangency "
              f"({s.p_start[0]:.5f}, {s.p_start[1]:.5f}) -> ({s.p_end[0]:.5f}, {s.p_end[1]:.5f}); "
              f"{s.fit.inflections()} inflections, {s.junction_inflections} at junctions, "
              f"max dev {s.fit.max_dev:.6f}, {s.points_over} points over")
    if args.out:
        with open(args.out, "w") as f:
            json.dump(result.to_dict(), f, indent=2)
        print(f"Wrote {args.out}")
    return 0 if all(s.points_over == 0 for s in result.sides) else 1


if __name__ == "__main__":
    sys.exit(main())
