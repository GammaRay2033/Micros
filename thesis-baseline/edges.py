#!/usr/bin/env python3
"""LE/TE detection, edge fitting and tangency selection for one section.

A replacement for Algorithms 2-3 of the thesis:

1. Edges from curvature. A lightly smoothed periodic spline through the
   ordered contour; the LE and TE are its two strongest convex curvature peaks.
2. Edge shapes by geometric least squares, as circles or ellipses. Starting
   from a few points around each peak, neighbours are added while the window
   stays within the deviation tolerance of the fitted shape.
3. Drawing constraints. A circle can take its radius from the drawing (LE_R /
   TE_R). An ellipse can take its axis ratio, its semi-minor axis (half the
   thickness at the joints) and its orientation (a given angle, or the
   outward normal at the curvature peak).
4. Tangency by fit quality. Each tangency point is a parameter on its edge
   shape (the angle on a circle, the eccentric anomaly on an ellipse),
   searched near the end of the edge window. For each candidate pair, the side
   is fitted with the baseline (airfoil_baseline.fit_side) and scored by:
   points over tolerance, then inflections (including at the joints), then
   maximum deviation. Elliptical edges can optionally require G2 joints, where
   the side's curvature matches the ellipse's.

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

# Edge windows grow while every point is within GROW_FRAC * tol of the shape.
# Below 1, side points that happen to lie within tolerance near a tangency are
# less likely to join the window; rounding to 4 decimals moves true edge
# points by at most ~0.00007, so 0.5 * 0.00015 still admits them.
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
# Edge shapes
# ---------------------------------------------------------------------------
#
# Both shapes are parametrized counter-clockwise by t, so a contour travelling
# counter-clockwise (orient = +1) crosses an edge with increasing t.

@dataclass
class Circle:
    center: np.ndarray
    radius: float
    kind = "circle"

    def point(self, t):
        return circle_point(self.center, self.radius, t)

    def tangent(self, t, orient):
        return circle_tangent(t, orient)

    def curvature(self, t):
        return 1.0 / self.radius

    def param_of(self, p):
        d = np.asarray(p, float) - self.center
        return math.atan2(d[1], d[0])

    def distance(self, pts):
        pts = np.atleast_2d(pts)
        return np.abs(np.hypot(*(pts - self.center).T) - self.radius)

    @property
    def size(self):
        """Half-width of the edge, used to tell the LE from the TE."""
        return self.radius

    def to_dict(self):
        return {"kind": "circle", "center": self.center.tolist(), "radius": self.radius}


@dataclass
class Ellipse:
    center: np.ndarray
    a: float            # semi-major axis, along the edge's direction
    b: float            # semi-minor axis
    phi: float          # direction of the major axis, pointing to the tip

    kind = "ellipse"

    @property
    def u(self):
        return np.array([math.cos(self.phi), math.sin(self.phi)])

    @property
    def v(self):
        return np.array([-math.sin(self.phi), math.cos(self.phi)])

    def point(self, t):
        return self.center + self.a * math.cos(t) * self.u + self.b * math.sin(t) * self.v

    def derivative(self, t):
        return -self.a * math.sin(t) * self.u + self.b * math.cos(t) * self.v

    def tangent(self, t, orient):
        return orient * ab.unit(self.derivative(t))

    def curvature(self, t):
        a, b = self.a, self.b
        return a * b / (a * a * math.sin(t) ** 2 + b * b * math.cos(t) ** 2) ** 1.5

    def _local(self, pts):
        d = np.atleast_2d(pts) - self.center
        return d @ self.u, d @ self.v

    def params_of(self, pts, n_coarse=720, newton_iters=20):
        """Closest-point eccentric anomaly for each point."""
        x, y = self._local(pts)
        a, b = self.a, self.b
        ts = np.linspace(-math.pi, math.pi, n_coarse, endpoint=False)
        d2 = (a * np.cos(ts)[None] - x[:, None]) ** 2 + (b * np.sin(ts)[None] - y[:, None]) ** 2
        t = ts[np.argmin(d2, axis=1)]
        for _ in range(newton_iters):
            s, c = np.sin(t), np.cos(t)
            f = (b * b - a * a) * s * c + a * x * s - b * y * c
            fp = (b * b - a * a) * (c * c - s * s) + a * x * c + b * y * s
            t = t - np.where(np.abs(fp) > 1e-30, f / np.where(np.abs(fp) > 1e-30, fp, 1.0), 0.0)
        return t

    def param_of(self, p):
        return float(self.params_of(p)[0])

    def signed_distance(self, pts):
        x, y = self._local(pts)
        t = self.params_of(pts)
        d = np.hypot(self.a * np.cos(t) - x, self.b * np.sin(t) - y)
        inside = (x / self.a) ** 2 + (y / self.b) ** 2 < 1.0
        return np.where(inside, -d, d)

    def distance(self, pts):
        return np.abs(self.signed_distance(pts))

    @property
    def size(self):
        return self.b

    @property
    def tip_radius(self):
        return self.b * self.b / self.a

    def to_dict(self):
        return {"kind": "ellipse", "center": self.center.tolist(), "a": self.a,
                "b": self.b, "phi_deg": math.degrees(self.phi),
                "ratio": self.a / self.b, "tip_radius": self.tip_radius}


# ---------------------------------------------------------------------------
# Step 1: edges from curvature
# ---------------------------------------------------------------------------

def curvature_peaks(pts, tol, min_separation=0.25, oversample=20, normals=False):
    """Contour indices of the two strongest convex curvature peaks.

    Returns (indices, orient), where orient is +1 for a counter-clockwise
    contour and -1 for clockwise. With normals=True, also returns the outward
    unit normal of the smoothed contour at each peak.
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

    peaks, outward = [], []
    for k in (first, second):
        x, y = splev(us[k], tck)
        peaks.append(int(np.argmin(np.hypot(pts[:, 0] - x, pts[:, 1] - y))))
        t = ab.unit([dx[k], dy[k]])
        outward.append(orient * np.array([t[1], -t[0]]))
    if normals:
        return peaks, orient, outward
    return peaks, orient


# ---------------------------------------------------------------------------
# Step 2-3: fitting edge shapes
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


def fitzgibbon_ellipse(points):
    """Direct least-squares ellipse (Fitzgibbon; Halir & Flusser's stable form).

    Algebraic, so biased on partial arcs: used only as a starting point.
    Returns an Ellipse, or None if the fit does not give one.
    """
    p = np.asarray(points, dtype=float)
    if len(p) < 6:
        return None
    mean = p.mean(axis=0)
    scale = np.abs(p - mean).max() or 1.0
    x, y = ((p - mean) / scale).T
    D1 = np.column_stack([x * x, x * y, y * y])
    D2 = np.column_stack([x, y, np.ones_like(x)])
    S1, S2, S3 = D1.T @ D1, D1.T @ D2, D2.T @ D2
    try:
        T = -np.linalg.solve(S3, S2.T)
    except np.linalg.LinAlgError:
        return None
    M = S1 + S2 @ T
    M = np.array([M[2] / 2, -M[1], M[0] / 2])
    w, vecs = np.linalg.eig(M)
    vecs = np.real(vecs)
    cond = 4 * vecs[0] * vecs[2] - vecs[1] ** 2
    ok = np.flatnonzero(cond > 0)
    if len(ok) == 0:
        return None
    a1 = vecs[:, ok[0]]
    A, B, C = a1
    D, E, F = T @ a1
    den = B * B - 4 * A * C
    if den >= 0:
        return None
    cx = (2 * C * D - B * E) / den
    cy = (2 * A * E - B * D) / den
    F0 = A * cx * cx + B * cx * cy + C * cy * cy + D * cx + E * cy + F
    evals, evecs = np.linalg.eigh(np.array([[A, B / 2], [B / 2, C]]))
    if F0 == 0 or np.any(-F0 / evals <= 0):
        return None
    axes = np.sqrt(-F0 / evals)
    i_major = int(np.argmax(axes))
    major = evecs[:, i_major]
    return Ellipse(center=mean + scale * np.array([cx, cy]),
                   a=float(axes[i_major] * scale), b=float(axes[1 - i_major] * scale),
                   phi=math.atan2(major[1], major[0]))


def _orient_to_tip(e, points):
    """Point the major axis at the data, so the tip is at t = 0."""
    if (np.mean(points, axis=0) - e.center) @ e.u < 0:
        e.phi = wrap_angle(e.phi + math.pi)
    return e


def fit_ellipse(points, ratio=None, minor=None, phi=None, init=None):
    """Geometric (orthogonal-distance) least-squares ellipse.

    Optional drawing constraints fix the axis ratio a/b, the semi-minor axis b,
    and/or the major-axis direction phi (pointing to the tip). Several starting
    points are tried (Fitzgibbon, circle-based, `init`), keeping the best.
    Returns (Ellipse, max_residual).
    """
    p = np.asarray(points, dtype=float)
    span = float(np.ptp(p, axis=0).max())
    n_free = 2 + (minor is None) + (ratio is None) + (phi is None)
    if len(p) < n_free:
        raise ValueError(f"need at least {n_free} points, got {len(p)}")

    def unpack(z):
        k = 2
        cx, cy = z[0], z[1]
        if minor is None:
            b = z[k]
            k += 1
        else:
            b = minor
        if ratio is None:
            a = z[k]
            k += 1
        else:
            a = ratio * b
        ang = z[k] if phi is None else phi
        return Ellipse(center=np.array([cx, cy]), a=float(a), b=float(b), phi=float(ang))

    def pack(e):
        z = [e.center[0], e.center[1]]
        if minor is None:
            z.append(e.b)
        if ratio is None:
            z.append(e.a)
        if phi is None:
            z.append(e.phi)
        return np.array(z, dtype=float)

    lo = [-np.inf, -np.inf] + [1e-6 * span] * ((minor is None) + (ratio is None)) + \
        ([-np.inf] if phi is None else [])
    hi = [np.inf, np.inf] + [10 * span] * ((minor is None) + (ratio is None)) + \
        ([np.inf] if phi is None else [])

    # Starting points.
    c_c, r_c, _ = fit_circle(p)
    tip_dir = ab.unit(p.mean(axis=0) - c_c) if np.linalg.norm(p.mean(axis=0) - c_c) > 0 \
        else np.array([1.0, 0.0])
    ang0 = phi if phi is not None else math.atan2(tip_dir[1], tip_dir[0])
    u0 = np.array([math.cos(ang0), math.sin(ang0)])
    tip = c_c + r_c * u0
    starts = []
    for k in ([ratio] if ratio is not None else [1.0, 2.0, 3.0]):
        for b0 in ([minor] if minor is not None else [r_c, k * r_c]):
            a0 = k * b0
            starts.append(Ellipse(center=tip - a0 * u0, a=a0, b=b0, phi=ang0))
    fz = fitzgibbon_ellipse(p) if (ratio is None and minor is None and phi is None) else None
    if fz is not None and np.all(np.isfinite([fz.a, fz.b])) and fz.a < 10 * span:
        starts.append(_orient_to_tip(fz, p))
    if init is not None:
        starts.append(init)

    best = None
    for e0 in starts:
        z0 = np.clip(pack(e0), np.array(lo) + 1e-15, np.array(hi) - 1e-15)
        try:
            res = least_squares(lambda z: unpack(z).signed_distance(p), z0,
                                bounds=(lo, hi), method="trf", x_scale="jac")
        except ValueError:
            continue
        if best is None or res.cost < best.cost:
            best = res
    e = unpack(best.x)
    if ratio is None and minor is None and e.a < e.b:
        e = Ellipse(center=e.center, a=e.b, b=e.a, phi=e.phi + math.pi / 2)
    if phi is None:
        e = _orient_to_tip(e, p)
    e.phi = wrap_angle(e.phi)
    return e, float(e.distance(p).max())


@dataclass
class EdgeSpec:
    """What the drawing says about one edge.

    kind: "circle", "ellipse" or "auto" (choose by BIC).
    radius: circle radius. ratio / minor: ellipse a/b and semi-minor axis.
    axis: ellipse major-axis direction in radians, or "tip" to use the
    outward normal at the curvature peak.
    """
    kind: str = "circle"
    radius: float = None
    ratio: float = None
    minor: float = None
    axis: object = None

    def n_params(self):
        if self.kind == "circle":
            return 3 - (self.radius is not None)
        return 5 - sum(x is not None for x in (self.ratio, self.minor, self.axis))


def fit_shape(points, spec, axis_angle=None, init=None):
    """Fit `spec`'s shape to `points`. Returns (shape, max_residual)."""
    if spec.kind == "circle":
        c, r, res = fit_circle(points, spec.radius,
                               init=None if init is None else (init.center, init.size))
        return Circle(center=c, radius=r), res
    return fit_ellipse(points, spec.ratio, spec.minor, axis_angle,
                       init=init if isinstance(init, Ellipse) else None)


@dataclass
class Edge:
    seed: int
    window: list          # contour indices on the edge, in travel order
    shape: object         # Circle or Ellipse
    max_residual: float
    spec: EdgeSpec = field(default_factory=EdgeSpec)
    label: str = ""
    axis_angle: float = None

    @property
    def center(self):
        return self.shape.center

    @property
    def radius(self):
        """Circle radius, or the ellipse's tip radius of curvature."""
        return self.shape.radius if self.shape.kind == "circle" else self.shape.tip_radius

    @property
    def radius_fixed(self):
        return self.spec.kind == "circle" and self.spec.radius is not None

    def to_dict(self):
        d = {"label": self.label, "window": [int(i) for i in self.window],
             "max_residual": self.max_residual, "radius_fixed": self.radius_fixed}
        d.update(self.shape.to_dict())
        return d


def grow_edge(pts, seed, tol, radius=None, grow_tol=None, start_half=None,
              max_frac=0.25, max_passes=4, spec=None, axis_angle=None):
    """Fit an edge shape around `seed`, growing the window while it fits.

    The window must stay within `tol`, and each point added to it must itself
    lie within `grow_tol` (default `tol`) of the refitted shape.

    The curvature peak is often off-center on the edge, so a starting window
    around it can include side points. After each pass the seed moves to the
    middle of the window and the window is regrown, until it stops changing.
    """
    if spec is None:
        spec = EdgeSpec(kind="circle", radius=radius)
    n = len(pts)
    edge = None
    for _ in range(max_passes):
        edge = _grow_once(pts, seed, tol, grow_tol if grow_tol is not None else tol,
                          spec, axis_angle, start_half, max_frac)
        middle = edge.window[len(edge.window) // 2]
        if middle == seed % n:
            break
        seed = middle
    return edge


def _grow_once(pts, seed, tol, grow_tol, spec, axis_angle, start_half, max_frac):
    n = len(pts)
    n_par = spec.n_params()
    min_half = max(1, math.ceil((n_par - 1) / 2))
    half = start_half if start_half is not None else max(2, math.ceil(n_par / 2))
    max_len = max(2 * min_half + 1, int(max_frac * n))

    while True:
        lo, hi = seed - half, seed + half
        idx = [i % n for i in range(lo, hi + 1)]
        shape, res = fit_shape(pts[idx], spec, axis_angle)
        if res <= tol or half <= min_half:
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
            shape2, res2 = fit_shape(pts[idx], spec, axis_angle, init=shape)
            new = pts[(nlo if side == "left" else nhi) % n]
            new_res = float(shape2.distance(new)[0])
            if res2 <= tol and new_res <= grow_tol:
                lo, hi, shape, res = nlo, nhi, shape2, res2
            elif side == "left":
                grow_left = False
            else:
                grow_right = False

    window = [i % n for i in range(lo, hi + 1)]
    return Edge(seed=seed % n, window=window, shape=shape, max_residual=res,
                spec=spec, axis_angle=axis_angle)


# Residuals below NOISE_FRAC * tol are treated as noise when choosing between
# a circle and an ellipse (about the rounding noise of 4-decimal coordinates
# for the default tolerance).
NOISE_FRAC = 0.25

# "auto" picks an ellipse only when its fitted axis ratio reaches this. On
# synthetic sections, ellipses fitted to circular edges reached 1.00-1.21
# (they stretch slightly to absorb side points), while a true 2.4:1 LE fitted
# at 2.39-2.42. Real elliptical LEs are usually 1.5:1 to 4:1.
AUTO_MIN_RATIO = 1.3


def _bic(residuals, k, sigma):
    """BIC with the mean squared residual floored at sigma^2: a model can't
    earn credit for fitting below the noise level (with few points, an
    ellipse passes through all of them exactly)."""
    n = len(residuals)
    mse = max(float(np.mean(np.asarray(residuals) ** 2)), sigma ** 2)
    return n * math.log(mse) + k * math.log(n)


def _fit_auto(pts, seed, tol, grow_tol, spec, axis_angle):
    """Fit both a circle and an ellipse. Keep the ellipse only if it is
    clearly elongated (a/b >= AUTO_MIN_RATIO) and BIC prefers it on the
    ellipse's window, where the circle has to fit the same points.

    The ratio test is needed because an ellipse grown on a circular edge also
    absorbs a few smoothly joined side points that the circle can't, which
    BIC alone counts as evidence for the ellipse."""
    circ_spec = EdgeSpec(kind="circle", radius=spec.radius)
    ell_spec = EdgeSpec(kind="ellipse", ratio=spec.ratio, minor=spec.minor, axis=spec.axis)
    circ = grow_edge(pts, seed, tol, grow_tol=grow_tol, spec=circ_spec)
    ell = grow_edge(pts, seed, tol, grow_tol=grow_tol, spec=ell_spec, axis_angle=axis_angle)
    W = pts[ell.window]
    c_on_w, _ = fit_shape(W, circ_spec)
    sigma = NOISE_FRAC * tol
    bic_c = _bic(c_on_w.distance(W), circ_spec.n_params(), sigma)
    bic_e = _bic(ell.shape.signed_distance(W), ell_spec.n_params(), sigma)
    elongated = ell.shape.a / ell.shape.b >= AUTO_MIN_RATIO
    return ell if elongated and bic_e < bic_c else circ


def local_width(pts, idx, frac=0.03):
    """Distance across the contour at +-frac of its length around index idx."""
    n = len(pts)
    seg = np.linalg.norm(np.diff(np.vstack([pts, pts[:1]]), axis=0), axis=1)
    L = seg.sum()
    s = np.concatenate([[0], np.cumsum(seg)])
    closed = np.vstack([pts, pts[:1]])

    def at(arc):
        arc %= L
        return np.array([np.interp(arc, s, closed[:, 0]), np.interp(arc, s, closed[:, 1])])

    return float(np.linalg.norm(at(s[idx] + frac * L) - at(s[idx] - frac * L)))


def _as_spec(x):
    if x is None:
        return EdgeSpec()
    if isinstance(x, EdgeSpec):
        return x
    return EdgeSpec(kind="circle", radius=float(x))


LABEL_RULES = ("radius", "width", "min-x", "max-x", "min-y", "max-y")


def detect_edges(pts, tol, le=None, te=None, grow_frac=GROW_FRAC, le_radius=None,
                 te_radius=None, label_by="radius"):
    """Find, fit and label the two edges. Returns (edges, orient).

    le / te: an EdgeSpec, a circle radius from the drawing, or None (free
    circle). le_radius / te_radius are shorthands for circles with drawing
    radii.

    Every edge is first grown as a free circle, which also moves its seed to
    the middle of the edge; the requested shape is then grown from that seed.

    label_by decides which edge is the LE:
      "radius": with drawing radii for two circles, the closer match;
                otherwise the larger fitted circle (right for circular edges,
                wrong for a sharp elliptical LE, whose tip radius is small).
      "width":  the edge whose contour is wider nearby.
      "min-x", "max-x", "min-y", "max-y": the edge at that extreme of the
                drawing's coordinates (e.g. min-x when x runs downstream).
    """
    if label_by not in LABEL_RULES:
        raise ValueError(f"label_by must be one of {LABEL_RULES}")
    le_spec = _as_spec(le if le is not None else le_radius)
    te_spec = _as_spec(te if te is not None else te_radius)
    peaks, orient, outward = curvature_peaks(pts, tol, normals=True)
    grow_tol = grow_frac * tol
    free = [grow_edge(pts, s, tol, grow_tol=grow_tol) for s in peaks]

    if label_by == "radius":
        if all(s.kind == "circle" and s.radius is not None for s in (le_spec, te_spec)):
            direct = abs(free[0].radius - le_spec.radius) + abs(free[1].radius - te_spec.radius)
            swapped = abs(free[0].radius - te_spec.radius) + abs(free[1].radius - le_spec.radius)
            first_is_le = direct <= swapped
        else:
            first_is_le = free[0].radius >= free[1].radius
    elif label_by == "width":
        first_is_le = local_width(pts, free[0].seed) >= local_width(pts, free[1].seed)
    else:
        axis = 0 if label_by.endswith("x") else 1
        c0, c1 = pts[free[0].seed, axis], pts[free[1].seed, axis]
        first_is_le = c0 <= c1 if label_by.startswith("min") else c0 >= c1
    labels = ("LE", "TE") if first_is_le else ("TE", "LE")

    specs = {"LE": le_spec, "TE": te_spec}
    edges = []
    for f, normal, label in zip(free, outward, labels):
        spec = specs[label]
        axis_angle = math.atan2(normal[1], normal[0]) if spec.axis == "tip" else spec.axis
        if spec.kind == "circle" and spec.radius is None:
            e = f
        elif spec.kind == "auto":
            e = _fit_auto(pts, f.seed, tol, grow_tol, spec, axis_angle)
        else:
            e = grow_edge(pts, f.seed, tol, grow_tol=grow_tol, spec=spec, axis_angle=axis_angle)
        e.label = label
        edges.append(e)
    return edges, orient


# ---------------------------------------------------------------------------
# Step 4: tangency by fit quality
# ---------------------------------------------------------------------------

@dataclass
class _End:
    """Search range for one tangency, and the window points it may hand to the side.

    The window can hold side points that happen to lie within tolerance of the
    edge shape, so the range starts `reach` points inside the window and ends
    at the first point outside it. Window points past the chosen tangency (in
    the direction away from the edge) are fitted as side points.
    """
    lo: float           # parameter range, in travel order
    hi: float
    idx: list           # window points inside the range, in travel order
    ang: np.ndarray     # their parameters, unwrapped consistently with lo/hi
    end: str


def _default_reach(edge):
    # An ellipse can absorb more of a smoothly joined side than a circle.
    return 2 if edge.shape.kind == "circle" else max(2, len(edge.window) // 3)


def _end_range(edge, pts, end, orient, n, reach=None):
    reach = _default_reach(edge) if reach is None else reach
    reach = max(0, min(reach, len(edge.window) - 2))
    par = edge.shape.param_of
    if end == "out":
        idx = edge.window[len(edge.window) - 1 - reach:]
        outside = (edge.window[-1] + 1) % n
        ref = par(pts[idx[0]])
        ang = np.array([ref + wrap_angle(par(pts[i]) - ref) for i in idx])
        a_out = ref + wrap_angle(par(pts[outside]) - ref)
        return _End(lo=ref, hi=a_out, idx=list(idx), ang=ang, end=end)
    idx = edge.window[:reach + 1]
    outside = (edge.window[0] - 1) % n
    ref = par(pts[idx[-1]])
    ang = np.array([ref + wrap_angle(par(pts[i]) - ref) for i in idx])
    a_out = ref + wrap_angle(par(pts[outside]) - ref)
    return _End(lo=a_out, hi=ref, idx=list(idx), ang=ang, end=end)


def _side_points(pts, outside_idx, start, end, th0, th1, orient):
    """Side points for tangency parameters th0 (leaving start edge) and th1
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
    g2_start: bool = False
    g2_end: bool = False

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
            "g2_start": self.g2_start,
            "g2_end": self.g2_end,
            "junction_inflections": self.junction_inflections,
            "curvature_jump_start": self.curvature_jump_start,
            "curvature_jump_end": self.curvature_jump_end,
            "points_over_tolerance": self.points_over,
        })
        return d


def _joint(e, th, orient, g2):
    """Point, tangent and (for G2) signed target curvature at parameter th."""
    k = orient * e.shape.curvature(th)
    return e.shape.point(th), e.shape.tangent(th, orient), (k if g2 else None), k


def _evaluate_side(Q, e0, e1, th0, th1, orient, m, lam, tol, kappa_eps,
                   param_iters=1, fit=None, g2=(False, False)):
    p0, t0, ks, k0 = _joint(e0, th0, orient, g2[0])
    p1, t1, ke, k1 = _joint(e1, th1, orient, g2[1])
    if fit is None:
        m = min(m, len(Q) + 2)
        fit = ab.fit_side(Q, p0, p1, t0, t1, m, lam, param_iters=param_iters,
                          k_start=ks, k_end=ke)
    k_start, k_end = fit.end_curvatures()
    # A joint is an inflection when the side turns the other way from the edge.
    # Near-zero side curvature is ambiguous, so it must exceed JUNCTION_EPS.
    junction = int(orient * k_start < -JUNCTION_EPS) + int(orient * k_end < -JUNCTION_EPS)
    over = int(np.count_nonzero(fit.deviations > tol))
    score = (over, fit.inflections(kappa_eps) + junction, fit.max_dev)
    return score, fit, p0, p1, junction, abs(k_start - k0) / abs(k0), abs(k_end - k1) / abs(k1)


def choose_side_tangency(pts, Q_idx, e0, e1, orient, tol, grid=9,
                         kappa_eps=1e-9, g2=False):
    """Search both tangency parameters of one side on a grid over their ranges.

    The control-point count and smoothing are chosen once at the middle of the
    ranges; the chosen tangencies then get a full fit-to-tolerance search.
    With g2, joints at elliptical edges also match the ellipse's curvature.
    """
    n = len(pts)
    r0 = _end_range(e0, pts, "out", orient, n)
    r1 = _end_range(e1, pts, "in", orient, n)
    g2_ends = (g2 and e0.shape.kind == "ellipse", g2 and e1.shape.kind == "ellipse")

    def side_fit_to_tol(th0, th1):
        Q = _side_points(pts, Q_idx, r0, r1, th0, th1, orient)
        p0, t0, ks, _ = _joint(e0, th0, orient, g2_ends[0])
        p1, t1, ke, _ = _joint(e1, th1, orient, g2_ends[1])
        fit, _ = ab.fit_to_tolerance(Q, p0, p1, t0, t1, tol, kappa_eps=kappa_eps,
                                     k_start=ks, k_end=ke)
        return Q, fit

    _, ref = side_fit_to_tol(0.5 * (r0.lo + r0.hi), 0.5 * (r1.lo + r1.hi))
    m, lam = ref.n_ctrl, ref.lam

    best = None
    for th0 in np.linspace(r0.lo, r0.hi, grid):
        for th1 in np.linspace(r1.lo, r1.hi, grid):
            Q = _side_points(pts, Q_idx, r0, r1, th0, th1, orient)
            try:
                score = _evaluate_side(Q, e0, e1, th0, th1, orient, m, lam, tol,
                                       kappa_eps, g2=g2_ends)[0]
            except ValueError:
                continue
            if best is None or score < best[0]:
                best = (score, th0, th1)
    _, th0, th1 = best
    Q, fit = side_fit_to_tol(th0, th1)
    _, fit, p0, p1, junction, jump0, jump1 = _evaluate_side(
        Q, e0, e1, th0, th1, orient, None, None, tol, kappa_eps, fit=fit, g2=g2_ends)
    return SideResult(start_edge=e0.label, end_edge=e1.label,
                      theta_start=float(th0), theta_end=float(th1),
                      p_start=p0, p_end=p1, fit=fit,
                      junction_inflections=junction,
                      curvature_jump_start=float(jump0),
                      curvature_jump_end=float(jump1), tol=tol,
                      g2_start=g2_ends[0], g2_end=g2_ends[1])


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
                        grid=9, kappa_eps=1e-9, grow_frac=GROW_FRAC, le=None,
                        te=None, g2=False, label_by="radius"):
    """Detect and fit both edges, then choose all four tangencies.

    le / te: EdgeSpec for each edge (le_radius / te_radius are shorthands for
    circles with drawing radii). g2: require curvature-matching joints at
    elliptical edges.
    """
    pts = np.asarray(pts, dtype=float)
    le = le if le is not None else le_radius
    te = te if te is not None else te_radius
    edges, orient = detect_edges(pts, tol, le, te, grow_frac, label_by=label_by)
    a, b = edges
    n = len(pts)
    sides = [
        choose_side_tangency(pts, side_indices(n, a, b), a, b, orient, tol, grid,
                             kappa_eps, g2),
        choose_side_tangency(pts, side_indices(n, b, a), b, a, orient, tol, grid,
                             kappa_eps, g2),
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
# Synthetic sections with known answer
# ---------------------------------------------------------------------------

@dataclass
class SyntheticSection:
    pts: np.ndarray
    centers: dict
    radii: dict
    tangency: dict      # label -> list of true tangency points
    on_arc: dict        # label -> contour indices that truly lie on the edge
    true_inflections: dict = field(default_factory=dict)  # "LE->TE" -> count
    shapes: dict = field(default_factory=dict)            # label -> true shape


def _bezier(P, s, der=0):
    P = np.asarray(P, dtype=float)
    n = len(P) - 1
    for _ in range(der):
        P = n * np.diff(P, axis=0)
        n -= 1
    B = np.stack([math.comb(n, i) * s ** i * (1 - s) ** (n - i) for i in range(n + 1)], 1)
    return B @ P


def _bezier_inflections(P, eps=1e-6):
    s = np.linspace(0, 1, 20001)
    d1, d2 = _bezier(P, s, 1), _bezier(P, s, 2)
    k = ab.cross2(d1, d2) / np.linalg.norm(d1, axis=1) ** 3
    return ab.count_sign_changes(k, eps)


def _sample_side(P, k):
    """k points on a Bezier side, by arc length with cosine clustering at the ends."""
    s = np.linspace(0, 1, 8001)
    c = _bezier(P, s)
    L = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(c, axis=0), axis=1))])
    targets = L[-1] * 0.5 * (1 - np.cos(np.pi * np.linspace(0, 1, k + 2)[1:-1]))
    return np.column_stack([np.interp(targets, L, c[:, 0]), np.interp(targets, L, c[:, 1])])


def _sample_edge(shape, t0, t1, spacing):
    """Points on an edge between parameters t0 < t1, at about `spacing` apart by
    arc length, half a spacing in from each end (no tangency is a data point)."""
    ts = np.linspace(t0, t1, 4001)
    c = np.array([shape.point(t) for t in ts])
    L = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(c, axis=0), axis=1))])
    k = max(2, int(round(L[-1] / spacing)))
    targets = (np.arange(k) + 0.5) * L[-1] / k
    return np.column_stack([np.interp(targets, L, c[:, 0]), np.interp(targets, L, c[:, 1])])


def synthetic_section(le_r=0.05, te_r=0.02, edge_spacing=0.01, side_points=24,
                      decimals=4, seed=None):
    """A cambered section with circular edges and a known answer, sampled and
    rounded like a drawing.

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

    tan = lambda th: np.array([-math.sin(th), math.cos(th)])
    p_le_in, p_le_out = circle_point(c_le, le_r, le_in), circle_point(c_le, le_r, le_out)
    p_te_in, p_te_out = circle_point(c_te, te_r, te_in), circle_point(c_te, te_r, te_out)

    ps_poles = [p_le_out, p_le_out + 0.28 * tan(le_out), [0.579, -0.093], [1.006, 0.189],
                p_te_in - 0.14 * tan(te_in), p_te_in]
    ss_poles = [p_te_out, p_te_out + 0.2 * tan(te_out), p_le_in - 0.8 * tan(le_in), p_le_in]

    le_arc = arc(c_le, le_r, le_in, le_out)
    ps = _sample_side(ps_poles, side_points)
    te_arc = arc(c_te, te_r, te_in, te_out)
    ss = _sample_side(ss_poles, side_points)

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
        on_arc=on_arc, true_inflections={"LE->TE": 2, "TE->LE": 0},
        shapes={"LE": Circle(c_le, le_r), "TE": Circle(c_te, te_r)})


# Shape of the synthetic elliptical-LE section (found by search so that the
# suction side is convex and the pressure side has exactly 2 inflections,
# away from the joints).
_ELL_LE = dict(center=(0.0, 0.0), a=0.06, b=0.025, phi_deg=200.0)
_ELL_TE = dict(center=(1.5, 0.25), r=0.02, t_in_deg=-79.0, t_out_deg=65.0)
_ELL_PS = dict(h1=0.03, h2=0.173, mid=((0.295, 0.129), (0.732, 0.189)), h5=0.361)
_ELL_SS = dict(h1=0.032, h2=0.483, mid=((0.975, 0.326), (0.668, 0.237)), h5=0.356)


def _g2_poles(p0, t0, kappa, h1, h2, degree):
    """First three Bezier poles leaving p0 along t0 with signed curvature kappa."""
    n0 = np.array([-t0[1], t0[0]])
    height = kappa * h1 ** 2 * degree / (degree - 1)
    return [p0, p0 + h1 * t0, p0 + h2 * t0 + height * n0]


def synthetic_ellipse_section(edge_spacing=0.008, side_points=24, decimals=4,
                              seed=None):
    """A section with an elliptical LE (a/b = 2.4) and a circular TE.

    The sides join the ellipse at its co-vertices (t = +-90 degrees, the
    common design) with matching curvature (G2), and the TE circle with G1.
    Contour runs counter-clockwise: LE, pressure side, TE, suction side.
    """
    le = Ellipse(center=np.array(_ELL_LE["center"]), a=_ELL_LE["a"], b=_ELL_LE["b"],
                 phi=math.radians(_ELL_LE["phi_deg"]))
    te = Circle(center=np.array(_ELL_TE["center"]), radius=_ELL_TE["r"])
    le_in, le_out = -math.pi / 2, math.pi / 2
    te_in, te_out = math.radians(_ELL_TE["t_in_deg"]), math.radians(_ELL_TE["t_out_deg"])
    k_le = le.curvature(le_out)

    # Pressure side: degree 6, G2 at the LE (leaving), G1 at the TE.
    p0, t0 = le.point(le_out), le.tangent(le_out, 1.0)
    p6, t6 = te.point(te_in), te.tangent(te_in, 1.0)
    ps_poles = (_g2_poles(p0, t0, k_le, _ELL_PS["h1"], _ELL_PS["h2"], 6)
                + [np.array(m) for m in _ELL_PS["mid"]] + [p6 - _ELL_PS["h5"] * t6, p6])
    # Suction side: degree 6, G1 at the TE (leaving), G2 at the LE (arriving).
    q0, s0 = te.point(te_out), te.tangent(te_out, 1.0)
    q5, s5 = le.point(le_in), le.tangent(le_in, 1.0)
    # Build the LE end reversed: leaving q5 along -s5, curvature sign flipped.
    end = _g2_poles(q5, -s5, -k_le, _ELL_SS["h1"], _ELL_SS["h2"], 6)
    ss_poles = [q0, q0 + _ELL_SS["h5"] * s0] + [np.array(m) for m in _ELL_SS["mid"]] + end[::-1]

    le_pts = _sample_edge(le, le_in, le_out, edge_spacing)
    ps = _sample_side(ps_poles, side_points)
    te_pts = _sample_edge(te, te_in, te_out, edge_spacing)
    ss = _sample_side(ss_poles, side_points)

    pts = np.vstack([le_pts, ps, te_pts, ss])
    if seed is not None:
        rng = np.random.default_rng(seed)
        pts = pts + rng.uniform(-0.5, 0.5, pts.shape) * 10.0 ** -decimals
    pts = np.round(pts, decimals)

    n_le, n_ps, n_te = len(le_pts), len(ps), len(te_pts)
    on_arc = {"LE": list(range(0, n_le)),
              "TE": list(range(n_le + n_ps, n_le + n_ps + n_te))}
    return SyntheticSection(
        pts=pts, centers={"LE": le.center, "TE": te.center},
        radii={"LE": le.tip_radius, "TE": te.radius},
        tangency={"LE": [le.point(le_in), le.point(le_out)],
                  "TE": [te.point(te_in), te.point(te_out)]},
        on_arc=on_arc,
        true_inflections={"LE->TE": _bezier_inflections(ps_poles),
                          "TE->LE": _bezier_inflections(ss_poles)},
        shapes={"LE": le, "TE": te})


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


# Configurations compared on the elliptical-LE section.
ELLIPSE_CONFIGS = {
    "circle": dict(le=EdgeSpec("circle"), g2=False),
    "ellipse": dict(le=EdgeSpec("ellipse"), g2=False),
    "ellipse+G2": dict(le=EdgeSpec("ellipse"), g2=True),
    "ellipse+ratio": dict(le=EdgeSpec("ellipse", ratio=2.4), g2=False),
    "ellipse+ratio+G2": dict(le=EdgeSpec("ellipse", ratio=2.4), g2=True),
    "ellipse+ratio+tip-axis": dict(le=EdgeSpec("ellipse", ratio=2.4, axis="tip"), g2=False),
    "auto": dict(le=EdgeSpec("auto"), g2=False),
}


def ellipse_trial(sec, config, tol=0.00015, grid=9):
    """Run one configuration on an elliptical-LE section; return error metrics."""
    cfg = ELLIPSE_CONFIGS[config]
    result = reconstruct_section(sec.pts, tol, grid=grid, le=cfg["le"], g2=cfg["g2"],
                                 label_by="min-x")
    edges = {e.label: e for e in result.edges}
    le, true_le = edges["LE"], sec.shapes["LE"]
    row = {"le_kind": le.shape.kind,
           "le_center_err": float(np.linalg.norm(le.center - true_le.center)),
           "le_tip_radius_err": abs(le.radius - true_le.tip_radius)}
    if le.shape.kind == "ellipse":
        row.update({"le_a_err": abs(le.shape.a - true_le.a),
                    "le_b_err": abs(le.shape.b - true_le.b),
                    "le_phi_err_deg": abs(math.degrees(wrap_angle(le.shape.phi - true_le.phi)))})
    terr, jumps = [], []
    for s in result.sides:
        for label, p, jump in ((s.start_edge, s.p_start, s.curvature_jump_start),
                               (s.end_edge, s.p_end, s.curvature_jump_end)):
            if label == "LE":
                terr.append(min(np.linalg.norm(p - t) for t in sec.tangency["LE"]))
                jumps.append(jump)
    row["le_tangency_err"] = terr
    row["le_curvature_jump"] = jumps
    sides = {f"{s.start_edge}->{s.end_edge}": s for s in result.sides}
    row["inflections_correct"] = all(
        s.fit.inflections() + s.junction_inflections == sec.true_inflections[k]
        for k, s in sides.items())
    row["points_over"] = sum(s.points_over for s in result.sides)
    return row


def ellipse_trials(n_trials, config, tol=0.00015, grid=9, seed=0):
    rng = np.random.default_rng(seed)
    rows = []
    for _ in range(n_trials):
        sec = synthetic_ellipse_section(edge_spacing=float(rng.uniform(0.005, 0.01)),
                                        side_points=int(rng.integers(18, 31)),
                                        seed=int(rng.integers(1 << 30)))
        rows.append(ellipse_trial(sec, config, tol, grid))
    return rows


def summarize_ellipse_trials(rows):
    def stats(key):
        vals = np.array([r[key] for r in rows if key in r])
        if len(vals) == 0:
            return None
        return {"median": float(np.median(vals)), "max": float(vals.max())}

    terr = np.concatenate([r["le_tangency_err"] for r in rows])
    jumps = np.concatenate([r["le_curvature_jump"] for r in rows])
    return {
        "trials": len(rows),
        "le_fitted_as_ellipse": int(sum(r["le_kind"] == "ellipse" for r in rows)),
        "le_center_err": stats("le_center_err"),
        "le_tip_radius_err": stats("le_tip_radius_err"),
        "le_a_err": stats("le_a_err"),
        "le_b_err": stats("le_b_err"),
        "le_phi_err_deg": stats("le_phi_err_deg"),
        "le_tangency_err": {"median": float(np.median(terr)), "max": float(terr.max())},
        "le_curvature_jump": {"median": float(np.median(jumps)), "max": float(jumps.max())},
        "inflections_correct_trials": int(sum(r["inflections_correct"] for r in rows)),
        "trials_with_points_over": int(sum(r["points_over"] > 0 for r in rows)),
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _spec_from_args(kind, radius, ratio, minor, axis):
    if axis is None or axis == "tip":
        axis_val = axis
    else:
        axis_val = math.radians(float(axis))
    if kind == "circle" and ratio is None and minor is None:
        return EdgeSpec("circle", radius=radius)
    return EdgeSpec(kind, radius=radius, ratio=ratio, minor=minor, axis=axis_val)


def main(argv=None):
    ap = argparse.ArgumentParser(description="LE/TE detection and tangency selection.")
    ap.add_argument("csv", nargs="?", help="contour points ordered around the section: x,y")
    ap.add_argument("--tol", type=float, default=0.00015)
    for e in ("le", "te"):
        E = e.upper()
        ap.add_argument(f"--{e}-shape", choices=("circle", "ellipse", "auto"), default="circle",
                        help=f"{E} shape (auto: choose circle or ellipse by BIC)")
        ap.add_argument(f"--{e}-radius", type=float,
                        help=f"{E}_R from the drawing (fixes a circular {E}'s radius)")
        ap.add_argument(f"--{e}-ratio", type=float, help=f"elliptical {E}: axis ratio a/b")
        ap.add_argument(f"--{e}-minor", type=float,
                        help=f"elliptical {E}: semi-minor axis (half the thickness at the joints)")
        ap.add_argument(f"--{e}-axis",
                        help=f"elliptical {E}: major-axis direction in degrees, or 'tip'")
    ap.add_argument("--le-at", choices=LABEL_RULES, default="radius",
                    help="how to tell the LE from the TE (default: larger radius)")
    ap.add_argument("--g2", action="store_true",
                    help="match curvature at joints with elliptical edges")
    ap.add_argument("--grid", type=int, default=9, help="tangency search grid per parameter")
    ap.add_argument("--out", help="write the result as JSON")
    ap.add_argument("--demo", action="store_true", help="run on a synthetic section with known answer")
    ap.add_argument("--trials", type=int,
                    help="repeat the synthetic comparison over random spacing and rounding")
    ap.add_argument("--ellipse-demo", action="store_true",
                    help="run each configuration on one synthetic elliptical-LE section")
    ap.add_argument("--ellipse-trials", type=int,
                    help="repeat the elliptical-LE comparison over random spacing and rounding")
    ap.add_argument("--config", choices=tuple(ELLIPSE_CONFIGS),
                    help="with --ellipse-trials: run only this configuration")
    args = ap.parse_args(argv)

    if args.trials:
        for use_radii in (False, True):
            summary = summarize_trials(trials(args.trials, args.tol, use_radii, args.grid))
            print(f"\nRadius {'fixed from drawing' if use_radii else 'fitted'}:")
            print(json.dumps(summary, indent=2))
        return 0

    if args.ellipse_trials:
        for config in ([args.config] if args.config else ELLIPSE_CONFIGS):
            summary = summarize_ellipse_trials(
                ellipse_trials(args.ellipse_trials, config, args.tol, args.grid))
            print(f"\n{config}:")
            print(json.dumps(summary, indent=2))
        return 0

    if args.ellipse_demo:
        sec = synthetic_ellipse_section()
        t = sec.shapes["LE"]
        print(f"True LE ellipse: a {t.a:.4f}  b {t.b:.4f}  phi {math.degrees(t.phi):.1f} deg  "
              f"tip radius {t.tip_radius:.5f}; true inflections {sec.true_inflections}")
        for config in ELLIPSE_CONFIGS:
            r = ellipse_trial(sec, config, args.tol, args.grid)
            shape = (f"a err {r['le_a_err']:.5f}  b err {r['le_b_err']:.5f}  "
                     f"phi err {r['le_phi_err_deg']:.2f} deg" if r["le_kind"] == "ellipse"
                     else "fitted as a circle")
            print(f"  {config:20s} {shape}  tip-radius err {r['le_tip_radius_err']:.5f}  "
                  f"LE tangency err {', '.join(f'{x:.4f}' for x in r['le_tangency_err'])}  "
                  f"LE curvature jump {', '.join(f'{x:.2f}' for x in r['le_curvature_jump'])}  "
                  f"inflections {'ok' if r['inflections_correct'] else 'WRONG'}  "
                  f"points over {r['points_over']}")
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
        ap.error("give a CSV file, --demo, --trials, --ellipse-demo or --ellipse-trials")
    pts = ab.read_points(args.csv)
    le = _spec_from_args(args.le_shape, args.le_radius, args.le_ratio, args.le_minor, args.le_axis)
    te = _spec_from_args(args.te_shape, args.te_radius, args.te_ratio, args.te_minor, args.te_axis)
    result = reconstruct_section(pts, args.tol, grid=args.grid, le=le, te=te, g2=args.g2,
                                 label_by=args.le_at)
    for e in result.edges:
        s = e.shape
        desc = (f"circle R {s.radius:.5f}{' (fixed)' if e.radius_fixed else ''}"
                if s.kind == "circle" else
                f"ellipse a {s.a:.5f} b {s.b:.5f} (a/b {s.a / s.b:.2f}) "
                f"axis {math.degrees(s.phi):.2f} deg, tip radius {s.tip_radius:.5f}")
        print(f"{e.label}: center ({e.center[0]:.5f}, {e.center[1]:.5f})  {desc}  "
              f"window {len(e.window)} pts  max residual {e.max_residual:.6f}")
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
