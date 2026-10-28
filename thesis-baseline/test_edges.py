import math

import numpy as np
import pytest

import edges as E


@pytest.fixture(scope="module")
def section():
    return E.synthetic_section()


@pytest.fixture(scope="module")
def result(section):
    return E.reconstruct_section(section.pts)


def test_fit_circle_recovers_exact_circle():
    th = np.linspace(0.3, 1.9, 9)
    c, r = np.array([1.2, -0.4]), 0.05
    pts = c + r * np.column_stack([np.cos(th), np.sin(th)])
    center, radius, res = E.fit_circle(pts)
    np.testing.assert_allclose(center, c, atol=1e-10)
    assert radius == pytest.approx(r, abs=1e-10)
    assert res < 1e-10
    center, radius, _ = E.fit_circle(pts + 1e-3, radius=r)
    np.testing.assert_allclose(center, c + 1e-3, atol=1e-10)
    assert radius == r


def test_curvature_peaks_land_on_both_arcs(section):
    peaks, orient = E.curvature_peaks(section.pts, 0.00015)
    assert orient == 1.0
    arcs = [set(section.on_arc["LE"]), set(section.on_arc["TE"])]
    assert sorted(any(p in a for p in peaks) for a in arcs) == [True, True]


def test_edges_labelled_and_fitted(result, section):
    by_label = {e.label: e for e in result.edges}
    assert set(by_label) == {"LE", "TE"}
    for label, e in by_label.items():
        assert e.radius == pytest.approx(section.radii[label], abs=5e-4)
        assert np.linalg.norm(e.center - section.centers[label]) < 1e-3
        assert set(e.window) <= set(section.on_arc[label]) | set(
            range(min(section.on_arc[label]) - 1, max(section.on_arc[label]) + 2))


def test_sides_meet_tolerance_and_true_inflections(result, section):
    for s in result.sides:
        key = f"{s.start_edge}->{s.end_edge}"
        assert s.points_over == 0
        assert s.fit.inflections() + s.junction_inflections == section.true_inflections[key]


def test_sides_are_g1_with_their_arcs(result):
    edges = {e.label: e for e in result.edges}
    for s in result.sides:
        for label, p, th in ((s.start_edge, s.p_start, s.theta_start),
                             (s.end_edge, s.p_end, s.theta_end)):
            e = edges[label]
            assert abs(np.linalg.norm(p - e.center) - e.radius) < 1e-12
        P = s.fit.poles
        np.testing.assert_allclose(E.ab.unit(P[1] - P[0]),
                                   E.circle_tangent(s.theta_start, result.orient), atol=1e-9)
        np.testing.assert_allclose(E.ab.unit(P[-1] - P[-2]),
                                   E.circle_tangent(s.theta_end, result.orient), atol=1e-9)


def test_tangency_close_to_truth(result, section):
    for s in result.sides:
        for label, p in ((s.start_edge, s.p_start), (s.end_edge, s.p_end)):
            err = min(np.linalg.norm(p - t) for t in section.tangency[label])
            assert err < 0.03


def test_fixed_radius_is_kept(section):
    edges, _ = E.detect_edges(section.pts, 0.00015, le_radius=0.05, te_radius=0.02)
    by_label = {e.label: e for e in edges}
    assert by_label["LE"].radius == 0.05 and by_label["LE"].radius_fixed
    assert by_label["TE"].radius == 0.02 and by_label["TE"].radius_fixed


def test_side_indices_cover_contour_between_windows(result, section):
    n = len(section.pts)
    a, b = result.edges
    covered = set(a.window) | set(b.window) | set(E.side_indices(n, a, b)) | set(
        E.side_indices(n, b, a))
    assert covered == set(range(n))


# --- Ellipses -----------------------------------------------------------------

TRUE_ELL = E.Ellipse(center=np.array([0.3, -0.1]), a=0.06, b=0.025, phi=math.radians(200))


def test_ellipse_geometry_is_consistent():
    ts = np.linspace(-1.4, 1.4, 15)
    pts = np.array([TRUE_ELL.point(t) for t in ts])
    np.testing.assert_allclose(TRUE_ELL.params_of(pts), ts, atol=1e-9)
    assert TRUE_ELL.distance(pts).max() < 1e-12
    # Tip curvature b/a^2 at the co-vertex, a/b^2 at the tip.
    assert TRUE_ELL.curvature(math.pi / 2) == pytest.approx(0.025 / 0.06 ** 2)
    assert TRUE_ELL.curvature(0.0) == pytest.approx(0.06 / 0.025 ** 2)
    # Offsetting along the normal gives that distance back.
    t = 0.7
    n = np.array([TRUE_ELL.derivative(t)[1], -TRUE_ELL.derivative(t)[0]])
    p = TRUE_ELL.point(t) + 0.003 * n / np.linalg.norm(n)
    assert TRUE_ELL.distance(p)[0] == pytest.approx(0.003, rel=1e-6)


@pytest.mark.parametrize("constraints", [{}, {"ratio": 2.4}, {"minor": 0.025},
                                         {"phi": math.radians(200)}])
def test_fit_ellipse_recovers_half_ellipse(constraints):
    pts = np.array([TRUE_ELL.point(t) for t in np.linspace(-math.pi / 2, math.pi / 2, 17)])
    e, res = E.fit_ellipse(pts, **constraints)
    assert res < 1e-8
    assert e.a == pytest.approx(0.06, abs=1e-7)
    assert e.b == pytest.approx(0.025, abs=1e-7)
    assert abs(E.wrap_angle(e.phi - TRUE_ELL.phi)) < 1e-6
    np.testing.assert_allclose(e.center, TRUE_ELL.center, atol=1e-7)


def test_fitzgibbon_on_full_ellipse():
    pts = np.array([TRUE_ELL.point(t) for t in np.linspace(0, 2 * math.pi, 40, endpoint=False)])
    e = E.fitzgibbon_ellipse(pts)
    assert e.a == pytest.approx(0.06, rel=1e-6) and e.b == pytest.approx(0.025, rel=1e-6)


@pytest.fixture(scope="module")
def ell_section():
    return E.synthetic_ellipse_section()


def test_synthetic_ellipse_section_truth(ell_section):
    assert ell_section.true_inflections == {"LE->TE": 2, "TE->LE": 0}
    assert E.signed_area(ell_section.pts) > 0


def test_detect_elliptical_le(ell_section):
    edges, _ = E.detect_edges(ell_section.pts, 0.00015, le=E.EdgeSpec("ellipse"),
                              label_by="min-x")
    le = {e.label: e for e in edges}["LE"]
    assert le.shape.kind == "ellipse"
    assert le.shape.a == pytest.approx(0.06, abs=1e-3)
    assert le.shape.b == pytest.approx(0.025, abs=2e-4)
    assert abs(math.degrees(E.wrap_angle(le.shape.phi - math.radians(200)))) < 1.0


def test_auto_picks_the_right_shape(ell_section, section):
    edges, _ = E.detect_edges(ell_section.pts, 0.00015, le=E.EdgeSpec("auto"), label_by="min-x")
    assert {e.label: e for e in edges}["LE"].shape.kind == "ellipse"
    edges, _ = E.detect_edges(section.pts, 0.00015, le=E.EdgeSpec("auto"), label_by="min-x")
    assert {e.label: e for e in edges}["LE"].shape.kind == "circle"


def test_ellipse_g2_section(ell_section):
    result = E.reconstruct_section(ell_section.pts, le=E.EdgeSpec("ellipse", ratio=2.4),
                                   g2=True, label_by="min-x")
    for s in result.sides:
        key = f"{s.start_edge}->{s.end_edge}"
        assert s.points_over == 0
        assert s.fit.inflections() + s.junction_inflections == ell_section.true_inflections[key]
        if s.start_edge == "LE":
            assert s.g2_start and s.curvature_jump_start < 1e-6
        if s.end_edge == "LE":
            assert s.g2_end and s.curvature_jump_end < 1e-6
