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
