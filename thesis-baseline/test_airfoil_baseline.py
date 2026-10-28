import json

import numpy as np
import pytest

import airfoil_baseline as ab


@pytest.fixture(scope="module")
def demo():
    return ab.demo_side()


def test_fit_respects_g1_end_constraints(demo):
    Q, ps, pe, t0, t1 = demo
    fit = ab.fit_side(Q, ps, pe, t0, t1, n_ctrl=7, lam=0.5)
    P = fit.poles
    np.testing.assert_allclose(P[0], ps, atol=1e-12)
    np.testing.assert_allclose(P[-1], pe, atol=1e-12)
    np.testing.assert_allclose(ab.unit(P[1] - P[0]), t0, atol=1e-9)
    np.testing.assert_allclose(ab.unit(P[-1] - P[-2]), t1, atol=1e-9)


def test_deviation_is_true_distance(demo):
    Q, ps, pe, t0, t1 = demo
    fit = ab.fit_side(Q, ps, pe, t0, t1, n_ctrl=8)
    # Dense sampling can only overestimate the true distance.
    dense = fit.spline(np.linspace(0, 1, 200001))
    brute = np.sqrt(((Q[:, None, :] - dense[None]) ** 2).sum(axis=2)).min(axis=1)
    assert np.all(fit.deviations <= brute + 1e-12)
    assert np.all(brute - fit.deviations < 2e-6)
    # At an interior closest point, the offset is perpendicular to the curve.
    r = fit.spline(fit.params) - Q
    tangent = fit.spline.derivative(1)(fit.params)
    cos = np.abs((r * tangent).sum(axis=1)) / (
        np.linalg.norm(r, axis=1) * np.linalg.norm(tangent, axis=1))
    assert np.all(cos < 1e-6)


def test_more_fairness_never_adds_deviation_slack(demo):
    Q, ps, pe, t0, t1 = demo
    devs = [ab.fit_side(Q, ps, pe, t0, t1, 8, lam).max_dev for lam in (0.0, 1.0, 100.0)]
    assert devs[0] <= devs[1] <= devs[2]


def test_baseline_beats_interpolation_on_demo(demo):
    Q, ps, pe, t0, t1 = demo
    fit, feasible = ab.fit_to_tolerance(Q, ps, pe, t0, t1, tol=0.00015)
    interp = ab.interpolate_side(Q, ps, pe, t0, t1)
    assert feasible
    assert fit.max_dev <= 0.00015 + 1e-12
    assert fit.inflections() == 0
    assert fit.polygon_inflections() == 0
    assert interp.inflections() > 0


def test_infeasible_tolerance_reports_closest_fit(demo):
    Q, ps, pe, t0, t1 = demo
    fit, feasible = ab.fit_to_tolerance(Q, ps, pe, t0, t1, tol=1e-9, m_max=8)
    assert not feasible
    assert fit.max_dev > 1e-9


def test_cli_with_csv_and_tangents(demo, tmp_path):
    Q, ps, pe, t0, t1 = demo
    csv_path = tmp_path / "side.csv"
    csv_path.write_text("x,y\n" + "\n".join(f"{x},{y}" for x, y in np.vstack([ps, Q, pe])))
    out = tmp_path / "fit.json"
    code = ab.main([str(csv_path), "--start-tangent", *map(str, t0),
                    "--end-tangent", *map(str, t1), "--out", str(out)])
    assert code == 0
    data = json.loads(out.read_text())
    assert data["inflections"] == 0
    assert data["max_deviation"] <= 0.00015 + 1e-12
    assert sum(data["multiplicities"]) == data["n_ctrl"] + data["degree"] + 1
