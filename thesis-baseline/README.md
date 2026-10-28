# Airfoil side baseline: least squares + fairness

A deterministic baseline to compare against the genetic algorithm. For one side
of a section (PS or SS), it fits a cubic B-spline by constrained least squares
plus a fairness term instead of interpolating every coordinate. It then reports
inflections and point deviations.

- The fit keeps G1 continuity with the LE/TE arcs. The first and last control
  points sit on the tangency points, and the second and second-to-last lie on
  the arc tangents.
- The number of control points grows until every point is within tolerance.
  The fairness weight `lambda` is then raised as far as the tolerance allows.
- Deviation is the true closest-point distance to the curve, not the residual
  at the fitting parameter.
- Only numpy and scipy are needed, plus matplotlib for `--plot`. Everything
  runs locally.

## Usage

```
pip install numpy scipy matplotlib
python airfoil_baseline.py --demo --plot demo.png
python airfoil_baseline.py side.csv --tol 0.00015 \
    --start-tangent TX TY --end-tangent TX TY \
    --out fit.json --plot fit.png
```

`side.csv` holds the ordered side points as `x,y`, one per line; header lines
are skipped. By default the first and last rows are the tangency points (PT
from Algorithm 3). If the CSV holds only interior points, pass `--start-point`
and `--end-point`.

Tangents point in the direction of travel along the side. Take them from the
edge arcs; if they are omitted they are estimated from the points, with a
warning.

The exit code is 0 when the tolerance is met and 1 when it is not. When it is
not met, the fit with the smallest maximum deviation is reported.

On the synthetic demo (25 points rounded to 4 decimals from a curve with no
inflections), interpolation produces 2 spurious inflections. The baseline gives
0 inflections within 0.00015 using 5 control points.

## Notes

- **Ties go to fewer control points.** Among the control-point counts that
  meet the tolerance, the fit with the fewest inflections wins, then the one
  with the fewest control points.
- **No safety margin.** The tolerance search pushes `lambda` until the largest
  deviation sits at the tolerance. Pass a slightly smaller `--tol` if you want
  a margin.
- **Junction inflections.** The report prints the curvature at both ends.
  Compare its sign with the adjacent arc's curvature, measured in the same
  direction of travel, to catch an inflection at an arc-spline junction.
- **Scaling of `lambda`.** `lambda` is dimensionless: at `lambda = 1` the
  fairness and fit terms weigh about equally.

## Rebuilding the curve in OpenCASCADE (pythonocc)

```python
import json
from OCC.Core.Geom2d import Geom2d_BSplineCurve
from OCC.Core.TColgp import TColgp_Array1OfPnt2d
from OCC.Core.TColStd import TColStd_Array1OfReal, TColStd_Array1OfInteger
from OCC.Core.gp import gp_Pnt2d

fit = json.load(open("fit.json"))
poles = TColgp_Array1OfPnt2d(1, len(fit["poles"]))
for i, (x, y) in enumerate(fit["poles"], 1):
    poles.SetValue(i, gp_Pnt2d(x, y))
knots = TColStd_Array1OfReal(1, len(fit["knots"]))
mults = TColStd_Array1OfInteger(1, len(fit["knots"]))
for i, (k, m) in enumerate(zip(fit["knots"], fit["multiplicities"]), 1):
    knots.SetValue(i, k)
    mults.SetValue(i, m)
side = Geom2d_BSplineCurve(poles, knots, mults, fit["degree"])
```

The resulting curve can replace the `Geom2dAPI_Interpolate` result in
Algorithm 4. The existing evaluations (Algorithms 5, 6 and 9-11) then run on
it unchanged.

## Tests

```
pip install pytest
python -m pytest
```
