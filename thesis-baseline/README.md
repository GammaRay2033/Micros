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

## Edges and tangency points (`edges.py`)

A replacement for Algorithms 2-3. It takes the contour ordered around the
section and does the following:

1. Finds the LE and TE as the two strongest convex curvature peaks of a lightly
   smoothed periodic spline.
2. Fits each edge circle by geometric least squares, growing the window while
   points stay within tolerance. Pass the drawing radius to fit only the center.
3. Chooses all four tangency points by side-fit quality, using the baseline
   above.

```
python edges.py contour.csv --tol 0.00015 --le-radius 0.050 --te-radius 0.020 --out section.json
python edges.py --demo        # one synthetic section with known answer
python edges.py --trials 20   # compare against Algorithm 2 over random spacing and rounding
```

By default the LE is the edge with the larger fitted circle (or the closer
match when both drawing radii are given). That is wrong for a sharp elliptical
LE, whose tip radius is small, so `--le-at` can pick it by coordinates instead
(`min-x`, `max-x`, `min-y`, `max-y`) or by local width (`width`). A section
takes about 8 seconds with circular edges, mostly the tangency grid search
(`--grid`).

### Elliptical edges

```
python edges.py contour.csv --le-shape ellipse --le-ratio 2.4 --g2 --le-at min-x
python edges.py contour.csv --le-shape auto --le-at min-x
python edges.py --ellipse-demo            # each configuration on one synthetic section
python edges.py --ellipse-trials 20       # the same over random spacing and rounding
```

- `--le-shape ellipse` fits the LE as an ellipse: a geometric (orthogonal
  distance) least-squares fit, started from Fitzgibbon's direct fit and from
  circle-based guesses.
- `--le-ratio` (a/b) and `--le-minor` (semi-minor axis, half the thickness at
  the joints) take values from the drawing. `--le-axis` fixes the major-axis
  direction in degrees. `--le-axis tip` uses the smoothed contour's normal at
  the curvature peak, but on the synthetic section that estimate was 2.5
  degrees off and made results worse; prefer fitting the axis unless the
  drawing gives it.
- `--g2` makes the sides match the ellipse's curvature at the joints, not just
  its tangent. The baseline side fit (`fit_side(..., k_start=, k_end=)`) does
  this exactly by searching the end handle length, with one linear solve per
  trial.
- `--le-shape auto` fits both shapes and keeps the ellipse only when it is
  clearly elongated (a/b >= 1.3) and BIC prefers it.
- The TE options are the same with `--te-`.

## Tests

```
pip install pytest
python -m pytest
```
