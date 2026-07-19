# Sidewalk test — 2026-07-19

Four RTK-surveyed spots along the sidewalk, recorded with the ZED-F9R +
Iowa DOT RTN (see `waypoints.csv`). Goal: rover starts at spot 1, drives
the sidewalk to spot 4, and stops.

## Files

| File | What it is |
|------|-----------|
| `waypoints.csv` | The 4 raw surveyed spots (WGS84 lat/lon from the receiver, UTC time, fix quality, hAcc). |
| `path-spot1-to-spot4.csv` | Drivable path: the 1→2→3→4 polyline densified to ~0.5 m spacing. 40 points, 19.31 m total. |

## Path CSV columns

- `lat`, `lon` — WGS84 degrees (receiver-native; self-consistent with the
  live RTK position the rover navigates by, so no datum shift needed).
- `x_east_m`, `y_north_m` — local ENU meters, origin = spot 1.
- `dist_along_m` — cumulative distance from spot 1.
- `label` — `spot N` at the surveyed vertices, `path` for interpolated points.

## Geometry (ENU, origin = spot 1)

```
spot 1:   0.00 E,   0.00 N   (start)
spot 2:  -3.16 E,  -3.50 N   bend, ~62° right
spot 3: -10.71 E,  -1.58 N   bend, ~77° right
spot 4: -10.58 E,  +5.23 N   (stop)
```

Segment bearings: 1→2 222° (SW, 4.71 m) · 2→3 284° (WNW, 7.79 m) ·
3→4 001° (N, 6.81 m). Not a sharp single-corner "L" — two ~70° right-hand
bends, consistent with the sidewalk curving around the court.

## Accuracy notes

Spots 1–3 were RTK FLOAT (hAcc 0.12–0.20 m); spot 4 caught a FIXED window
(hAcc 0.04 m). Positions are receiver-reported, so the path inherits that
per-point uncertainty — fine for a first drive test, re-survey in FIXED if
we want better.
