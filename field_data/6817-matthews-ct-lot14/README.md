# Property Boundary — 6817 Matthews Ct, Bettendorf, IA 52722

Parcel **840317414** · Lot **14**, Forest Grove Crossing 4th Addition (Scott County, IA)
Area: **0.296 ac** (~12,894 SF) · Boundary: 11 corners, perimeter ~500 ft.

Source: Scott County / Beacon GIS assessor parcel polygon.

---

## READ THIS FIRST (for the rover)

**1. Coordinate system.** All `.csv` and `.txt` files use:
> **NAD83 Iowa South State Plane, US survey feet** — ESRI WKID **102676** (EPSG equivalent for the ftUS Iowa South zone). Easting ≈ 2,463,700 ft, Northing ≈ 593,800 ft.

Set the rover/controller job to this exact CRS before importing. Do **not** reproject through WGS84.

**2. Datum — matters at RTK precision.** The Iowa DOT RTN broadcasts corrections in **NAD83(2011)**. The State Plane E/N here are NAD83 and drop straight in with no datum shift. The **KML is WGS84** and is for *visualization only* — WGS84 vs NAD83 differ by ~1–1.5 m in Iowa, so never navigate off the KML lat/longs.

**3. US survey ft vs International ft.** These are **US survey feet**. If the job is set to Intl ft, eastings near 2.46 M pick up a ~12 ft error. Confirm *US survey ft*.

**4. Accuracy — this is GIS data, NOT a survey.** Corners are county-GIS grade, roughly **±1–3 ft** of the true platted corners. RTK will hit each listed point to the cm, but the point itself may be a couple feet off ground truth. To ground-truth: measure a found rebar/pin with the rover, compare to the nearest corner, and shift the whole set by that delta.

**5. Elevation.** Data is 2D. `Z = 0.00` everywhere — ignore / stake horizontal-only.

---

## Files

| File | What it is |
|------|-----------|
| `parcel-stakeout-PNEZD.csv` | 11 corner points for stakeout. |
| `parcel-boundary-loop-PNEZD.csv` | Perimeter as a looped waypoint path (11 corners + return to #1). For straight waypoint-to-waypoint driving. |
| `parcel-boundary-densified-2ft-PNEZD.csv` | Same loop, one point every ~2 ft (258 pts). For smooth line-following / marking. Corners flagged `CORNER`. |
| `parcel-corners.txt` | Human-readable: corners + side lengths + grid bearings + perimeter. |
| `parcel-boundary.kml` | Boundary polygon, WGS84, for map/phone viewing only. |
| `parcel-map.html` | Standalone satellite map viewer (open in a browser; needs internet for tiles). |

**CSV column order (headerless PNEZD):** `Point, Northing, Easting, Elevation, Description`

## Boundary notes
- Corner order = drive order; consecutive corners are edges; loop closes 11 → 1.
- Corners **3–9** trace the cul-de-sac frontage arc on Matthews Ct (seven ~8.7 ft chords).
- Longest runs: side 2→3 ~170 ft, side 9→10 ~112 ft.
