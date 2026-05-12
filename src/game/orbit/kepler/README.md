# Kepler Orbit Module

`game/orbit/kepler` is the game-side adapter for spacecraft patched-conics prediction.
It does not own a worker thread, prediction cache, derived frame cache, streamed chunks,
or `OrbitRenderCurve`.

The module wraps `orbitsim` Kepler primitives into gameplay-friendly pieces:

- `kepler_primary_resolver.*` selects the current primary and resolves `mu` plus body state.
- `kepler_arc_builder.*` builds a base `orbitsim::KeplerArc` and splits it around RTN impulse nodes.
- `kepler_celestial_nbody.*` builds a small n-body celestial ephemeris and samples body orbit lines.
- `kepler_arc_line_builder.*` samples arcs into world-space line vertices for draw and pick.
- `kepler_arc_info.*` derives lightweight orbital metrics and apsis positions.

Keep this module independent from `prediction_nbody`, `maneuver_nbody`, and `OrbitRenderCurve`.
The only intended bridge to moving celestial bodies is a thin body-state provider callback.
