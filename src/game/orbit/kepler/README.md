# Kepler Orbit Module

`game/orbit/kepler` is the game-side adapter for spacecraft patched-conics prediction.
It does not own a worker thread, prediction cache, derived frame cache, streamed chunks,
or `OrbitRenderCurve`.

The module wraps `orbitsim` Kepler primitives into gameplay-friendly pieces:

- `kepler_primary_resolver.*` selects the current primary and resolves `mu` plus body state.
- `kepler_orbit_builder.*` builds a base `orbitsim::KeplerArc` from an inertial subject state.
- `kepler_maneuver_solver.*` splits a single-primary arc around RTN impulse nodes.
- `kepler_orbit_tessellator.*` samples arcs into world-space line vertices for draw and pick.
- `kepler_orbit_metrics.*` derives lightweight orbital metrics and apsis positions.

Keep this module independent from `prediction_nbody`, `maneuver_nbody`, and `OrbitRenderCurve`.
The only intended bridge to moving celestial bodies is a thin body-state provider callback.
