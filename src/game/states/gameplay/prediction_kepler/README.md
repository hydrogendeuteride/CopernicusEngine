# Kepler Prediction

`prediction_kepler` is the gameplay-side adapter for analytic spacecraft orbit display.

It deliberately does not use the n-body prediction runtime, derived-service cache,
streamed chunks, or `OrbitRenderCurve`. The flow is direct:

```txt
current celestial n-body ephemeris
  -> celestial body lines and Kepler body-state provider
player subject state
  -> game/orbit/kepler builder
  -> Kepler arcs
  -> tessellated line vertices
  -> OrbitPlotSystem lines and simple line picking
```

The module accepts maneuver nodes as already-normalized `KeplerManeuverNode` values,
but it does not depend on `maneuver_nbody`.
