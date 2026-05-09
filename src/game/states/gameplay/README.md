# Gameplay Folder Guide

This folder contains `GameplayState` and the supporting modules for the main
space-combat gameplay mode: scene setup, orbital simulation, time warp,
prediction display, maneuver planning, settings, and the gameplay HUD.

## Folder Structure

```
gameplay/
  gameplay_state.h / .cpp         # GameplayState lifecycle and top-level dispatch
  gameplay_state_scene.cpp        # scene bootstrap, spawning, player orbiter selection
  gameplay_state_sim.cpp          # time-warp transitions and simulation stepping glue
  time_warp_state.h               # warp mode and factor table
  settings/                       # gameplay settings plus preload-retention cache
  orbital/                        # orbital runtime, physics coupling, orbiter bridges
  ui/                             # HUD, debug windows, frame monitor
  prediction_kepler/              # Kepler prediction model, system, draw path, adapter
  prediction_nbody/               # N-body prediction runtime, cache, draw, adapter
  maneuver_nbody/                 # maneuver-node editor, commands, gizmo, execution
  scenario/                       # scenario config data model and JSON serialization
```

## Core Files

- `gameplay_state.h`
  Declares `GameplayState`, its lifecycle methods, prediction and maneuver
  adapter entry points, and owned gameplay state.

- `gameplay_state.cpp`
  Handles enter/exit/update/fixed-update dispatch, settings/scenario loading,
  time-warp input, component context creation, and preload cleanup.

- `gameplay_state_scene.cpp`
  Builds the gameplay scene, initializes the orbital scenario, spawns celestial
  bodies and orbiters, manages active player orbiters, and syncs camera/collision
  callbacks.

- `gameplay_state_sim.cpp`
  Handles time-warp level transitions and calls `OrbitalPhysicsSystem` for
  physics warp and rails warp behavior.

- `time_warp_state.h`
  Holds realtime, physics-warp, and rails-warp state plus the warp factor table.

## Subfolders

### `settings/`

- `gameplay_settings.h / .cpp`
  `GameplaySettings` and JSON load/save helpers.

- `gameplay_preload_cache.h / .cpp`
  Retains preloaded GLTF scenes across the loading-to-gameplay transition.

### `orbital/`

- `orbit_runtime.h`
  Runtime orbital data (`CelestialBodyInfo`, `OrbitalScenario`, `OrbiterInfo`)
  and inline N-body/orbit helper functions.

- `orbital_runtime_system.h / .cpp`
  Owns the `OrbitalScenario`, orbiter registry, player orbiter lookup, and
  scenario-to-orbitsim initialization.

- `orbital_physics_system.h / .cpp`
  Couples physics and orbitsim, handles rails warp, runtime rails promotion,
  formation hold dispatch, and `GameplayOrbitalContextBuilder`.

- `formation_hold_system.h / .cpp`
  Implements LVLH station-keeping for formation-held orbiters.

- `orbiter_state_bridge.h / .cpp`
  Contains physics body bridge helpers and shared orbiter world-state sampling.

### `ui/`

- `gameplay_state_ui.cpp`
  Main gameplay HUD, settings controls, scenario save/load controls, and window
  toggles.

- `gameplay_state_kepler_prediction_debug_ui.cpp`
  Kepler prediction debug window.

- `gameplay_state_nbody_orbit_debug_ui.cpp`
  N-body orbit diagnostics and prediction runtime debug UI.

- `frame_monitor.h / .cpp`
  FPS/frame-time monitor overlay.

### Prediction And Maneuver

- `prediction_kepler/gameplay_state_kepler_prediction.cpp`
  `GameplayState` adapter for Kepler prediction mode.

- `gameplay_state_orbit_prediction.cpp`
  `GameplayState` prediction mode router for Kepler and N-body prediction.

- `prediction_kepler/`, `prediction_nbody/`, and `maneuver_nbody/`
  Keep their internal README files as the starting point for deeper changes.

## Notes

- `GameplayState` remains the owner of gameplay-level state. Subfolders hold
  cohesive support systems, not a replacement state hierarchy.
- `orbital/` intentionally merges the former small helper/context/provider files
  into fewer support units to reduce root-folder clutter.
- `settings/` contains both persisted gameplay settings and the small preload
  retention cache used by `GameplayLoadingState`.
