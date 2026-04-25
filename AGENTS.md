# AGENTS.md

Guidance for AI coding agents working in this repository.

## Project

`github.com/errornesttorn/mini-traffic-simulation-core` — the headless simulation
core for a Warsaw commute game. It is a Go library (single package `sim`,
Go 1.22) with a hot inner loop implemented in C and called via cgo. There is no
`main`; the package is consumed by a separate renderer/UI.

## Layout

- `sim.go` — bulk of the simulation: world state, splines, routes, traffic
  lights, pedestrians, lane changes, spawning, update loop. Deliberately a
  single large file; do not split it without being asked.
- `braking.c` / `braking.h` / `braking_cgo.go` — C implementation of the
  per-tick braking / collision-prediction / blame computation, plus the cgo
  bridge. Keep the C and Go sides in lockstep when changing struct layouts or
  constants.
- `player_proxy.go` — fits a "player proxy" car to externally-controlled
  position/heading input by projecting onto candidate splines.
- `vehicle_models.go` + `assets/cars.json`, `assets/buses.json` — vehicle model
  catalog loaded at init.
- `*_test.go` — unit tests. Run with `go test ./...`.
- `go.mod` — module definition; no external dependencies.

## Conventions

- Package is `sim`; exported API surface is what the renderer consumes
  (`World`, `NewWorld`, `Car`, `Spline`, `RoadGraph`, `FitPlayerProxyCar`, …).
  Treat exported names as part of a contract — renaming requires updating the
  consumer.
- `float32` is the default numeric type for simulation state (positions,
  speeds, times). Don't silently promote to `float64`.
- Constants for tunables live in `const (...)` blocks at the top of `sim.go`
  and as `#define`s in `braking.h`. When tuning braking/following/lane-change
  behavior, check whether the value is mirrored on the C side.
- The simulation is deterministic given a seed; avoid introducing
  wall-clock-dependent behavior in core update paths (rendering blink cadence
  is the documented exception, see `TurnSignalState` comment).

## Build & test

```
go build ./...
go test ./...
```

cgo requires a working C toolchain. If `braking.c` or `braking.h` change,
`go build` will rebuild the C side automatically.

## When making changes

- Prefer editing existing structures over adding parallel ones.
- Keep `braking_cgo.go` marshalling consistent with the C struct definitions
  in `braking.h` — mismatches manifest as silent corruption, not compile
  errors.
- Run `go test ./...` before declaring work done. Tests cover braking,
  pedestrians, turn signals, vehicle models, and the player proxy.
- Don't add a `main` package or CLI; this repo is a library.
- Don't add new top-level docs (README, design docs) unless asked.
