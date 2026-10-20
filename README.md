# 3D Pendulum (Rust)

A headless Rust port of the original Three.js and Rapier pendulum simulation. The simulation uses Rapier 3D for the bob, a fixed 120 Hz physics step, a taut-cord projection, gravity, quadratic air drag, variable wind, flutter, and entropy in the reading result.

Run with Cargo:

```sh
cargo run --release -- --question "Should I take the opportunity?"
```

Options:

```text
--length METERS       Cord length (default 1.8)
--mass GRAMS          Bob mass (default 85)
--drag VALUE          Quadratic drag coefficient (default 0.18)
--wind MPS            Wind speed (default 0)
--duration SECONDS    Reading duration (default 6.2)
--question TEXT       Intention shown with the reading
--random-org          Request a seed from random.org, with OS randomness fallback
```

The separate `die` executable simulates a rolling die. Its default geometry is a
prism; select `--shape bipyramid` for an isohedral bipyramid with any even
number of faces (for example, `cargo run --release --bin die -- --shape
bipyramid --sides 50`). Bipyramid face counts must be even and at least six.
Very high face counts are geometrically valid but are not practically fair
physical dice: their face wells become shallow and Rapier's contact solver may
not reliably distinguish neighboring faces. A simulated result is therefore
not guaranteed to be statistically fair.

The default entropy source is the operating system's cryptographic random source. Enable the optional unauthenticated random.org HTTP API with:

```sh
cargo run --release --features randomorg -- --random-org
```

If random.org is unavailable or returns an invalid response, the simulation falls back to operating-system randomness. This is a command-line, headless port; Three.js rendering and browser controls are not part of the Rust executable.

# Links
GitHub: https://github.com/Reden777/3DPendulum-rs

crates.io: https://crates.io/crates/pendulum-3d

Intention Repeater Forum: https://intentionrepeater.boards.net/
