# mavlink-gs

**A terminal-based ground station for MAVLink-speaking autonomous vehicles.**

Connect to a real or simulated drone over UDP or serial, watch live telemetry, send
commands, upload missions, and record flight logs, all from a single, keyboard-driven
TUI. Grabbing inspo from [QGroundControl](https://qgroundcontrol.com/) or Mission Planner, but stripped
down, TUI-first, and hackable.

<!-- Hero screenshot goes here once v0.5 lands:
     terminal connected to PX4 SITL showing live telemetry + mission + command bar. -->

---

## Why

Ground stations like QGroundControl are powerful but heavy. `mavlink-gs` is the opposite, as it's a fast, keyboard-driven cockpit that lives in
your terminal, starts instantly, and is small enough to still read and modify. 

## Planned features

Full milestone breakdown in **[ROADMAP.md](./ROADMAP.md)**. The short version:

- **v0.1 — It talks:** connect to PX4 SITL over UDP, decode and print core telemetry.
- **v0.5 — It shows:** live `ratatui` TUI with telemetry panels updating at ≥10 Hz.
- **v0.8 — It commands:** arm / disarm / takeoff / land / RTL / goto-waypoint from a
  command palette, with `COMMAND_ACK` handling.
- **v1.0 — It flies missions:** YAML mission upload/download, flight-log recording, and one
  differentiator feature (multi-vehicle, log playback, or an ASCII map view).

## Tech stack

| Layer | Choice |
|---|---|
| Language | Rust (2024 edition) |
| MAVLink | [`mavlink`](https://docs.rs/mavlink) crate (v2 only) |
| TUI | [`ratatui`](https://ratatui.rs/) + `crossterm` |
| Async runtime | [`tokio`](https://tokio.rs/) |
| CLI / config / logging | `clap`, `serde` + `serde_yaml`, `tracing` |

## Development setup

The whole stack is developed against **PX4 SITL** (software-in-the-loop) — no hardware
required. The fastest way to get a simulated drone talking MAVLink on UDP `14550`:

```bash
# One-time pull (~2 GB)
docker pull jonasvautherin/px4-gazebo-headless:latest

# Run — exposes MAVLink over UDP 14550
docker run --rm -it \
  -p 14550:14550/udp \
  -p 14556:14556/udp \
  jonasvautherin/px4-gazebo-headless:latest
```

You'll see PX4 boot logs ending in `INFO [commander] Ready for takeoff!`. To sanity-check
the connection, point [QGroundControl](https://qgroundcontrol.com/) at it (auto-connects to
`udp://127.0.0.1:14550`) — a vehicle should appear near PX4's default home (Zurich), and
you should be able to arm → takeoff → land.

> **Note:** PX4's boot log prints `MAVLink only on localhost (set MAV_0_BROADCAST=1 ...)`.
> This is harmless with the Docker command above — the `-p 14550:14550/udp` port forward
> lets a ground station on the host connect fine. No parameter change needed.

_Validated against PX4 `v1.18.0` (git-hash `aed118e`), Gazebo 8.14.0, x500 quadcopter._

### Building (once the crate lands)

```bash
cargo run -- --connect udp://127.0.0.1:14550
```

## License

Licensed under the [MIT License](./LICENSE).
