# MAVLink Ground Station — Roadmap

Human-readable snapshot of what's shipped and what's next. Checked boxes are done and on `main`.

## v0.1 — It talks (Week 1)
- [ ] Connects to PX4 SITL over UDP
- [ ] Decodes `HEARTBEAT` / `GLOBAL_POSITION_INT` / `SYS_STATUS` / `ATTITUDE`
- [ ] CLI-first, `println!` output
- [ ] Graceful connect / disconnect

## v0.5 — It shows (Week 2)
- [ ] `ratatui` TUI live-updating at ≥10 Hz
- [ ] Configurable connection string (`--connect udp://127.0.0.1:14550`)
- [ ] Connection status indicator

## v0.8 — It commands (Week 4)
- [ ] Arm / disarm / takeoff / land / RTL via `COMMAND_LONG`
- [ ] Goto-waypoint via `SET_POSITION_TARGET_GLOBAL_INT`
- [ ] Keyboard-driven command palette
- [ ] `COMMAND_ACK` handling with surfaced failures

## v1.0 — It flies missions (Week 6)
- [ ] Mission upload from YAML (`MISSION_ITEM_INT` sequence)
- [ ] Mission download + display
- [ ] Flight log recording (`.tlog` raw stream + parsed CSV)
- [ ] One differentiator (pick: multi-vehicle / log playback / ASCII map)
- [ ] Hero screenshot + demo video in README
- [ ] Published to crates.io
