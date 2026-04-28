# Velocity bridge contract

The bridge is the only thing that talks to the Go2W Sport API in this stack. Treat it as the safety boundary.

## Encoding

| Action     | `api_id` | `parameter` (JSON)                          |
|------------|----------|---------------------------------------------|
| Move       | `1008`   | `{"x": vx, "y": vy, "z": wz}` (4 decimals)  |
| StopMove   | `1003`   | `""` (empty string)                         |

`vx`, `vy`, `wz` are the linear-x, linear-y, and angular-z components of the upstream `/cmd_vel`, after clamping to `(vx_max, vy_max, wz_max)`. The defaults are `0.30 / 0.20 / 0.50` (m/s, m/s, rad/s).

## Behavior

- **Subscriber:** `/cmd_vel` (`geometry_msgs/Twist`).
- **Publisher:** `/api/sport/request` (`unitree_api/Request`).
- **Rate:** 50 Hz (configurable via `publish_rate`).
- **Active path:** every timer tick, take the latest `/cmd_vel`, clamp it, encode as Move, publish.
- **Watchdog:** if no `/cmd_vel` has arrived for `watchdog_timeout` seconds (default 0.5 s, measured against the node's clock so `use_sim_time` works during bag replay):
    1. On the first idle tick after activity, emit a single StopMove (`api_id=1003`).
    2. Continue streaming zero-Moves (`{"x":0,"y":0,"z":0}`) at the publish rate so the locomotion controller keeps the legs balanced.
- **Shutdown:** the node tries to emit one final StopMove on Ctrl-C / SIGINT.

## Dry-run mode

`dry_run:=true` logs the Move/Stop intent at DEBUG/INFO level **without** publishing to `/api/sport/request`. This is the recommended starting state when:

- bringing the stack up on a new robot,
- replaying a bag through the full pipeline,
- after firmware updates,
- when validating MPPI tuning changes.

Switch to `dry_run:=false` only after RViz shows you a sensible `/map`, a smoothly-moving `/frontier_goal`, and a stable `/cmd_vel` stream. Keep the e-stop in hand the first time.

## Parameters

| Param                 | Default | Notes |
|-----------------------|---------|-------|
| `vx_max`              | 0.30    | m/s. Clamp magnitude on the linear-x axis. |
| `vy_max`              | 0.20    | m/s. |
| `wz_max`              | 0.50    | rad/s. |
| `watchdog_timeout`    | 0.5     | seconds. |
| `publish_rate`        | 50.0    | Hz. |
| `dry_run`             | `false` | top-level bringup defaults this to `true`. |
| `emit_stop_on_idle`   | `true`  | one-shot StopMove on transition to idle. |
| `api_id_move`         | 1008    | Sport API Move id. |
| `api_id_stop`         | 1003    | Sport API StopMove id. |

## Why these caps

`vx 0.30 / vy 0.20 / wz 0.50` is conservative for the Go2W and matches the limits configured in `nav2_params.yaml` for both the MPPI controller and the velocity_smoother. **If you raise the bridge caps you must raise the Nav2 caps too**, otherwise MPPI will plan trajectories that the bridge silently truncates and the local plan / actual motion will diverge.
