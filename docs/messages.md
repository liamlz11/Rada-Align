# Live state message (v1)

One JSON object per controller tick. The simulator emits it as JSON lines
(`python -m rada_align.sim --stream`); the real controller will emit the same
object. The dashboard (task 7) reads only this schema.

Angles in degrees, lengths in millimetres unless the name says otherwise.

| Field | Type | Meaning |
| --- | --- | --- |
| `t_s` | number | Time since start, seconds |
| `state` | string | `IDLE`, `ACQUIRE`, `ALIGN`, `VERIFY`, `TRACK`, `FAULT` |
| `phase` | string | Sub-step: `search`, `settle`, `measure`, `trim`, `track`, `hold`, or empty |
| `aligned` | bool | Pass criterion `abs(mean) + 2 * sigma < 0.05` currently met on all four axes |
| `aligned_with_bias` | bool | Same, with the calibration uncertainty `u_bias_deg` added on every axis |
| `hold` | bool | Hold input active (radar measuring): no motor motion |
| `fault_reason` | string or null | Set in `FAULT` |
| `warning` | string or null | e.g. unstable conditions while tracking |
| `distance_m` | number or null | Measured distance between the heads |
| `errors` | object | Keys `radar_az`, `radar_el`, `horn_az`, `horn_el` |
| `errors.<axis>.last_deg` | number or null | Latest single sample |
| `errors.<axis>.mean_deg` | number | Mean over the last completed window (absent before the first) |
| `errors.<axis>.sigma_deg` | number | Standard deviation of the samples in that window |
| `errors.<axis>.score_deg` | number | `abs(mean) + 2 * sigma`, compared with the 0.05 deg tolerance |
| `errors.<axis>.u_bias_deg` | number | Expanded calibration uncertainty, `2 * u_delta`. The system cannot observe this error |
| `errors.<axis>.score_with_bias_deg` | number | `score_deg + u_bias_deg` |
| `errors.<axis>.n` | integer | Samples in that window |
| `dx_mm`, `dh_mm` | number or null | Horn center off the radar axis: to the radar's right, and up |
| `actuators` | object | `x`, `z` (mm), `pan`, `tilt` (deg) |
| `before` | object or null | `<axis>`: mean error at the first measurement |
| `after` | object or null | `<axis>`: `mean_deg`, `sigma_deg`, `score_deg`, `u_bias_deg`, `score_with_bias_deg`, `n` of the passed verify window |
| `iterations` | integer | Coarse corrections in this alignment |
| `verify_attempts` | integer | Verify windows in this alignment |
| `corrections` | integer | Re-alignments triggered while tracking |
| `events` | array | `[t_s, text]` log entries since the previous message. Every motor command appears as `move <axis> <delta> <unit>` |
| `sim` | bool | `true` when the data comes from the simulator |
| `seq` | integer | Dashboard session only: message counter |
| `setup_id` | integer | Dashboard session only: which setup this is (changes on "new setup") |
| `tolerance_deg` | number | Dashboard session only: the tolerance the page colours against |
| `truth` | object | Simulator only: the true error per axis, for display and debugging |

Sign conventions: `radar_az` > 0 means the horn center is to the right of the
radar axis as seen from the radar; `radar_el` > 0 means above it. `horn_az` > 0
means the radar is to the right of the horn boresight as seen from the horn.

Changes from v0: `aligned_with_bias`, `hold`, `u_bias_deg`, `score_with_bias_deg`,
phase `hold`, and `move ...` events.

Control input (not part of this message): `Controller.set_hold(True | False)`.

## Dashboard backend (`python -m rada_align.ui`)

| Endpoint | Meaning |
| --- | --- |
| `GET /` | The operator page |
| `WS /ws` | The message above, one per tick, starting from the latest |
| `GET /api/state` | Latest message |
| `POST /api/start`, `/api/stop`, `/api/realign` | Controller commands |
| `POST /api/hold` with `{"hold": true \| false}` | Hold input |
| `POST /api/new_setup` | Simulator only: a new random manual setup |
| `GET /api/report` | Before/after, pass/fail and the event log of the last alignment |
