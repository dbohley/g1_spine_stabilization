# Torso-Motion GP — Export for Downstream Control

Frozen direct-H Gaussian Process predictors of human torso (AprilTag marker
id 2) motion, with calibrated uncertainty, for a robot aligning its head to
the torso (one-step servo with GP feedforward; no CBF — see HANDOFF.md).

This folder is self-contained: **numpy + scipy only** (no torch, no training
code, no dataset needed at runtime).

> **Taking over this project? Read `HANDOFF.md` first** — full context,
> environment, data inventory, gotchas, and open threads. This README covers
> usage of the export package only.

**Two model sets ship here — use `models_robot/` on the robot platform:**

| model set | trained on | grid | horizons | held-out accuracy |
|---|---|---|---|---|
| `models_robot/` **(current)** | robot-platform pelvis recordings, 40 eps (2026-09-13), 200 pts/GP | 228 ms (4.4 Hz) | 0.23 / 0.46 / 0.91 / 2.06 s | 0.43→1.68 cm, 1.0°→4.6° |
| `models/` (legacy, lab) | lab half-roll dataset, 36 eps | 248 ms (4 Hz) | 0.25 → 2.98 s | 0.61→2.96 cm, 6.2°→7.4° |

Each set carries its own grid in its `normalization.npz`; `TorsoGP` reads it
(`gp.hset`, `gp.step_dt`) so both load through the same API. **Do not use the
legacy set on the robot platform** — cross-dataset transfer was tested and it
is worse than a freeze baseline there (e.g. 9.1° vs 3.8° at 0.5 s).

## Contents

| file | what |
|---|---|
| `torso_gp_predictor.py` | the runtime module — `TorsoGP` class (streaming API) |
| `models_robot/robot_gp_s6_h{1,2,4,9}.npz` | **current** GPs: 6 output dims × 4 horizons, 200 pts each |
| `models_robot/normalization.npz` | feature mean/std + grid metadata + split |
| `models/direct_gp_s8_h{1,2,4,8,12}.npz` | legacy lab GPs (6 dims × 5 horizons) |
| `models/normalization.npz` | legacy metadata |
| `verify_robot_export.py` | reproduces the robot-platform error table (PASS) |
| `verify_export.py` | reproduces the legacy lab table (PASS) |
| `CONTROLLER_MATH.md` | how gp-h1 feeds the head controller, written out |
| `HANDOFF.md` | take-over doc: context, environment, data, gotchas, open threads |

## Quick start

```python
from torso_gp_predictor import TorsoGP

gp = TorsoGP('models_robot')  # current model set (legacy: 'models')
gp.reset()                    # call at the START of each motion segment

for t, p, R in pose_stream:  # t [s], p (3,) [m], R (3,3) — any faster raw rate
    if gp.update(t, p, R):   # True when an internal model-rate step was accepted
        out = gp.predict()   # {h: {...} for h in gp.hset}
        h2s = gp.hset[-1]          # longest horizon of the loaded set
        p2s = out[h2s]['p_world']  # predicted torso position ~2 s ahead
        sd  = out[h2s]['sigma']    # 6-dim std [m, m, m, rad, rad, rad]
        mean, var = gp.twist()     # 1-step body twist (mean feeds the servo)
```

Each `out[h]` entry: `t_ahead` (s), `dp`/`drot` (mean pose change in the body
frame at *now*, m / rotation-vector rad), `sigma` (noise-inclusive std of
`[dp, drot]`), `p_rel`/`R_rel` (predicted absolute pose in the anchor frame),
`p_world`/`R_world` (same, mapped back to your input frame).

## Rules you must follow

1. **Reset per motion segment.** The predictor anchors on the first pose after
   `reset()`, and features include position/orientation relative to that
   anchor. Training clips were "half roll" segments anchored at segment start
   — anchor the same way. Everything is invariant to rigid transforms of the
   camera/world frame, so *where* the camera is doesn't matter; *when* you
   anchor does.
2. **It runs at the model set's own rate** (`gp.step_dt`: 228 ms for
   `models_robot`, 248 ms legacy). Feed `update()` at any rate; samples are
   accepted only when ≥ `accept_dt` (default 0.88 × step_dt) has elapsed.
   `predict()` is refreshed on each accepted sample and needs two accepted
   samples before the first output (it uses a backward-difference velocity).
3. **Predictions are re-anchored on the current measured state** every step —
   that is the regime the accuracy table below was measured in. Do not roll
   the model open-loop from a stale state and expect table accuracy.
4. **Never fall back to constant-twist extrapolation** at long horizons: the
   motion is ~0.19 Hz oscillation, and twist extrapolation is up to 3.5×
   *worse than freezing* on orientation by 3 s. If the GP is unavailable,
   freeze the last pose instead.

## Accuracy — `models_robot/` (held-out: 10 test episodes, every anchor)

| horizon | freeze baseline | **this model (200 pts/GP)** |
|---|---|---|
| 0.23 s | 0.79 cm / 2.0° | **0.43 cm / 1.0°** |
| 0.46 s | 1.39 cm / 3.8° | **0.67 cm / 1.6°** |
| 0.91 s | 2.71 cm / 7.5° | **0.99 cm / 2.6°** |
| 2.06 s | 5.64 cm / 15.9° | **1.68 cm / 4.6°** |

Per-dim 95% coverage 0.92 pooled; per-call latency (this workstation, numpy):
`twist()` / `predict(h=1)` **1.4 ms**, all 4 horizons 6.5 ms, `update()`
0.1 ms — vs the 228 ms control step. (An 800-pt fit was 25 ms per twist for
statistically indistinguishable accuracy; the size sweep is
`../sweep_gp_size.py` → `../plots/robot_gp_size_sweep.png`.)

Closed-loop (rate-limited head servo, one-step planner, medium agility):
gp-h1 targeting halves the tracking error vs the reactive baseline —
2.04° → 1.04° and 0.79 → 0.44 cm — and the residual equals the model's
one-step open-loop error, i.e. the controller extracts everything the
predictor offers.

Legacy `models/` accuracy (lab dataset): 0.61→2.96 cm and 6.2°→7.4° over
0.25–2.98 s; see `verify_export.py`. Its ~6° orientation floor was lab
marker noise; the robot platform tracks ~1°.

Uncertainty caveats:
- `sigma` includes observation noise; per-dim 95% coverage is 0.89–0.93
  (mildly overconfident vs nominal 0.95 — inflate by ~1.15× if you need a
  strict bound).
- `sigma` is nearly **state-independent**: treat it as a per-horizon tube,
  not a per-state confidence. It will not warn you about an unusually
  unpredictable moment.

## Controller integration (what is actually used)

The deployed scheme — the only one validated in closed loop — is the plain
one-step servo of `CONTROLLER_MATH.md` §1–4: feedback on the current
head-torso error plus the GP's shortest-horizon prediction as the target
(equivalently, `twist()` mean as a feedforward velocity). **No CBF is
implemented or used anywhere today**; only the twist *mean* and the h=hset[0]
prediction feed the controller.

## Wiring into a CBF (FUTURE design sketch — not implemented)

- **Continuous-time CBF:** use `gp.twist()` — mean and (diagonal) variance of
  the body twist `[v, omega]` over the next step (`gp.step_dt`). The human is exogenous:
  the twist enters `hdot = dh/dx_r (f + g u) + dh/dx_h xdot_h` as a constant
  plus Gaussian noise, so the chance constraint stays a QP (variance is
  independent of `u`).
- **MPC-CBF horizon tube:** use `predict()[h]['sigma']` per horizon — direct-H
  gives calibrated per-horizon variance with no covariance propagation and no
  divergence risk (unlike the AR/EKF path).

## Verifying the export

```bash
cd export && python verify_robot_export.py   # current set (needs dataset in parent dir)
cd export && python verify_export.py         # legacy lab set
```

Each feeds its 10 held-out episodes through the streaming API and must
reproduce its accuracy table (prints PASS/FAIL). Note: unpickling the
datasets needs numpy ≥ 2; the runtime module itself has no such constraint.

## Provenance / retraining

- `models_robot/`: the n=200 set from `../sweep_gp_size.py` (seed-0 split,
  40 train / 10 test of 50 episodes, `robot platform data/*/*.pkl`, stride 6
  → 4.4 Hz; 200 points/GP chosen for runtime — 18× faster than 800 at equal
  accuracy). To refit: rerun `sweep_gp_size.py` in the parent dir (or
  `retrain_robot_gp.py` for a full-size fit), copy the chosen set's
  `robot_gp_s6_h*.npz` + `normalization.npz` here, update EXPECT in
  `verify_robot_export.py`, rerun it.
- `models/` (legacy): fit by `../regen_direct_gp.py` (36 train / 8 val / 10
  test of 54 episodes, `Half Roll_WorldFrameMoved copy`, stride 8 → 4 Hz).

Also update the quick-start comment in `predict()` if horizons change. Full
study context: `../GP_MOTION_HANDOFF.md`.
