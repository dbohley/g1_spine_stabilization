# HANDOFF — Torso-Motion GP Project (read this first)

For the next Claude/engineer taking over. Updated 2026-09-14. The parent
directory (`..` = `/home/xliang/Downloads/runs`) holds the full project; this
`export/` folder is the self-contained deployment package.

## 1. What this project is

Predict human torso motion (AprilTag marker id 2) so a robot can align its
head to the torso without lagging.

**Control approach in use NOW (the only one tested): a plain one-step servo —
feedback on the current alignment error + the GP's one-step prediction as
the target (gp-h1 / twist feedforward). There is NO CBF anywhere in the
current pipeline** — no barrier function, no QP, no safety constraint has
been implemented or run. CBF is a future design sketch only (see §7); ignore
it unless that becomes the task. The winning model, settled after comparing
AR-chained GP / direct-horizon GP / GRU / twist extrapolation, is the
**direct-H GP**: one exact GP (hand-written numpy ARD-RBF, ~800 training
pairs) per (horizon, output dim), mapping the current 15-dim state
`[p_rel, R_rel[:,0], R_rel[:,1], v, omega]` directly to the pose change h
steps ahead in the body frame at the anchor time. First-order Markov — no
phase, no history (phase was tried, hurt, rejected).

## 2. Current state (all verified, nothing pending)

- **Deployed model: `models_robot/`** — retrained 2026-09-14 on the
  robot-platform dataset, **200 training points per GP** (size sweep
  `../sweep_gp_size.py`: 18× faster than 800 pts at equal accuracy).
  Held-out (10 eps, every anchor): 0.43 / 0.67 / 0.99 / 1.68 cm and
  1.03 / 1.62 / 2.62 / 4.55 deg at 0.23 / 0.46 / 0.91 / 2.06 s; coverage
  0.92. Runtime per control step: `twist()`/`predict(h=1)` ~1.4 ms, all
  horizons ~6.5 ms (workstation numpy) vs the 228 ms step budget.
  `python verify_robot_export.py` → PASS.
- **Legacy model: `models/`** — lab half-roll dataset. 0.61→2.96 cm,
  6.2→7.4 deg at 0.25→2.98 s. `python verify_export.py` → PASS. Kept for
  reproducibility only; on the robot platform it is WORSE than freeze
  (its lab-learned priors over-predict rotation on gentle motion).
- **Controller integration is decided and demonstrated** (closed-loop sims,
  `../control_study*.py`): use the GP's shortest-horizon prediction as the
  servo target (or equivalently `twist()` mean as feedforward added to the
  existing error-correction command). On robot data this halves tracking
  error vs reactive (2.04→1.04 deg, 0.79→0.44 cm, medium agility). The
  closed-loop residual equals the model's open-loop 1-step error, so the
  predictor, not the servo, is the current bottleneck. Math:
  `CONTROLLER_MATH.md` §1–4. **Deploy exactly this and nothing more** — the
  GP variance and longer horizons (h >= 2) are currently unused by the
  controller; §5 of that doc (CBF) is future work, not implemented.

## 3. Environment

- Python: `/home/xliang/miniconda3/envs/genesis/bin/python` (numpy 2.4,
  scipy, matplotlib, torch). **Datasets unpickle only with numpy >= 2**
  (`numpy._core`); the sam3/base envs fail. The export runtime itself needs
  only numpy+scipy, any recent version.
- The OLD lab dataset needs module stubs to unpickle
  (`g1_spine_stabilization...`); `../gp_phase_so3.py` installs them on
  import. Gotcha already fixed there: the stub's `__getattr__` must raise
  AttributeError for dunders or `import torch` breaks via inspect.
- New-style datasets (schema_version 1) are plain dicts — no stubs needed:
  `{'samples': [{'t', 'pose_in_pelvis' (4x4), 'pose_in_optical',
  'reprojection_error'}, ...], 'marker_id': 2, ...}`. `pose_in_pelvis` =
  constant calibration transform ∘ `pose_in_optical` (verified).

## 4. Data inventory (all under `..`)

| dataset | eps | rate | style | used for |
|---|---|---|---|---|
| `robot platform data/*/*.pkl` (unzipped from 4 zips) | 50 | 26.3 Hz | schema v1, gentle rolls (max ~26 deg), clean (~1 deg ori noise) | **current train/test** (40/10, seed 0) |
| `Half Roll_WorldFrameMoved copy/*.pkl` | 54 | ~32 Hz jittery | recorder objects, big rolls (~49 deg), noisy (~6 deg) | legacy model (36/8/10 split) |
| `pelvis_tracks_real/*.pkl` | 3 | 26.4 Hz | schema v1, first platform samples | early transfer test only |

Splits are reproduced by consuming the same seed-0 rng stream — see
`../regen_direct_gp.py` (it deliberately burns the AR-GP subsample draw to
match `../direct_h.py` exactly). Test-file lists are stored in each model
set's `normalization.npz` (`z['test_files']`) — use those, don't re-derive.

## 5. Key files

Here (`export/`): `torso_gp_predictor.py` (runtime; README has the API),
`models_robot/`, `models/`, `verify_robot_export.py`, `verify_export.py`,
`CONTROLLER_MATH.md`, `README.md` (usage + accuracy tables).

Parent dir, by topic (each script's docstring has details; results in
sibling `.npz`, figures in `../plots/` with matching names):
- training: `retrain_robot_gp.py` (current; REFIT=1 to refit),
  `regen_direct_gp.py` (legacy), `direct_h.py` + `gp_phase_so3.py` (original
  study: GP class, AR path, EKF/MC uncertainty — the GP class is imported
  by the retrain scripts).
- evaluation/visuals: `fan_gifs.py`, `pelvis_fan_gifs.py`, `test_pelvis_gp.py`,
  `check_pelvis_tracks.py`.
- control: `control_study.py` (lab data), `control_study_fast.py` (fast 50%),
  `control_study_robot.py` (current platform).
- history/context: `../GP_MOTION_HANDOFF.md` — the full study log (why
  direct-H won, downsampling decision, evaluation-protocol trap, EKF
  divergence story, §9 tried-and-rejected). Read §5b before changing any
  evaluation code.

## 6. Gotchas that will bite you

1. **Anchoring**: features are relative to the pose at `reset()`. Reset at
   motion-segment start (training clips are anchored there). Never feed a
   long session without resetting.
2. **Model-file naming**: `TorsoGP` finds horizons by glob `*_h{h}.npz` in
   the model dir — exactly one match per h required; the grid comes from
   `normalization.npz` (`hset`, `step_dt`). Add a horizon = add both.
3. **Evaluation protocol**: always score re-anchored on the current measured
   state, every valid start index. Open-loop rollouts from t=0 once produced
   a false "no better than freeze" conclusion.
4. **Never use constant-twist extrapolation** as fallback (worse than
   freeze); freeze instead. Never aim the tracking servo at horizons beyond
   the effective latency — closed-loop tests show it is worse than no
   prediction. The longer horizons (h >= 2) are unused by the current
   controller; they exist only for a possible future safety tube.
5. **Retrain per platform.** Cross-platform transfer was tested twice and
   failed (old→robot worse than freeze). The retrain recipe is one script
   run; follow README "Provenance / retraining".
6. **sigma** is noise-inclusive, coverage 0.89–0.93 (inflate 1.15x for a
   strict 95%), and nearly state-independent — a per-horizon tube, not a
   per-state warning.
7. Horizon durations are nominal (`h * step_dt`); velocities inside features
   use true recorded dt. A stream at a different rate than training gives a
   small horizon-time mismatch (~6% is fine in practice).

## 7. Open threads (in priority order)

1. **Build the CBF-QP** — designed only (`CONTROLLER_MATH.md` §5,
   `../GP_MOTION_HANDOFF.md` §8). Needs: a concrete barrier h(x), the robot
   model f + g u + noise, then a small QP (variance is u-independent, stays
   QP). Interfaces (`twist()`, per-horizon sigma) are ready and calibrated.
2. **Head/object marker**: the task error is head-vs-torso relative pose;
   only the torso marker is modeled so far. On the old dataset the head
   marker exists but is NOT world-frame corrected (align separately).
3. **State-dependent sigma** (heteroscedastic or input-dependent noise) if
   the CBF needs per-state risk rather than a per-horizon tube.
4. On-robot validation of the gp-h1 servo (all closed-loop results are
   kinematic simulation with rate limits + 1-step latency; real actuation
   dynamics may want a slightly longer lookahead — match h to latency).

## 8. Quick smoke test after any change

```bash
cd export
python verify_robot_export.py   # must PASS (current model)
python verify_export.py         # must PASS (legacy path unbroken)
```
(with `/home/xliang/miniconda3/envs/genesis/bin/python`; datasets in `..`)
