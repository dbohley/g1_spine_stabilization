# Incorporating the GP (h=1) into the Head-Alignment Controller

How the recommended `gp-h1` strategy from the closed-loop study
(`../control_study.py`) works, written out. Plain-text math throughout.

## Setup

Control step `k`, period `D = 0.248 s` (4 Hz). Known at step k:

```
torso pose (tracked):  p_k   (3,)  R_k   (3x3)
head  pose:            ph_k  (3,)  Rh_k  (3x3)
```

The command computed at step k executes over `[k, k+1]` and is scored against
the torso at `k+1`. That one step of latency is why the reactive baseline
(aim at the torso's *current* pose) always trails a moving torso.

Notation: `exp(r)` is the rotation matrix of rotation-vector `r` (Rodrigues);
`log(R)` is its inverse (rotation vector of `R`); `^T` is transpose.

## 1. What the GP outputs

From the current feature vector

```
x_k = [ p_rel(3), R_rel[:,0](3), R_rel[:,1](3), v_k(3), w_k(3) ]      (15,)
```

the six h=1 GPs return the mean pose change over the NEXT step, expressed in
the torso body frame at time k:

```
dp_hat  ~  R_k^T (p_{k+1} - p_k)          predicted position change   [m]
dr_hat  ~  log( R_k^T R_{k+1} )           predicted rotation change   [rad]
```

(`TorsoGP.predict()[1]['dp']` and `['drot']`, each with a std `sigma`.)

## 2. Target = predicted next torso pose

Map the body-frame change back to the world/anchor frame
(`predict()[1]['p_rel'] / ['R_rel']` do exactly this):

```
p*_{k+1} = p_k + R_k dp_hat
R*_{k+1} = R_k exp(dr_hat)
```

The reactive baseline is the special case `dp_hat = dr_hat = 0`,
i.e. target `(p_k, R_k)`.

## 3. Single-step plan toward the target

Identical to the baseline planner — one step that mitigates the alignment
error — only the error is now measured to the PREDICTED pose:

```
e_p = p*_{k+1} - ph_k                       position error   (3,)
e_r = log( Rh_k^T R*_{k+1} )                orientation error (rotvec, 3,)
```

Executed step, saturated at the rate limits `v_max`, `w_max`:

```
ph_{k+1} = ph_k + e_p * min(1, v_max*D / ||e_p||)
Rh_{k+1} = Rh_k exp( e_r * min(1, w_max*D / ||e_r||) )
```

Unsaturated, this is deadbeat: the head lands exactly on the predicted pose,
and the residual tracking error equals the GP's one-step prediction error.
That is why the closed-loop result (6.1 deg / 0.65 cm at medium agility)
matches the open-loop h=1 GP accuracy (6.2 deg / 0.61 cm).

## 4. Equivalent form: feedback + GP feedforward

Divide the commanded step by `D` and split it. The velocity command is

```
u  =  u_fb  +  u_ff

u_fb = (1/D) * [ p_k - ph_k ;  log(Rh_k^T R_k) ]     your existing
                                                     error-mitigation term
u_ff = [ R_k dp_hat / D ;  dr_hat / D ]              GP feedforward
     = TorsoGP.twist() mean                          (v in world, w in torso
                                                      body frame - rotate to
                                                      your control frame)
```

then saturate at `(v_max, w_max)`. The exact rotational split is
`log( Rh_k^T R_k exp(dr_hat) )`, but at the per-step angles seen here
(< ~7 deg) the additive form is first-order exact.

So the incorporation is literally: **keep the baseline feedback untouched and
add the GP's predicted torso twist as a feedforward velocity.** With a
proportional gain `K < 1/D` instead of deadbeat, nothing changes structurally
— the feedforward still cancels the steady-state lag `(torso velocity)/K`
that pure feedback carries against a moving target.

## 5. Chance-constrained CBF version (FUTURE work — NOT implemented)

Nothing below is in use. The current controller is §1–4 only (mean
prediction, no variance, no constraints). This section records how a CBF
*would* consume the GP if one is ever built.

Same two ingredients, with the GP variance attached. The GP twist is
Gaussian:

```
xdot_h ~ N( mu_tw , S_v ),   mu_tw = twist() mean,
                             S_v   = diag(sigma^2) / D^2   (twist() var)
```

CBF condition with robot dynamics `xdot_r = f + g u + w`, `w ~ N(0, S_w)`:

```
hdot = dh/dx_r (f + g u) + dh/dx_h xdot_h

mean(hdot) - c * sqrt(var(hdot)) >= -alpha(h),   c = PhiInv(1 - delta)

var(hdot) = dh/dx_r S_w dh/dx_r^T + dh/dx_h S_v dh/dx_h^T
```

`var(hdot)` does not depend on `u`, so the tightened constraint is still
linear in `u` and the QP stays a QP (not an SOCP).

For an MPC-CBF over the horizon, use `predict()[h]` for h in {2,4,8,12}: each
gives a calibrated per-horizon Gaussian tube directly — no covariance
propagation. Per the closed-loop study, those longer horizons belong ONLY in
the safety constraint, never in the tracking target (aiming the servo at
h >= 4 predictions was worse than no prediction at all).

## Caveats

- `sigma` is nearly state-independent (a per-horizon tube, not a per-state
  bound) and coverage is 0.87-0.93; multiply by ~1.15 for a strict 95% bound.
- The study used a kinematic rate-limited head with exactly one step of
  latency. If the real head has more effective latency L steps, aim at the
  prediction whose horizon matches L (h ~ L), not farther.
- Orientation residual (~6 deg) is marker noise floor: filtering the
  measurement, not smarter control, is the lever there.
