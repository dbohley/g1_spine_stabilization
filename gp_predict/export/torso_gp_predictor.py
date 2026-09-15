"""
Standalone torso-motion predictor: direct-H GP at 4 Hz.

Self-contained — needs only numpy and scipy (no torch, no training code).
Predicts the human-torso (AprilTag marker id 2) pose change fractions of a
second to ~2-3 s ahead (the loaded model set defines the grid), with
calibrated per-horizon uncertainty.  Current use: the one-step prediction
(or twist() mean) as the head-servo target — see CONTROLLER_MATH.md §1-4.
No CBF is implemented; sigma and the longer horizons are provided for
future use.

Frames and conventions (must match training — read before wiring in):
  * The predictor anchors itself on the FIRST pose you feed it after reset():
    anchor position p0 and anchor rotation R0.  All internal features and all
    "anchor-frame" outputs are expressed in that frame:
        p_rel = R0^T (p - p0),      R_rel = R0^T R
    Training episodes were clipped "half roll" segments anchored at segment
    start, so RESET AT THE START OF A MOTION SEGMENT, not hours earlier.
    Everything is invariant to rigid transforms of the camera/world frame.
  * The model runs at 4 Hz (248 ms steps).  Feed update() at any rate
    (e.g. the raw ~32 Hz marker stream); samples are internally accepted
    only when at least `accept_dt` has elapsed.  Predictions refresh on
    each accepted sample.
  * Targets are pose changes in the BODY FRAME AT THE ANCHOR TIME k:
        dp  = R_k^T (p_{k+h} - p_k)          [m]
        drv = log(R_k^T R_{k+h})             [rotation vector, rad]
  * sigma is the GP posterior std WITH observation noise included — this is
    the calibration that was validated (per-dim 95% coverage 0.87–0.93 on
    held-out episodes).  Units: m for dims 0..2, rad for dims 3..5.

Minimal use:
    from torso_gp_predictor import TorsoGP
    gp = TorsoGP('models')            # directory with the .npz files
    gp.reset()
    for t, p, R in pose_stream:       # t [s], p (3,) [m], R (3,3)
        if gp.update(t, p, R):        # True when an internal 4 Hz step ran
            out = gp.predict()        # {h: prediction dict}, see predict()
            tw  = gp.twist()          # 1-step mean/cov twist for the CBF
"""
import glob
import os
import numpy as np
from scipy.spatial.transform import Rotation as Rot

# Defaults of the original (lab half-roll) model set, kept for backward
# compatibility.  Since v2 each model set carries its own grid in
# normalization.npz (hset, step_dt); TorsoGP reads it from there — use
# gp.hset / gp.step_dt, not these constants, for new code.
HSET = (1, 2, 4, 8, 12)
STEP_DT = 0.248


class _GP:
    """Exact-GP posterior evaluation (ARD-RBF), loaded from cache."""

    def __init__(self, z, j):
        self.ll = z[f'{j}_ll']
        self.sf2 = float(z[f'{j}_sf2'])
        self.sn2 = float(z[f'{j}_sn2'])
        self.ymu = float(z[f'{j}_ymu'])
        self.ysd = float(z[f'{j}_ysd'])
        self.alpha = z[f'{j}_alpha']
        self.L = z[f'{j}_L']
        self.Xtr = z[f'{j}_X']
        self.noise_raw = self.sn2 * self.ysd**2

    def predict(self, Xs):
        A, B = Xs / self.ll, self.Xtr / self.ll
        d2 = (A**2).sum(1)[:, None] + (B**2).sum(1)[None, :] - 2 * A @ B.T
        Ks = self.sf2 * np.exp(-0.5 * np.maximum(d2, 0))
        mu = Ks @ self.alpha * self.ysd + self.ymu
        v = np.linalg.solve(self.L, Ks.T)
        var = np.maximum(self.sf2 - (v**2).sum(0), 0) * self.ysd**2
        return mu, var + self.noise_raw          # noise-inclusive (calibrated)


class TorsoGP:
    def __init__(self, model_dir, accept_dt=None):
        """model_dir: folder holding one `*_h{h}.npz` per horizon plus
        `normalization.npz` (which declares hset and step_dt — the model
        set's own grid, exposed as self.hset / self.step_dt).
        accept_dt: min seconds between internally accepted samples; default
        0.88 * step_dt, which decimates any faster jittery stream to ~the
        model's rate."""
        z = np.load(os.path.join(model_dir, 'normalization.npz'),
                    allow_pickle=False)
        self.xmu, self.xsd = z['xmu'], z['xsd']
        self.hset = tuple(int(h) for h in z['hset'])
        self.step_dt = float(z['step_dt'])
        self.gps = {}
        for h in self.hset:
            hits = glob.glob(os.path.join(model_dir, f'*_h{h}.npz'))
            if len(hits) != 1:
                raise FileNotFoundError(
                    f'expected exactly one *_h{h}.npz in {model_dir}, '
                    f'found {hits}')
            zh = np.load(hits[0], allow_pickle=False)
            self.gps[h] = [_GP(zh, j) for j in range(6)]
        self.accept_dt = (0.88 * self.step_dt if accept_dt is None
                          else accept_dt)
        self.reset()

    # ------------------------------------------------------------- stream --
    def reset(self):
        """Clear the anchor frame and history. Call at the start of each
        motion segment (the anchor is taken from the next update())."""
        self.p0 = self.R0 = None
        self._t = self._pr = self._Rr = None      # last ACCEPTED sample
        self._x = None                            # current 15-dim feature
        self.k_time = None                        # time of current feature

    def update(self, t, p, R):
        """Feed one raw pose sample (any rate). Returns True when a 4 Hz step
        was accepted AND a feature is available (predict() is now fresh)."""
        p = np.asarray(p, float)
        R = np.asarray(R, float)
        if self.p0 is None:                       # first sample = anchor
            self.p0, self.R0 = p.copy(), R.copy()
        if self._t is not None and t - self._t < self.accept_dt:
            return False
        pr = self.R0.T @ (p - self.p0)
        Rr = self.R0.T @ R
        if self._t is None:                       # no velocity yet
            self._t, self._pr, self._Rr = t, pr, Rr
            return False
        dt = t - self._t
        v = (pr - self._pr) / dt
        w = Rot.from_matrix(self._Rr.T @ Rr).as_rotvec() / dt
        self._x = np.concatenate([pr, Rr[:, 0], Rr[:, 1], v, w])
        self._t, self._pr, self._Rr = t, pr, Rr
        self.k_time = t
        return True

    # ------------------------------------------------------------ predict --
    def predict(self, horizons=None):
        """Predictions from the latest accepted state.  None until update()
        has returned True at least once.  Returns {h: dict} with
          t_ahead : seconds ahead (h * step_dt)
          dp      : (3,) mean position change, body frame at now  [m]
          drot    : (3,) mean rotation vector, body frame at now  [rad]
          sigma   : (6,) noise-inclusive std of [dp, drot]        [m, rad]
          p_rel, R_rel : predicted absolute pose in the ANCHOR frame
          p_world, R_world : same, mapped back to the raw input frame
        """
        if self._x is None:
            return None
        if horizons is None:
            horizons = self.hset
        xz = ((self._x - self.xmu) / self.xsd)[None]
        out = {}
        for h in horizons:
            mu = np.empty(6)
            var = np.empty(6)
            for j, g in enumerate(self.gps[h]):
                m, s2 = g.predict(xz)
                mu[j], var[j] = m[0], s2[0]
            dp, rv = mu[:3], mu[3:]
            p_rel = self._pr + self._Rr @ dp
            R_rel = self._Rr @ Rot.from_rotvec(rv).as_matrix()
            out[h] = dict(t_ahead=h * self.step_dt, dp=dp, drot=rv,
                          sigma=np.sqrt(var),
                          p_rel=p_rel, R_rel=R_rel,
                          p_world=self.p0 + self.R0 @ p_rel,
                          R_world=self.R0 @ R_rel)
        return out

    def twist(self):
        """Shortest-horizon prediction as a body-frame twist for the
        continuous-time CBF:  returns (mean (6,), var (6,)) of
        [v_lin (m/s), omega (rad/s)] — the h=hset[0] pose change divided by
        its duration T = hset[0] * step_dt.  var is the (diagonal) GP
        variance scaled by 1/T^2."""
        h1 = self.hset[0]
        o = self.predict(horizons=(h1,))
        if o is None:
            return None
        y = o[h1]
        T = h1 * self.step_dt
        mean = np.concatenate([y['dp'], y['drot']]) / T
        var = (y['sigma'] / T)**2
        return mean, var
