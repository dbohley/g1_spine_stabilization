"""
End-to-end check of the exported predictor against the recorded test set.

Run from the export/ directory with the dataset available in the parent dir:
    python verify_export.py
Feeds each held-out test episode through the streaming TorsoGP API and
recomputes the handoff §6 error table.  Expected (regenerated 2026-09-07):
    pos  cm: 0.61  0.82  1.30  2.17  2.96
    ori deg: 6.15  6.56  7.03  7.31  7.37
Also writes ../plots/export_verify_ep0.png (predicted-vs-true overlay).
"""
import os, sys, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import gp_phase_so3 as M                     # dataset loader (+ pickle stubs)
M.STRIDE = 8                                 # 4 Hz, matches the export

from torso_gp_predictor import TorsoGP, HSET, STEP_DT

EXPECT = {1: (0.61, 6.15), 2: (0.82, 6.56), 4: (1.30, 7.03),
          8: (2.17, 7.31), 12: (2.96, 7.37)}


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    z = np.load(os.path.join(here, 'models', 'normalization.npz'))
    test_f = [os.path.join(here, '..', f) for f in z['test_files']]

    gp = TorsoGP(os.path.join(here, 'models'), accept_dt=0.0)
    err = {h: dict(p=[], o=[]) for h in HSET}
    ep0 = None
    for ei, f in enumerate(test_f):
        p, R, t = M.load_run(f)              # stride-8 samples, anchor frame
        gp.reset()
        preds = {}                           # k -> predict() output
        for k in range(len(t)):
            if gp.update(t[k], p[k], R[k]):
                preds[k] = gp.predict()
        for h in HSET:
            for k in range(1, len(t) - h):
                y = preds[k][h]
                dp_t = R[k].T @ (p[k + h] - p[k])
                dR_t = R[k].T @ R[k + h]
                err[h]['p'].append(np.linalg.norm(y['dp'] - dp_t) * 100)
                err[h]['o'].append(np.degrees(
                    (Rot.from_matrix(y['R_rel'].T) *
                     Rot.from_matrix(R[k] @ dR_t)).magnitude()))
        if ei == 0:
            ep0 = (f, p, R, t, preds)

    print(f'{"h":>3s} {"t(s)":>5s} {"n":>5s} | {"pos cm":>7s} {"expect":>7s} '
          f'| {"ori deg":>8s} {"expect":>7s}')
    ok = True
    for h in HSET:
        pe, oe = np.mean(err[h]['p']), np.mean(err[h]['o'])
        n = len(err[h]['p'])
        print(f'{h:3d} {h*STEP_DT:5.2f} {n:5d} | {pe:7.2f} {EXPECT[h][0]:7.2f} '
              f'| {oe:8.2f} {EXPECT[h][1]:7.2f}')
        ok &= abs(pe - EXPECT[h][0]) < 0.02 and abs(oe - EXPECT[h][1]) < 0.02
    print('\nVERIFY', 'PASS' if ok else 'FAIL',
          '- exported predictor reproduces the handoff §6 table' if ok else
          '- numbers do not match the handoff table!')

    # overlay plot from the exported streaming path, test episode 0
    f, p, R, t, preds = ep0
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    h = 12
    ks = np.array(sorted(k for k in preds if k + h < len(t)))
    pp = np.array([preds[k][h]['p_rel'] for k in ks]) * 100
    for i, c in enumerate('xyz'):
        ax[0].plot(t - t[0], p[:, i] * 100, c=f'C{i}', lw=1.5, label=f'true {c}')
        ax[0].plot(t[ks + h] - t[0], pp[:, i], '.', c=f'C{i}', label=f'pred {c}')
    ang_t = np.degrees(Rot.from_matrix(R).magnitude())
    ang_p = np.degrees(Rot.from_matrix(
        np.array([preds[k][h]['R_rel'] for k in ks])).magnitude())
    ax[1].plot(t - t[0], ang_t, c='k', lw=1.5, label='true')
    ax[1].plot(t[ks + h] - t[0], ang_p, '.', c='C3', label='pred')
    ax[0].set_ylabel('position (cm)'); ax[1].set_ylabel('rotation (deg)')
    ax[1].set_xlabel('time (s)')
    for a in ax:
        a.grid(alpha=.3); a.legend(fontsize=8, ncols=2)
    fig.suptitle(f'Exported TorsoGP, streaming API — h=12 ({h*STEP_DT:.2f} s '
                 f'ahead), test ep 0')
    out = os.path.join(here, '..', 'plots', 'export_verify_ep0.png')
    fig.tight_layout(); fig.savefig(out, dpi=150)
    print('overlay saved:', os.path.normpath(out))
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
