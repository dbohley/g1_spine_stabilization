"""
End-to-end check of the robot-platform model set (models_robot/) through the
streaming TorsoGP API.  Run from export/ with the dataset in the parent dir:
    python verify_robot_export.py
Expected (n=200 set from sweep_gp_size.py, 2026-09-14):
    pos  cm: 0.43  0.67  0.99  1.68     at 0.23 / 0.46 / 0.91 / 2.06 s
    ori deg: 1.03  1.62  2.62  4.55
"""
import os, sys, numpy as np
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from retrain_robot_gp import load_ep, targets_h, geo
from torso_gp_predictor import TorsoGP

EXPECT = {1: (0.43, 1.03), 2: (0.67, 1.62), 4: (0.99, 2.62), 9: (1.68, 4.55)}


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    mdir = os.path.join(here, 'models_robot')
    z = np.load(os.path.join(mdir, 'normalization.npz'))
    gp = TorsoGP(mdir, accept_dt=0.0)
    err = {h: dict(p=[], o=[]) for h in gp.hset}
    for f in z['test_files']:
        e = load_ep(os.path.join(here, '..', str(f)))
        gp.reset()
        preds = {}
        for k in range(e['N']):
            if gp.update(e['t'][k], e['p'][k], e['R'][k]):
                preds[k] = gp.predict()
        for h in gp.hset:
            ks, Yt = targets_h(e, h)
            mu = np.array([np.concatenate([preds[k][h]['dp'],
                                           preds[k][h]['drot']]) for k in ks])
            err[h]['p'].append(np.linalg.norm(mu[:, :3] - Yt[:, :3], axis=1) * 100)
            err[h]['o'].append(geo(mu[:, 3:], Yt[:, 3:]))

    ok = True
    print(f'{"h":>3s} {"t(s)":>5s} | {"pos cm":>7s} {"expect":>7s} '
          f'| {"ori deg":>8s} {"expect":>7s}')
    for h in gp.hset:
        pe = float(np.mean(np.concatenate(err[h]['p'])))
        oe = float(np.mean(np.concatenate(err[h]['o'])))
        print(f'{h:3d} {h*gp.step_dt:5.2f} | {pe:7.2f} {EXPECT[h][0]:7.2f} '
              f'| {oe:8.2f} {EXPECT[h][1]:7.2f}')
        ok &= abs(pe - EXPECT[h][0]) < 0.02 and abs(oe - EXPECT[h][1]) < 0.02
    print('\nVERIFY', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
