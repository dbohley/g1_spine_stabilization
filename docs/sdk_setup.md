# SDK Setup — Per-Machine Ledger

This document records the specifics of the `g1_inspire_sdk` conda environment
on the development machine. It is **not** a rebuild recipe — that is the
[SDK README](https://github.com/Daniel891116/g1_inspire_sdk). This doc is
the per-machine ledger of what was installed, where, and how, so a future
session can orient itself quickly.

## Environment Overview

| Property | Value |
|----------|-------|
| Conda env name | `g1_inspire_sdk` |
| Python version | 3.12.13 |
| Created by | Hand install per the SDK README (out-of-session, SDK-1) |
| Miniforge root | `/home/moira/miniforge3` |
| Env path | `/home/moira/miniforge3/envs/g1_inspire_sdk` |

## SDK Install

The `g1_inspire_sdk` is installed editable from git:

```
-e git+https://github.com/Daniel891116/g1_inspire_sdk@981e9e18a23430a5f8d3930cf9cba465485d9478#egg=g1_inspire_sdk
```

The SDK commit pinned at install time is `981e9e18`. The clone lives at
`~/g1_inspire_sdk` (this is the editable project location reported by pip).

Sub-packages bundled in the SDK repo are also installed editable:

```
-e .../g1_inspire_sdk@981e9e18...#egg=inspire_sdkpy&subdirectory=third_party/inspire_hand_sdk
-e .../g1_inspire_sdk@981e9e18...#egg=unitree_sdk2py&subdirectory=third_party/unitree_sdk2_python
```

## Our Repo Install

`g1_spine_stabilization` is **importable from the repo root** via `sys.path`
when scripts or tests are run from `/home/moira/g1_spine_stabilization/`. It
is not yet pip-installed into this env. To make it persistently importable
from anywhere, run from the repo root:

```bash
conda run -n g1_inspire_sdk pip install -e .
```

## CycloneDDS

`cyclonedds==0.10.2` is installed from PyPI (no source build required on this
machine). `CYCLONEDDS_HOME` is not set in the shell environment; the PyPI
package handles its own library paths. There are minor account-specific quirks
for multi-user access on this machine, but those are not relevant to the repo
or the SDK examples.

## Packages Added Beyond the SDK README

These were added during SDK-2 preflight (not during the manual SDK-1 install):

| Package | Version | Why |
|---------|---------|-----|
| `pytest` | 9.1.1 | Core-stack item; was absent from the manual install |
| `coppeliasim-zmqremoteapi-client` | 2.0.4 | Core-stack item (needed by our test suite); absent |

## SDK Examples Verified

| Example | Mode | Result |
|---------|------|--------|
| `01_preview_joint_trajectory.py` | Preview (no robot) | Rerun visualizer renders correctly |
| `06_pick_and_place.py` | **Live robot** | Arm and hand commands executed; hardware path proven |
| `07_camera_preview.py` | Live camera | RealSense frames appear in Rerun |

## Notable Environment Notes

- `matplotlib` 3.11.0 is present (transitive dep). It does **not** leak at
  import time. The CLAUDE.md rule against importing
  `g1_inspire_sdk._common.weighted_moving_filter` remains critical — that
  import triggers a module-level `import matplotlib.pyplot`.
- `opencv-python` 4.13.0 is installed (not `opencv-contrib-python`). The
  `cv2.aruco` module works on this build; our `requirements.txt` still
  declares `opencv-contrib-python` per spec.
- `pinocchio` 3.9.0 from conda-forge — correct channel, includes casadi
  bindings. Do not `pip install pinocchio`; the PyPI package lacks the
  casadi integration.

## How to Rebuild

Follow the SDK README as the authoritative recipe. Refer to this file for
the per-machine specifics (env name, commit hash, packages added beyond the
README, account-specific CycloneDDS notes).

## conda list (snapshot at SDK-2)

```
# packages in environment at /home/moira/miniforge3/envs/g1_inspire_sdk:
#
# Name                             Version          Build                  Channel
_openmp_mutex                      4.5              20_gnu                 conda-forge
_x86_64-microarch-level            3                3_skylake              conda-forge
ampl-asl                           1.0.0            h5888daf_2             conda-forge
assimp                             6.0.5            h6b1a590_1             conda-forge
asttokens                          3.0.1            pypi_0                 pypi
attrs                              26.1.0           pypi_0                 pypi
bzip2                              1.0.8            hda65f42_9             conda-forge
c-ares                             1.34.6           hb03c661_0             conda-forge
ca-certificates                    2026.6.17        hbd8a1cb_0             conda-forge
casadi                             3.7.2            py312h5f23809_4        conda-forge
cbor                               1.0.0            pypi_0                 pypi
cli11                              2.6.2            h54a6638_0             conda-forge
click                              8.4.1            pypi_0                 pypi
coal                               3.0.2            h7900ff3_2             conda-forge
coal-python                        3.0.2            py312h58b0c77_2        conda-forge
colorama                           0.4.6            pypi_0                 pypi
colorcet                           3.2.1            pypi_0                 pypi
console_bridge                     1.0.2            h924138e_1             conda-forge
contourpy                          1.3.3            pypi_0                 pypi
coppeliasim-zmqremoteapi-client    2.0.4            pypi_0                 pypi
cycler                             0.12.1           pypi_0                 pypi
cyclonedds                         0.10.2           pypi_0                 pypi
decorator                          5.3.1            pypi_0                 pypi
eigen                              5.0.1            hc65338a_0             conda-forge
eigen-abi                          5.0.1.80         hf414acd_0             conda-forge
eigenpy                            3.12.0           np2py312hf9833ee_3     conda-forge
elfutils                           0.194            h849f50c_0             conda-forge
executing                          2.2.1            pypi_0                 pypi
fonttools                          4.63.0           pypi_0                 pypi
g1-inspire-sdk                     0.1.0            pypi_0                 pypi
gmp                                6.3.0            hac33072_2             conda-forge
gnutls                             3.8.13           h18acefa_0             conda-forge
hpp-fcl                            3.0.2            pyh2537366_0           conda-forge
icu                                78.3             h33c6efd_0             conda-forge
iniconfig                          2.3.0            pypi_0                 pypi
inspire-sdkpy                      1.0.0            pypi_0                 pypi
ipopt                              3.14.19          he45328a_2             conda-forge
ipython                            9.14.1           pypi_0                 pypi
ipython-pygments-lexers            1.1.1            pypi_0                 pypi
jedi                               0.20.0           pypi_0                 pypi
keyutils                           1.6.3            hb9d3cd8_0             conda-forge
kiwisolver                         1.5.0            pypi_0                 pypi
krb5                               1.22.2           hbde042b_1             conda-forge
ld_impl_linux-64                   2.45.1           default_hbd61a6d_102   conda-forge
libarchive                         3.8.7            gpl_hc2c16d8_101       conda-forge
libblas                            3.11.0           8_hc00574d_netlib      conda-forge
libblasfeo                         0.1.4.2          had105d5_300           conda-forge
libboost                           1.88.0           hd24cca6_7             conda-forge
libboost-devel                     1.88.0           hfcd1e18_7             conda-forge
libboost-headers                   1.88.0           ha770c72_7             conda-forge
libboost-python                    1.88.0           py312hf890105_7        conda-forge
libboost-python-devel              1.88.0           py312h26dfbe5_7        conda-forge
libcblas                           3.11.0           8_h8e06fc2_netlib      conda-forge
libcoal                            3.0.2            h6f6847b_2             conda-forge
libcurl                            8.20.0           hcf29cc6_0             conda-forge
libedit                            3.1.20250104     pl5321h7949ede_0       conda-forge
libev                              4.33             hd590300_2             conda-forge
libexpat                           2.8.1            hecca717_1             conda-forge
libfatrop                          0.0.4            h5888daf_1             conda-forge
libffi                             3.5.2            h3435931_0             conda-forge
libgcc                             15.2.0           he0feb66_19            conda-forge
libgcc-ng                          15.2.0           h69a702a_19            conda-forge
libgfortran                        15.2.0           h69a702a_19            conda-forge
libgfortran5                       15.2.0           h68bc16d_19            conda-forge
libgomp                            15.2.0           he0feb66_19            conda-forge
libgz-cmake3                       3.5.5            h54a6638_0             conda-forge
libgz-cmake4                       4.2.0            h54a6638_1             conda-forge
libgz-math7                        7.5.2            h54a6638_2             conda-forge
libgz-tools                        2.0.3            h54a6638_1             conda-forge
libgz-utils2                       2.2.1            h54a6638_2             conda-forge
libhwloc                           2.13.0           default_he001693_1000  conda-forge
libiconv                           1.18             h3b78370_2             conda-forge
libidn2                            2.3.8            hfac485b_1             conda-forge
liblapack                          3.11.0           8_h8876d29_netlib      conda-forge
liblzma                            5.8.3            hb03c661_0             conda-forge
libmicrohttpd                      1.0.5            hc2fc477_0             conda-forge
libnghttp2                         1.68.1           h877daf1_0             conda-forge
libnsl                             2.0.1            hb9d3cd8_1             conda-forge
libosqp                            1.0.0            np2py312h1a77e3e_2     conda-forge
libpinocchio                       3.9.0            h1c65005_1             conda-forge
libqdldl                           0.1.8            h3f2d84a_1             conda-forge
libscotch                          7.0.11           int64_h807e49d_3       conda-forge
libsdformat14                      14.8.0           py314h67520d3_3        conda-forge
libspral                           2025.09.18       h74cae3c_2             conda-forge
libsqlite                          3.53.2           h0c1763c_0             conda-forge
libssh2                            1.11.1           hcf80075_0             conda-forge
libstdcxx                          15.2.0           h934c35e_19            conda-forge
libstdcxx-ng                       15.2.0           hdf11a46_19            conda-forge
libtasn1                           4.21.0           hb03c661_1             conda-forge
libunistring                       0.9.10           h7f98852_0             conda-forge
libuuid                            2.42.2           h5347b49_0             conda-forge
libxcrypt                          4.4.36           hd590300_1             conda-forge
libxml2                            2.15.3           h49c6c72_0             conda-forge
libxml2-16                         2.15.3           hca6bf5a_0             conda-forge
libzlib                            1.3.2            h25fd6f3_2             conda-forge
lz4-c                              1.10.0           h5888daf_1             conda-forge
lzo                                2.10             h280c20c_1002          conda-forge
markdown-it-py                     4.2.0            pypi_0                 pypi
matplotlib                         3.11.0           pypi_0                 pypi
matplotlib-inline                  0.2.2            pypi_0                 pypi
mdurl                              0.1.2            pypi_0                 pypi
meshcat                            0.3.2            pypi_0                 pypi
metis                              5.1.0            hd0bcaf9_1007          conda-forge
mumps-include                      5.8.2            h5a610fb_2             conda-forge
mumps-seq                          5.8.2            hc1b3267_2             conda-forge
ncurses                            6.6              hdb14827_0             conda-forge
nettle                             3.10.1           h4a9d5aa_0             conda-forge
numpy                              2.5.0            py312h33ff503_0        conda-forge
octomap                            1.10.0           h84d6215_0             conda-forge
opencv-python                      4.13.0.92        pypi_0                 pypi
openssl                            3.6.3            h35e630c_0             conda-forge
p11-kit                            0.26.2           h3435931_0             conda-forge
packaging                          26.2             pyhc364b38_0           conda-forge
parso                              0.8.7            pypi_0                 pypi
pexpect                            4.9.0            pypi_0                 pypi
pillow                             12.2.0           pypi_0                 pypi
pinocchio                          3.9.0            h7900ff3_1             conda-forge
pinocchio-python                   3.9.0            py312h5447bb0_1        conda-forge
pip                                26.1.2           pyh8b19718_0           conda-forge
pluggy                             1.6.0            pypi_0                 pypi
prompt-toolkit                     3.0.52           pypi_0                 pypi
psutil                             7.2.2            pypi_0                 pypi
ptyprocess                         0.7.0            pypi_0                 pypi
pure-eval                          0.2.3            pypi_0                 pypi
pyarrow                            24.0.0           pypi_0                 pypi
pygments                           2.20.0           pypi_0                 pypi
pymodbus                           3.6.9            pypi_0                 pypi
pyngrok                            8.1.2            pypi_0                 pypi
pyparsing                          3.3.2            pypi_0                 pypi
pyqt5                              5.15.11          pypi_0                 pypi
pyqt5-qt5                          5.15.19          pypi_0                 pypi
pyqt5-sip                          12.18.0          pypi_0                 pypi
pyqtgraph                          0.14.0           pypi_0                 pypi
pyrealsense2                       2.58.2.10647     pypi_0                 pypi
pyserial                           3.5              pypi_0                 pypi
pytest                             9.1.1            pypi_0                 pypi
python                             3.12.13          hd63d673_0_cpython     conda-forge
python-dateutil                    2.9.0.post0      pypi_0                 pypi
python_abi                         3.12             8_cp312                conda-forge
pyyaml                             6.0.3            py312h8a5da7c_1        conda-forge
pyzmq                              27.1.0           pypi_0                 pypi
qhull                              2020.2           h434a139_5             conda-forge
qhull-static                       2020.2           h434a139_5             conda-forge
readline                           8.3              h853b02a_0             conda-forge
rerun-sdk                          0.33.1           pypi_0                 pypi
rich                               15.0.0           pypi_0                 pypi
rich-click                         1.9.8            pypi_0                 pypi
ruby                               4.0.5            h95e6935_0             conda-forge
scipy                              1.18.0           py312h54fa4ab_0        conda-forge
setuptools                         82.0.1           pyh332efcf_0           conda-forge
six                                1.17.0           pypi_0                 pypi
stack-data                         0.6.3            pypi_0                 pypi
tinyxml2                           11.0.0           h3f2d84a_0             conda-forge
tk                                 8.6.13           noxft_h366c992_103     conda-forge
tornado                            6.5.7            pypi_0                 pypi
traitlets                          5.15.1           pypi_0                 pypi
typing-extensions                  4.15.0           pypi_0                 pypi
tzdata                             2025c            hc9c84f9_1             conda-forge
u-msgpack-python                   2.8.0            pypi_0                 pypi
unitree-sdk2py                     1.0.1            pypi_0                 pypi
urdfdom                            5.1.2            h8631160_0             conda-forge
urdfdom_headers                    2.1.2            hb700be7_0             conda-forge
wcwidth                            0.8.1            pypi_0                 pypi
wheel                              0.47.0           pyhd8ed1ab_0           conda-forge
yaml                               0.2.5            h280c20c_3             conda-forge
zlib                               1.3.2            h25fd6f3_2             conda-forge
zstd                               1.5.7            hb78ec9c_6             conda-forge
```

## pip freeze (snapshot at SDK-2)

```
asttokens==3.0.1
attrs==26.1.0
casadi==3.7.2
cbor==1.0.0
click==8.4.1
colorama==0.4.6
colorcet==3.2.1
contourpy==1.3.3
coppeliasim_zmqremoteapi_client==2.0.4
cycler==0.12.1
cyclonedds==0.10.2
decorator==5.3.1
executing==2.2.1
fonttools==4.63.0
-e git+https://github.com/Daniel891116/g1_inspire_sdk@981e9e18a23430a5f8d3930cf9cba465485d9478#egg=g1_inspire_sdk
iniconfig==2.3.0
-e git+https://github.com/Daniel891116/g1_inspire_sdk@981e9e18a23430a5f8d3930cf9cba465485d9478#egg=inspire_sdkpy&subdirectory=third_party/inspire_hand_sdk
ipython==9.14.1
ipython_pygments_lexers==1.1.1
jedi==0.20.0
kiwisolver==1.5.0
markdown-it-py==4.2.0
matplotlib==3.11.0
matplotlib-inline==0.2.2
mdurl==0.1.2
meshcat==0.3.2
numpy @ file:///home/conda/feedstock_root/build_artifacts/bld/rattler-build_numpy_1782112536/work/dist/numpy-2.5.0-cp312-cp312-linux_x86_64.whl
opencv-python==4.13.0.92
packaging @ file:///home/conda/feedstock_root/build_artifacts/bld/rattler-build_packaging_1777103621/work
parso==0.8.7
pexpect==4.9.0
pillow==12.2.0
pluggy==1.6.0
prompt_toolkit==3.0.52
psutil==7.2.2
ptyprocess==0.7.0
pure_eval==0.2.3
pyarrow==24.0.0
Pygments==2.20.0
pymodbus==3.6.9
pyngrok==8.1.2
pyparsing==3.3.2
PyQt5==5.15.11
PyQt5-Qt5==5.15.19
PyQt5_sip==12.18.0
pyqtgraph==0.14.0
pyrealsense2==2.58.2.10647
pyserial==3.5
pytest==9.1.1
python-dateutil==2.9.0.post0
PyYAML @ file:///home/conda/feedstock_root/build_artifacts/pyyaml_1770223234623/work
pyzmq==27.1.0
rerun-sdk==0.33.1
rich==15.0.0
rich-click==1.9.8
scipy @ file:///home/conda/feedstock_root/build_artifacts/scipy-split_1781912125494/work/dist/scipy-1.18.0-cp312-cp312-linux_x86_64.whl
setuptools==82.0.1
six==1.17.0
stack-data==0.6.3
tornado==6.5.7
traitlets==5.15.1
typing_extensions==4.15.0
u-msgpack-python==2.8.0
-e git+https://github.com/Daniel891116/g1_inspire_sdk@981e9e18a23430a5f8d3930cf9cba465485d9478#egg=unitree_sdk2py&subdirectory=third_party/unitree_sdk2_python
wcwidth==0.8.1
wheel==0.47.0
```
