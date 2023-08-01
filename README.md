# RaiGym2 : Fast, Pythonic, Versatile environment design for learning based locomotion

# Installation

```bash
pip install git+https://github.com/railabatkaist/rg2.git
```

Steps:

0. Environment gets initialized. There are various aspect of Noise configurations, such as
   - Per-episode noise (These include ground config, initial position, command, etc.)
   - Per-sim-timestep noise (These include friction, simulation time-step, etc.)
   - Per-step noise (These include observation noise, action noise, etc.)
1. Agent watches the Observation. This includes command list.
2. Agent takes Action.
   - Here, the action is happened between $t_{i}$ and $t_{i+n}$, where $n$ is the number of steps for which the simulation is integrated. (Physics engine is integrated for $n$ steps. Every sim-timestep, the per-sim-timestep noise is applied, and every step, the per-step noise is applied.)

## Local Development

### Building C++ Gym Environment

Prepare required installations, such as pybind11 and Eigen (For Eigen, one can use the following command to install it in the system)

```bash
$ sudo apt install libeigen3-dev
```

Then, build the C++ Gym environment.

```bash
$ cmake -S . -B build
$ cmake --build build -j 2
```

Do

```bash
$ python setup.py sdist bdist_wheel
```

With python version:

```bash
$ python setup.py bdist_wheel --python-tag=cp3.8 #--plat-name=win_amd64
```

To build python distribution.

Setup twine to upload the package to PyPI.

```bash
$ python -m pip install --user --upgrade twine
```

Then, upload the package to PyPI.

```bash
$ python -m twine upload (--repository testpypi) dist/*
```

This will generate a shared library at `src/bin`. For which, you can develop your own Python wrapper defined at `src/rg2`

## Develop locally

```bash
$ bash cbuild.sh
```

This will build C++ Gym and generate Stub to further make interface.

## Test

Run tests at `tests/` directory.

```bash
$ python -m unittest tests/test_cenv.py
```

Test Pypi Install

```
python3 -m pip install --index-url https://test.pypi.org/simple/ rg2
```

## TODO

- [ ] Stable CI/CD package deployment using github actions
- [ ] Documented user friendly installation with pip
- [ ] Curriculums : pythonic modification on Ground configuration and Domain Randomization
- [ ] URDF to action-space mapping : pythonically definable actuators configurations

Examples require
\*stable-baselines3==1.7.0

-
