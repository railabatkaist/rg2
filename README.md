# RaiGym2 : Fast, Pythonic, Versatile environment design for learning based locomotion

[WIP]

## Development

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
