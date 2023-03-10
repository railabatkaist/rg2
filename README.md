# RaiGym2 : Fast, Pythonic, Versatile environment design for learning based locomotion
[WIP]
## Development

### Building C++ Gym Environment

Prepare required installations, such as pybind11 and Eigen. For Eigen, one can use the following command to install it in the system.

```bash
$ sudo apt install libeigen3-dev
```

Then, build the C++ Gym environment.


```bash
$ cmake -S . -B build
$ cmake --build build -j 2
```

This will generate a shared library at `src/bin`. For which, you can develop your own Python wrapper defined at `src/rg2`

## TODO

- [ ] Stable CI/CD package deployment using github actions
- [ ] Documented user friendly installation with pip
- [ ] Curriculums : pythonic modification on Ground configuration and Domain Randomization
- [ ] URDF to action-space mapping : pythonically definable actuators configurations