cmake -S . -B build
cmake --build build
pip install .
pybind11-stubgen rg2.bin --no-setup --bare-numpy-ndarray