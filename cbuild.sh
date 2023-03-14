cmake -S . -B build
cmake --build build
pybind11-stubgen rg2.bin --no-setup --bare-numpy-ndarray
mv stubs/rg2/bin-stubs/_rg2/__init__.pyi rg2/bin/__init__.pyi
pip install .