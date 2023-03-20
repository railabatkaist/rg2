

# iterate over all versions of python
for version in 3.6 3.7 3.8 3.9 3.10
do
    eval "$(conda shell.bash hook)"
    # activate the environment
    conda activate rg$version
    # remove the build directory
    rm -rf build
    rm -rf rg2/bin/*.so
    # run cmake
    cmake -S . -B build -DPYTHON_EXECUTABLE=$(which python) -DPYTHON_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") -DPYTHON_LIBRARY=$(python3 -c "import distutils.sysconfig as sysconfig; print(sysconfig.get_config_var('LIBDIR'))") -DPYTHON_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") -DCMAKE_BUILD_TYPE=Release
    # build the project
    cmake --build build
    export CPVER=$version
    # remove . from CPVER
    export CPVER=${CPVER/./}
    # setup
    python setup.py bdist_wheel --python-tag cp$CPVER

    # deactivate the environment
    conda deactivate
done