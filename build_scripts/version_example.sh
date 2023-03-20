# iterate over all versions of python
for version in 3.6 3.7 3.8 3.9 3.10
do
    # create a new conda environment
    conda create -n rg$version python=$version -y
    # activate the environment
    conda activate rg$version
    # remove the build directory
    rm -rf build
    # run cmake
    cmake -S . -B build -DPYTHON_EXECUTABLE=$(which python) -DPYTHON_INCLUDE_DIR=$(python -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") -DPYTHON_LIBRARY=$(python -c "import distutils.sysconfig as sysconfig; print(sysconfig.get_config_var('LIBDIR'))") -DPYTHON_PACKAGES_PATH=$(python -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") -DCMAKE_BUILD_TYPE=Release
    # build the project
    cmake --build build
    # deactivate the environment
    conda deactivate
done