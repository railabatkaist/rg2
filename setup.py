import os

import pkg_resources
from setuptools import find_packages, setup

from glob import glob

print(list(glob("rg2/linux/lib/**", recursive=True)))

cpver = os.environ.get("CPVER", None)
cpver = cpver.replace(".", "") if cpver else None

setup(
    name="rg2",
    py_modules=["rg2"],
    version="0.0.7",
    description="RaiGym2 : Fast, Pythonic, Versatile environment design for learning based locomotion",
    author="Simo Ryu",
    packages=find_packages(),
    install_requires=[
        str(r)
        for r in pkg_resources.parse_requirements(
            open(os.path.join(os.path.dirname(__file__), "requirements.txt"))
        )
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    data_files=[
        (
            "rg2",
            list(
                set(
                    list(glob("rg2/linux/lib/*.so*", recursive=True))
                    + list(glob(f"rg2/bin/_rg2.cpython-{cpver}*"))
                )   
                - set(["rg2/linux/lib/"])
            ),
        )
    ],
    include_package_data=True,
)
