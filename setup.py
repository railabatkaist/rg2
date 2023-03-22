import os

import pkg_resources
from setuptools import find_packages, setup

from glob import glob

from rg2 import __version__

print(list(glob("rg2/linux/lib/**", recursive=True)))

CPVER = os.environ.get("CPVER", None)
CPVER = CPVER.replace(".", "") if CPVER else None

setup(
    name="rg2",
    py_modules=["rg2"],
    version=__version__,
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
                    + list(glob(f"rg2/bin/_rg2.cpython-{CPVER}*"))
                )   
                - set(["rg2/linux/lib/"])
            ),
        )
    ],
    entry_points={
        "console_scripts": [
            "rg2 = rg2.clis.cli_inits:main"
        ],
    },
    include_package_data=True,
    package_data={"rg2": ["**"]}
)
