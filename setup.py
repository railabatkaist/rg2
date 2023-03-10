import os

import pkg_resources
from setuptools import find_packages, setup


setup(
    name="rg2",
    py_modules=["rg2"],
    version="0.0.1",
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
    include_package_data=True,
)
