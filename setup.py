#!/usr/bin/env python3

from wheel.bdist_wheel import bdist_wheel as bdist_wheel_
from setuptools import setup, Extension, Command
from distutils.util import get_platform

import glob
import sys
import os

directory = os.path.dirname(os.path.realpath(__file__))

requires = [
        "zencad"
    ] 

setup(
    name="zencad_dynamic",
    packages=["zencad_dynamic"],
    version="0.0.1",
    license="MIT",
    description="Dynamic solver for zencad",
    author="mirmik",
    author_email="netricks@protonmail.com",
    url="https://github.com/mirmik/zencad-dynamic",
    long_description=open(os.path.join(directory, "README.md"), "r").read(),
    long_description_content_type="text/markdown",
    keywords=["math"],
    classifiers=[],
    package_data={
        "zencad-dynamic": []
    },
)
