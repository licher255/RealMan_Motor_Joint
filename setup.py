#!/usr/bin/env python3
"""
Setup script for RealMan WHJ Driver Python bindings.

Usage:
    pip install .
    # or
    python setup.py install
"""

from pybind11.setup_helpers import Pybind11Extension, build_ext
from pybind11 import get_cmake_dir
from setuptools import setup, find_packages

# Get version from CMakeLists.txt
import re
with open("CMakeLists.txt", "r") as f:
    content = f.read()
    version_match = re.search(r"project\(realman_whj_driver VERSION ([\d.]+)", content)
    version = version_match.group(1) if version_match else "1.0.0"

# Extension module
ext_modules = [
    Pybind11Extension(
        "realman_whj",
        sources=[
            "src/python/bindings.cpp",
            "src/core/types.cpp",
            "src/core/driver.cpp",
            "src/platform/can_interface.cpp",
        ],
        include_dirs=["include"],
        cxx_std=17,
        # Platform-specific defines
        define_macros=[("VERSION_INFO", f'"{version}"')],
    ),
]

setup(
    name="realman-whj",
    version=version,
    author="RealMan Driver Team",
    author_email="support@example.com",
    description="RealMan WHJ Joint Motor Driver - Python Bindings",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/realman_whj_driver",
    license="MIT",
    packages=find_packages(),
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
    install_requires=[
        "pybind11>=2.6.0",
    ],
    extras_require={
        "dev": ["pytest", "black", "flake8"],
    },
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: C++",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: System :: Hardware",
    ],
)
