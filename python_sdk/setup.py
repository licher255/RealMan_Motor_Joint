"""
Setup script for RealMan WHJ Python SDK.

For modern installation, use:
    pip install -e .

Or install from source:
    pip install .
"""

from setuptools import setup, find_packages

setup(
    name="realman-whj",
    use_scm_version=True,
    setup_requires=['setuptools_scm'],
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    python_requires=">=3.8",
)
