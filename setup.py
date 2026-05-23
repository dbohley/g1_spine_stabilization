from setuptools import find_packages, setup

setup(
    name="g1_spine_stabilization",
    version="0.1.0",
    description="Bimanual rigid-body pose tracking for the Unitree G1.",
    packages=find_packages(exclude=("tests", "tests.*", "scripts", "scripts.*")),
    python_requires=">=3.10",
)
