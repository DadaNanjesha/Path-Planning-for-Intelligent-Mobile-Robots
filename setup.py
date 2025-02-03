from setuptools import setup, find_packages

setup(
    name="PathPlanning",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        "networkx==3.4.2",
        "matplotlib==3.10.0",
    ],
)
