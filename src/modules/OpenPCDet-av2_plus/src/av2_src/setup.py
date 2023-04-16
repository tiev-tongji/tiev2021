import sys
import os
import subprocess

from setuptools import find_packages, setup
from setuptools.command.install import install


if __name__ == '__main__':
    setup(
        name='av2',
        version='0.2.1',
        description="argoverse2 api",
        author='Argo AI',
        author_email='argoverse-api@argo.ai',
        url='https://github.com/argoai/av2-api',
        long_description='file: README.md',
        install_requires =[
            'av',
            'click',
            'joblib',
            'matplotlib',
            'nox',
            'numba',
            'numpy',
            'opencv-python',
            'pandas',
            'pyarrow',
            'pyproj',
            'rich',
            'scipy'
        ],
        packages=find_packages(where='./', exclude=['tools', 'data', 'output'])
    )