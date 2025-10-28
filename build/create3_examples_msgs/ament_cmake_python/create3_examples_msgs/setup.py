from setuptools import find_packages
from setuptools import setup

setup(
    name='create3_examples_msgs',
    version='1.0.0',
    packages=find_packages(
        include=('create3_examples_msgs', 'create3_examples_msgs.*')),
)
