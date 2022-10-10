#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["abb_robot_driver_state_machine"],
    package_dir={"": "src"},
)

setup(**setup_args)
