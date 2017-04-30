#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['src/drone_pos_node.py',
  'src/drone_demo_node.py'],
)

setup(**d)