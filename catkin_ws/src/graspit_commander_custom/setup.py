## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

requirements = [line.strip() for line in open("requirements.txt")]

d = generate_distutils_setup()

d['name'] = "graspit_commander_custom"
d['description'] = "Python interface for GraspIt! simulator"
d['packages'] = ['graspit_commander_custom']
d['package_dir'] = {'': 'src'}
d['requirements'] = requirements

setup(**d)

