from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['object_detection'],
    package_dir={'': 'src'},
    package_data={'object_detection': ['data/*.yaml']},  # tabla oficial de AprilTags
)
setup(**setup_args)
