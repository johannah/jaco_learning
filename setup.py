
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    name='ros_interface',
    packages=['ros_interface',
              ],
    install_requires=[],
    package_dir={'ros_interface': 'ros_interface'},
)

setup(**setup_args)
