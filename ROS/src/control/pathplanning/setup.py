from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["pathplanning_dc", "rrt", "triangulation"],
    package_dir={"": "pkgs"},
)
setup(**d)
