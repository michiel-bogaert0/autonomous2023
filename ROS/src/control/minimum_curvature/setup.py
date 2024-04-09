from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["scripts", "utils", "plotting_utils", "data"],
    package_dir={"": "pkgs"},
)
setup(**d)
