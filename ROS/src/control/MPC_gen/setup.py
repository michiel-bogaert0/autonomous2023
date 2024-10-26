from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["environments_gen", "optimal_control_gen", "scripts_gen", "utils_gen"],
    package_dir={"": "pkgs"},
)
setup(**d)
