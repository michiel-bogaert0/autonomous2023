from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["BE_dc", "BE"],
    package_dir={"": "pkgs"},
)
setup(**d)
