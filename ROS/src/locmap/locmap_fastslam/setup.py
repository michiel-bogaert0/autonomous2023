from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup
from Cython.Build import cythonize

d = generate_distutils_setup(
    packages=["fastslam"],
    package_dir={"": "src"},
    ext_modules=cythonize(
        [
            "./src/fastslam/*.py",
        ],
        annotate=True,
    ),
    compiler_directives={"language_level": "3"},
)

setup(**d)
