from Cython.Build import cythonize
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup(
    packages=["fastslam"],
    package_dir={"": "src"},
    ext_modules=cythonize(
        [
            "./src/fastslam/*.py",
        ],
        annotate=True,
    ),
    zip_safe=False,
)

setup(**d)