from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    package_dir={"": "pkgs"},
    install_requires=[
        "scikit-learn>=1.2.2",
        "pytest",
        "pre-commit",
        "scikit-image",
        "matplotlib>=3.6.2",
        # for labeling package, should be moved in time to separate setup.py
        "xmltodict",
        "pydantic",
        "plotly",
    ],
)
setup(**d)
