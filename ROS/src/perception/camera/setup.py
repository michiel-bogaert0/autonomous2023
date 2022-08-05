from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        "yolov5",
        "keypoint_detector",
        "cone_detection",
        "keypoint_detection",
        "pnp",
        "publisher_abstract"
    ],
    package_dir={"": "pkgs"},
)
setup(**d)
