from typing import List

from setuptools import setup

requirements: List[str] = [
    "cython",
    "tqdm"
]

setup(
    author="Lucas Van Dijck",
    author_email="lucas.vandijck@ugentracing.be",
    python_requires=">=3.8",
    classifiers=[],
    description="yolov5",
    install_requires=requirements,
    include_package_data=True,
    name="yolov5",
    version="0.0.2",
)
