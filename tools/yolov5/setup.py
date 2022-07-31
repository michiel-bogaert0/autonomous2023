from typing import List

import setuptools

requirements: List[str] = [
    "cython",
    "tqdm",
    "seaborn"
]

setuptools.setup(
    author="Lucas Van Dijck",
    author_email="lucas.vandijck@ugentracing.be",
    python_requires=">=3.8",
    description="yolov5",
    install_requires=requirements,
    packages=setuptools.find_packages(),
    include_package_data=True,
    name="yolov5",
    version="0.0.2",
)
