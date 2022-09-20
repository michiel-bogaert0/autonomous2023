import setuptools

setuptools.setup(
    name="pathplanning",  # Formerly pathfinding
    version="0.0.1",
    author="Kilian Janssens",
    author_email="kilian.janssens@ugentracing.be",
    description="",
    long_description="",
    long_description_content_type="text/markdown",
    packages=["pathplanning"],
    package_dir={"pathplanning": "src/pathplanning"},
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.8",
    install_requires=[
        "numpy==1.21.0",
        "matplotlib==3.4.2",
        "scipy==1.7.0",
        "pyyaml==5.4.1",
        "click>=8.0.4",
    ],
    extras_require={
        "test": [
            "black==20.8b1",
            "isort==5.7.0",
            "mypy==0.800",
            "pytest==6.2.2",
            "autoflake==1.4",
        ]
    },
)
