from setuptools import setup

setup(
    name="keypoint_detection",
    author="Thomas Lips",
    author_email="thomas.lips@ugent.be",
    version="1.0",
    description="Pytorch Models, Modules etc for keypoint detection",
    url="https://github.com/tlpss/keypoint-detection",
    packages=["keypoint_detection"],
    install_requires=[
        "torch>=0.10",
        "onnx>=1.13.0",
        "torchvision>=0.11",
        "pytorch-lightning==1.9.4",
        "torchmetrics>=0.7",
        "wandb<=0.13.4",  # artifact bug https://github.com/wandb/wandb/issues/4500
        "timm>=0.6.11",  # requires smallsized convnext models
        "scikit-learn>=1.2.2",
        "tqdm",
        "pytest",
        "pre-commit",
        "scikit-image",
        "albumentations",
        "matplotlib>=3.6.2",
        # for labeling package, should be moved in time to separate setup.py
        "xmltodict",
        "pydantic",
        "plotly",
    ],
)
