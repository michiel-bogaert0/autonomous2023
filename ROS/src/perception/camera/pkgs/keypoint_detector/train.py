from dataclasses import asdict
from pathlib import Path

import wandb
from argparse_dataclass import ArgumentParser
from keypoint_detector.data import KeyPointDataset
from keypoint_detector.module import RektNetModule
from keypoint_detector.param import RektNetTrainParam
from pytorch_lightning import Trainer
from pytorch_lightning.callbacks import ModelCheckpoint
from pytorch_lightning.loggers import WandbLogger

dataset_dir = Path.home() / "datasets" / "rektnet"
checkpoint_dir = str(Path.cwd() / "checkpoints")
print(checkpoint_dir)

train_param_parser = ArgumentParser(RektNetTrainParam)
train_param = train_param_parser.parse_args()


model = RektNetModule()

wandb_config = {**asdict(train_param), **model.hparams}
run = wandb.init(config=wandb_config, project="RektNet")

wandb_logger = WandbLogger(project="RektNet")

data = KeyPointDataset(model.hparams, train_param, data_folder=dataset_dir)


checkpoint_callback = ModelCheckpoint(
    monitor="val_loss",
    dirpath=checkpoint_dir,
    filename="best",
    save_top_k=1,
    mode="min",
)

trainer = Trainer(
    gpus=1,
    max_epochs=model.hparams.num_epochs,
    logger=wandb_logger,
    check_val_every_n_epoch=5,
    callbacks=[checkpoint_callback],
)
trainer.fit(model, data)
result = trainer.test()

model.to_onnx("rektnet.onnx", export_params=True, opset_version=11)

artifact = wandb.Artifact("RektNet_ONNX", type="RektNet_ONNX")
artifact.add_file("rektnet.onnx")
run.log_artifact(artifact)
