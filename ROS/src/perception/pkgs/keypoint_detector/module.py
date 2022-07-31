import torch
from keypoint_detector.nn.cross_ratio_loss import CrossRatioLoss
from keypoint_detector.nn.model import RektNet
from pytorch_lightning.core.lightning import LightningModule
from torch import optim


class RektNetModule(LightningModule):
    def __init__(
        self,
        lr: float = 1e-4,
        lr_gamma: float = 0.999,
        batch_size: int = 64,
        img_width: int = 60,
        img_height: int = 80,
        loss_type: str = "l1_softargmax",
        num_epochs: int = 200,
        enable_geo_loss: bool = True,
        geo_loss_gamma_vert: float = 80,
        geo_loss_gamma_horz: float = 60,
        apply_augmentation: bool = True,
    ):
        super().__init__()

        self.save_hyperparameters()

        self.model = RektNet()
        self.loss_function = CrossRatioLoss(
            self.hparams.loss_type,
            self.hparams.enable_geo_loss,
            self.hparams.geo_loss_gamma_horz,
            self.hparams.geo_loss_gamma_vert,
        )

        self.example_input_array = torch.randn(
            (1, 3, self.hparams.img_height, self.hparams.img_width)
        )

    def forward(self, x):
        return self.model(x)

    def step(self, batch, batch_idx):
        imgs, true_heatmaps, true_points = batch

        # Compute output and loss.
        pred_heatmaps, pred_points = self.model(imgs)
        loc_loss, geo_loss, loss = self.loss_function(
            pred_heatmaps, pred_points, true_heatmaps, true_points
        )

        return loss, loc_loss, geo_loss

    def training_step(self, batch, batch_idx):
        loss, loc_loss, geo_loss = self.step(batch, batch_idx)

        self.log("loss", loss, on_step=True, on_epoch=True, prog_bar=True, logger=True)
        self.log(
            "geo_loss",
            geo_loss,
            on_step=True,
            on_epoch=True,
            prog_bar=True,
            logger=True,
        )
        self.log(
            "loc_loss",
            loc_loss,
            on_step=True,
            on_epoch=True,
            prog_bar=True,
            logger=True,
        )

        return loss

    def validation_step(self, batch, batch_idx):
        loss, loc_loss, geo_loss = self.step(batch, batch_idx)
        self.log("val_loss", loss)

    def test_step(self, batch, batch_idx):
        loss, loc_loss, geo_loss = self.step(batch, batch_idx)
        self.log("test_loss", loss)

    def configure_optimizers(self):
        optimizer = optim.Adam(
            self.model.parameters(),
            lr=self.hparams.lr,
        )
        scheduler = optim.lr_scheduler.ExponentialLR(
            optimizer, gamma=self.hparams.lr_gamma
        )

        return [optimizer], [scheduler]
