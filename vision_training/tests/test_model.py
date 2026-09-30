import torch

from crackseg.inference import predict_logits
from crackseg.losses import BceDiceLoss
from crackseg.models import UNet


def test_unet_keeps_spatial_size():
    model = UNet(base_channels=8).eval()
    x = torch.randn(2, 3, 64, 96)
    assert model(x).shape == (2, 1, 64, 96)


def test_odd_image_size_is_padded_and_cropped_back():
    # 480x270 is not divisible by 16, like many real camera crops
    model = UNet(base_channels=8).eval()
    x = torch.randn(1, 3, 270, 480)
    assert predict_logits(model, x, multiple=16).shape == (1, 1, 270, 480)


def test_loss_prefers_the_right_answer():
    target = torch.zeros(1, 1, 16, 16)
    target[..., 8, :] = 1
    good = (target * 2 - 1) * 10  # confident and correct logits
    bad = -good
    loss = BceDiceLoss()
    assert loss(good, target) < loss(bad, target)


def test_one_training_step_reduces_loss():
    torch.manual_seed(0)
    model = UNet(base_channels=4)
    x = torch.randn(2, 3, 32, 32)
    y = (torch.rand(2, 1, 32, 32) > 0.9).float()
    loss_fn = BceDiceLoss()
    opt = torch.optim.Adam(model.parameters(), lr=1e-2)
    first = loss_fn(model(x), y)
    for _ in range(5):
        opt.zero_grad()
        loss = loss_fn(model(x), y)
        loss.backward()
        opt.step()
    assert loss_fn(model(x), y) < first
