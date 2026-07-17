"""Pose MLP + BatchNorm folding for the 6844 firmware port.

Architecture is TI's (retraining notebook cell 13), retargeted to 4 classes:

    bn1 -> fc1(IN->64) -> relu -> bn2 -> fc2(64->32) -> relu
        -> bn3 -> fc3(32->16) -> bn4 -> fc4(16->NCLS)

Two deliberate deviations from the notebook:

* TI's forward() ends in `torch.softmax(...)` and the training loop feeds that
  to `nn.CrossEntropyLoss`, which itself applies log_softmax -- i.e. softmax is
  applied twice, squashing the gradients.  We return logits and let the loss
  own the softmax; `predict_proba` applies it at inference.
* 4 output classes, not 5 (walking dropped -- see dataset.py).

Every BatchNorm here sits immediately before a Linear, so at inference all four
fold away exactly (fold_bn below), leaving a plain
`fc1' -> relu -> fc2' -> relu -> fc3' -> fc4'` that the firmware can evaluate
with nothing but multiply-add.  That is what makes the R5F port tractable
without TVM: no .a to link, no M-profile objects, no runtime.
"""

from __future__ import annotations

import numpy as np
import torch
import torch.nn as nn


class PoseMLP(nn.Module):
    def __init__(self, input_size: int, num_classes: int):
        super().__init__()
        self.bn1 = nn.BatchNorm1d(input_size)
        self.fc1 = nn.Linear(input_size, 64)
        self.bn2 = nn.BatchNorm1d(64)
        self.fc2 = nn.Linear(64, 32)
        self.bn3 = nn.BatchNorm1d(32)
        self.fc3 = nn.Linear(32, 16)
        self.bn4 = nn.BatchNorm1d(16)
        self.fc4 = nn.Linear(16, num_classes)
        self.relu = nn.ReLU()

    def forward(self, x):                      # -> logits
        out = self.fc1(self.bn1(x))
        out = self.relu(out)
        out = self.fc2(self.bn2(out))
        out = self.relu(out)
        out = self.fc3(self.bn3(out))
        out = self.fc4(self.bn4(out))
        return out

    @torch.no_grad()
    def predict_proba(self, x):
        return torch.softmax(self.forward(x), dim=1)


def fold_bn(bn: nn.BatchNorm1d, fc: nn.Linear) -> tuple[np.ndarray, np.ndarray]:
    """Fold `fc(bn(x))` into a single affine map (W', b').

    BN at inference is elementwise affine:  bn(x) = a*x + c,  where
        a = gamma / sqrt(running_var + eps),  c = beta - a * running_mean
    so   fc(bn(x)) = W(a*x + c) + b = (W * a) x + (W @ c + b).
    """
    a = (bn.weight / torch.sqrt(bn.running_var + bn.eps)).detach()
    c = (bn.bias - a * bn.running_mean).detach()
    W, b = fc.weight.detach(), fc.bias.detach()
    return (W * a).numpy().astype(np.float64), (W @ c + b).numpy().astype(np.float64)


def fold_model(m: PoseMLP) -> list[tuple[np.ndarray, np.ndarray]]:
    """The whole net as four folded (W, b) layers, in evaluation order."""
    m.eval()
    return [fold_bn(m.bn1, m.fc1), fold_bn(m.bn2, m.fc2),
            fold_bn(m.bn3, m.fc3), fold_bn(m.bn4, m.fc4)]


def folded_forward(layers, x: np.ndarray) -> np.ndarray:
    """Reference implementation of exactly what the firmware will compute.

    relu after layers 0 and 1 only -- layer 2 (fc3) has no activation in TI's
    graph, so fc3 and fc4 are adjacent linear maps.
    """
    out = np.asarray(x, dtype=np.float64)
    for i, (W, b) in enumerate(layers):
        out = out @ W.T + b
        if i < 2:
            out = np.maximum(out, 0.0)
    e = np.exp(out - out.max(axis=-1, keepdims=True))
    return e / e.sum(axis=-1, keepdims=True)
