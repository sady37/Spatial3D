"""Self-contained tests for the pose MLP fold + reference forward.

No dataset needed (classes.zip lives in the gitignored ti_ref tree). Guards the
BatchNorm folding math and the numpy reference that the firmware mirrors. Needs
torch; skipped if the isolated pose venv isn't the active interpreter.
"""

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from pose.dataset import INPUT_SIZE
from pose.model import PoseMLP, fold_model, folded_forward


def test_bn_fold_matches_torch():
    torch.manual_seed(0)
    m = PoseMLP(INPUT_SIZE, 4)
    # Give the BN running stats non-trivial values (eval() uses them, not batch).
    for bn in (m.bn1, m.bn2, m.bn3, m.bn4):
        bn.running_mean.normal_()
        bn.running_var.uniform_(0.5, 1.5)
        bn.weight.data.normal_()
        bn.bias.data.normal_()
    m.eval()

    layers = fold_model(m)
    x = np.random.default_rng(1).standard_normal((32, INPUT_SIZE)).astype(np.float32)
    with torch.no_grad():
        ref = m.predict_proba(torch.from_numpy(x)).numpy()
    got = folded_forward(layers, x.astype(np.float64))
    assert np.abs(ref - got).max() < 1e-5
    # argmax must agree on every row
    assert (ref.argmax(1) == got.argmax(1)).all()


def test_folded_forward_is_a_distribution():
    torch.manual_seed(2)
    layers = fold_model(PoseMLP(INPUT_SIZE, 4).eval())
    x = np.random.default_rng(3).standard_normal((10, INPUT_SIZE))
    p = folded_forward(layers, x)
    np.testing.assert_allclose(p.sum(1), 1.0, atol=1e-6)
    assert (p >= 0).all() and (p <= 1).all()
