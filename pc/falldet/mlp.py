"""Module 2 — MLP pose/fall classifier (TI Pose_And_Fall approach, sklearn proxy).

TI's on-chip net: 22 features x 8 frames -> 5-class softmax (Stood/Sat/Lying/Falling/
Walking). Features per frame = track kinematics (posZ, vel[3], acc[3]) + top-5 highest
points (y-posY, z, snr). We reproduce the feature layout and a 64-32-16 MLP (the exact
weights live in TI's compiled pose_model.a; this is a faithful proxy for evaluation).

Strengths (measured on TI data): great at the FALLING MOTION (~96%). Weaknesses: weak
on the sustained LYING state (~62%), and its posZ/vel/acc features BREAK when the track
freezes/ghosts during a fall -> pairs with WindowDetector, which covers exactly that.
"""
TOPK = 5
WIN = 8


def frame_features(posz, vel, acc, points, posy=0.0):
    """22 features. points = list of (x, y, z, snr). vel/acc = (x,y,z), missing -> 0.
    TI's feature uses (y-posY, z, snr) of the top-5 highest points (no point-x)."""
    top = sorted(points, key=lambda p: p[2], reverse=True)[:TOPK]   # by height z
    top = top + [(0.0, 0.0, 0.0, 0.0)] * (TOPK - len(top))
    kin = [posz, vel[0], vel[1], vel[2], acc[0], acc[1], acc[2]]
    return kin + [v for (x, y, z, s) in top for v in (y - posy, z, s)]


class MLPDetector:
    """Wraps an sklearn MLP over 8-frame feature windows (176-dim)."""
    def __init__(self, model=None):
        self.model = model

    def train(self, X, y, hidden=(64, 32, 16), max_iter=400, seed=0):
        from sklearn.neural_network import MLPClassifier
        self.model = MLPClassifier(hidden_layer_sizes=hidden, max_iter=max_iter,
                                   random_state=seed).fit(X, y)
        return self

    @staticmethod
    def window(feat_frames):
        """Concatenate WIN frames of 22 features -> a 176-dim sample (None if too short)."""
        if len(feat_frames) < WIN:
            return None
        return [v for fr in feat_frames[-WIN:] for v in fr]

    def predict(self, x176):
        """Returns dict(pose, probs, falling_p) for one 176-dim window."""
        import numpy as np
        p = self.model.predict_proba(np.asarray(x176).reshape(1, -1))[0]
        classes = list(self.model.classes_)
        probs = dict(zip(classes, p.tolist()))
        pose = classes[int(np.argmax(p))]
        falling_p = probs.get("falling", 0.0) + probs.get("lying", 0.0)   # "person down" prob
        return {"pose": pose, "probs": probs, "falling_p": falling_p}
