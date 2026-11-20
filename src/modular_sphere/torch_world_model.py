"""PyTorch MLP world model (optional extra ``modular-sphere[torch]``).

Deterministic one-step dynamics model as in the model-based RL literature (PETS, Chua et al. 2018;
MBPO, Janner et al. 2019): an MLP predicts the normalized change ``obs[t+1] - obs[t]`` from the
normalized ``(obs[t], action[t])`` and is rolled out autoregressively. Deviation from those
references: a single deterministic network instead of a probabilistic ensemble.
"""

from __future__ import annotations

import itertools
import logging
from pathlib import Path

import numpy as np
import torch
from numpy.typing import NDArray
from torch import nn

from modular_sphere.dataset import TrajectoryDataset

Array = NDArray[np.float64]
EPS = 1e-6  # Small constant to prevent division by zero in normalization
logger = logging.getLogger(__name__)


class MLPWorldModel:
    """Implements ``WorldModel`` (``rollout``) and ``fit``; numpy in, numpy out."""

    def __init__(self, hidden: int = 256, layers: int = 3, epochs: int = 50,
                 batch_size: int = 1024, lr: float = 1e-3, weight_decay: float = 1e-5,
                 seed: int = 0, device: str | None = None) -> None:  # fmt: skip
        self.hidden, self.layers = hidden, layers
        self.epochs, self.batch_size, self.lr, self.weight_decay = (
            epochs,
            batch_size,
            lr,
            weight_decay,
        )
        self.seed = seed
        if device is not None and device not in ("cpu", "cuda"):
            logger.warning(f"Unknown device {device!r}, falling back to default")
            device = None
        if device == "cuda" and not torch.cuda.is_available():
            logger.warning("CUDA requested but not available, falling back to CPU")
            device = "cpu"
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.net: nn.Sequential | None = None
        self.stats: dict[str, torch.Tensor] = {}
        self.train_losses: list[float] = []

    def _build(self, obs_dim: int, act_dim: int) -> nn.Sequential:
        dims = [obs_dim + act_dim] + [self.hidden] * self.layers
        blocks: list[nn.Module] = []
        for d_in, d_out in itertools.pairwise(dims):
            blocks += [nn.Linear(d_in, d_out), nn.SiLU()]
        return nn.Sequential(*blocks, nn.Linear(self.hidden, obs_dim)).to(self.device)

    def _tensor(self, x: Array) -> torch.Tensor:
        return torch.as_tensor(x, dtype=torch.float32, device=self.device)

    def fit(self, dataset: TrajectoryDataset) -> None:
        torch.manual_seed(self.seed)
        obs = self._tensor(dataset.obs[:, :-1].reshape(-1, dataset.obs.shape[-1]))
        act = self._tensor(dataset.actions.reshape(-1, dataset.actions.shape[-1]))
        delta = self._tensor(np.diff(dataset.obs, axis=1).reshape(-1, dataset.obs.shape[-1]))
        self.stats = {"obs_mean": obs.mean(0), "obs_std": obs.std(0) + EPS,
                      "delta_mean": delta.mean(0), "delta_std": delta.std(0) + EPS}  # fmt: skip
        x = torch.cat([(obs - self.stats["obs_mean"]) / self.stats["obs_std"], act], dim=-1)
        y = (delta - self.stats["delta_mean"]) / self.stats["delta_std"]
        self.net = net = self._build(obs.shape[-1], act.shape[-1])
        optim = torch.optim.AdamW(net.parameters(), lr=self.lr, weight_decay=self.weight_decay)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(optim, T_max=self.epochs)
        gen = torch.Generator(device="cpu").manual_seed(self.seed)
        self.train_losses = []
        for _ in range(self.epochs):
            perm = torch.randperm(x.shape[0], generator=gen).to(self.device)
            total = 0.0
            for idx in perm.split(self.batch_size):
                loss = nn.functional.mse_loss(net(x[idx]), y[idx])
                optim.zero_grad(set_to_none=True)
                loss.backward()
                optim.step()
                total += loss.item() * idx.numel()
            sched.step()
            self.train_losses.append(total / x.shape[0])

    @torch.no_grad()
    def rollout(self, obs0: Array, actions: Array) -> Array:
        net = self.net
        if net is None:
            raise RuntimeError("MLPWorldModel.fit (or load) must be called before rollout")
        net.eval()
        s = self.stats
        obs = self._tensor(obs0)
        acts = self._tensor(actions)
        preds = torch.empty((*acts.shape[:2], obs.shape[-1]), device=self.device)
        for k in range(acts.shape[1]):
            x = torch.cat([(obs - s["obs_mean"]) / s["obs_std"], acts[:, k]], dim=-1)
            obs = obs + net(x) * s["delta_std"] + s["delta_mean"]
            preds[:, k] = obs
        return preds.double().cpu().numpy()

    def save(self, path: str | Path) -> None:
        net = self.net
        if net is None:
            raise RuntimeError("nothing to save: call fit first")
        dims = (net[0].in_features, net[-1].out_features)
        torch.save({"hidden": self.hidden, "layers": self.layers, "state": net.state_dict(),
                    "stats": self.stats, "dims": dims}, path)  # fmt: skip

    @classmethod
    def load(cls, path: str | Path, device: str | None = None) -> MLPWorldModel:
        model = cls(device=device)
        ckpt = torch.load(path, map_location=model.device, weights_only=True)
        model.hidden, model.layers = ckpt["hidden"], ckpt["layers"]
        in_dim, obs_dim = ckpt["dims"]
        model.net = net = model._build(obs_dim, in_dim - obs_dim)
        net.load_state_dict(ckpt["state"])
        model.stats = {k: v.to(model.device) for k, v in ckpt["stats"].items()}
        return model
