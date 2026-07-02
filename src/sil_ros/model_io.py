#!/usr/bin/env python
import os
import torch
import rospy
from sil_ros.interaction_manager import LatentTaskEncoder

def save_encoder(encoder, path: str, metadata: dict = None):
    os.makedirs(os.path.dirname(path) if os.path.dirname(path) else ".", exist_ok=True)
    hidden_dims = []
    for layer in encoder.encoder:
        if isinstance(layer, torch.nn.Linear):
            hidden_dims.append(layer.out_features)
    if hidden_dims:
        hidden_dims = hidden_dims[:-1]
    checkpoint = {
        "state_dict": encoder.state_dict(),
        "config": {
            "input_dim": encoder.encoder[0].in_features,
            "hidden_dims": hidden_dims,
            "latent_dim": encoder.encoder[-2].out_features,  # Linear before Tanh
            "dropout": metadata.get("dropout", 0.2) if metadata else 0.2,
        },
        "metadata": metadata or {},
    }
    torch.save(checkpoint, path)
    rospy.loginfo(f"[SIL] Encoder saved: {path}")

def load_encoder(encoder, path: str) -> bool:
    if not os.path.exists(path):
        rospy.logwarn(f"[SIL] Encoder checkpoint not found: {path}")
        return False
    checkpoint = torch.load(path, map_location="cpu")
    encoder.load_state_dict(checkpoint["state_dict"])
    rospy.loginfo(f"[SIL] Encoder loaded: {path} "
                  f"(config={checkpoint.get('config', 'N/A')})")
    return True

def create_encoder_from_checkpoint(path: str):
    checkpoint = torch.load(path, map_location="cpu")
    config = checkpoint["config"]
    encoder = LatentTaskEncoder(**config)
    encoder.load_state_dict(checkpoint["state_dict"])
    return encoder
