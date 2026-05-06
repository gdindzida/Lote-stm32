"""Camera calibration utilities for loading camchain.yaml files."""

import yaml
from typing import Dict


def load_camchain(yaml_path: str) -> Dict[str, float]:
    """Load camera calibration parameters from camchain.yaml.

    Args:
        yaml_path: Path to camchain.yaml file

    Returns:
        Dictionary with keys: fx, fy, cx, cy, k1, k2

    Raises:
        FileNotFoundError: If yaml_path doesn't exist
        KeyError: If required fields are missing in YAML
        ValueError: If calibration data format is invalid

    Example camchain.yaml format:
        cam0:
          intrinsics: [fx, fy, cx, cy]
          distortion_coeffs: [k1, k2, p1, p2]
          distortion_model: radtan
    """
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    if "cam0" not in data:
        raise KeyError("camchain.yaml must contain 'cam0' key")

    cam = data["cam0"]

    # Extract intrinsics: [fx, fy, cx, cy]
    if "intrinsics" not in cam:
        raise KeyError("cam0 must contain 'intrinsics' field")

    intrinsics = cam["intrinsics"]
    if len(intrinsics) < 4:
        raise ValueError(
            f"intrinsics must have at least 4 values, got {len(intrinsics)}"
        )

    # Extract distortion coefficients: [k1, k2, ...]
    if "distortion_coeffs" not in cam:
        raise KeyError("cam0 must contain 'distortion_coeffs' field")

    distortion = cam["distortion_coeffs"]
    if len(distortion) < 2:
        raise ValueError(
            f"distortion_coeffs must have at least 2 values, got {len(distortion)}"
        )

    calibration = {
        "fx": float(intrinsics[0]) * 96 / 2056,
        "fy": float(intrinsics[1]) * 96 / 1542,
        "cx": float(intrinsics[2]),
        "cy": float(intrinsics[3]),
        "k1": float(distortion[0]),
        "k2": float(distortion[1]),
    }

    return calibration
