import cv2
import yaml
import os
import argparse
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


def center_crop(img, crop_size):
    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2

    half = crop_size // 2

    x1 = max(cx - half, 0)
    y1 = max(cy - half, 0)

    x2 = x1 + crop_size
    y2 = y1 + crop_size

    return img[y1:y2, x1:x2]


def process(input_dir, output_dir, K, D, crop_size=256, out_size=96):
    os.makedirs(output_dir, exist_ok=True)

    # get sample image
    sample = None
    for f in os.listdir(input_dir):
        if f.lower().endswith((".png", ".jpg", ".jpeg")):
            sample = cv2.imread(os.path.join(input_dir, f))
            break

    if sample is None:
        raise ValueError("No images found")

    h, w = sample.shape[:2]

    # TODO write new K to new camchain file
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))

    map1, map2 = cv2.initUndistortRectifyMap(K, D, None, newK, (w, h), cv2.CV_16SC2)

    for fname in sorted(os.listdir(input_dir)):
        if not fname.lower().endswith((".png", ".jpg", ".jpeg")):
            continue

        img = cv2.imread(os.path.join(input_dir, fname))
        if img is None:
            continue

        # 1. undistort
        rect = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)

        # 2. grayscale
        gray = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)

        # 3. center crop (256 or 512)
        cropped = center_crop(gray, crop_size)

        # 4. resize to 96x96
        small = cv2.resize(cropped, (out_size, out_size), interpolation=cv2.INTER_AREA)

        out_path = os.path.join(output_dir, fname)
        cv2.imwrite(out_path, small)

        print(f"Processed: {fname}")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--camchain", required=True)

    parser.add_argument(
        "--crop", type=int, default=256, help="crop size: 256 or 512 recommended"
    )

    parser.add_argument("--size", type=int, default=96, help="final output size")

    args = parser.parse_args()

    K, D = load_camchain(args.camchain)

    print("Using crop:", args.crop, "→ output:", args.size)

    process(args.input, args.output, K, D, args.crop, args.size)


if __name__ == "__main__":
    main()
