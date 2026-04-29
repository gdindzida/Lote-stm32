import cv2
import yaml
import os
import argparse
import numpy as np


def load_camchain(camchain_path):
    with open(camchain_path, "r") as f:
        data = yaml.safe_load(f)

    cam = data["cam0"]

    K = np.array(
        [
            [cam["intrinsics"][0], 0, cam["intrinsics"][2]],
            [0, cam["intrinsics"][1], cam["intrinsics"][3]],
            [0, 0, 1],
        ],
        dtype=np.float32,
    )

    D = np.array(cam["distortion_coeffs"], dtype=np.float32)

    return K, D


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
