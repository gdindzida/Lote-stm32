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


def center_crop(img: np.ndarray, crop_w: int, crop_h: int) -> np.ndarray:
    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2

    half_w = crop_w // 2
    half_h = crop_h // 2

    x1 = max(cx - half_w, 0)
    y1 = max(cy - half_h, 0)

    x2 = x1 + crop_w
    y2 = y1 + crop_h

    return img[y1:y2, x1:x2]


def process(
    input_dir: str,
    output_dir: str,
    K: np.ndarray,
    D: np.ndarray,
    crop_size: tuple[int, int] = (256, 256),
    out_size: tuple[int, int] = (96, 96),
) -> None:
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

        # 3. center crop
        cropped = center_crop(gray, crop_size[0], crop_size[1])

        # 4. resize to out_size
        small = cv2.resize(
            cropped, (out_size[0], out_size[1]), interpolation=cv2.INTER_AREA
        )

        out_path = os.path.join(output_dir, fname)
        cv2.imwrite(out_path, small)

        print(f"Processed: {fname}")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--camchain", required=True)

    parser.add_argument(
        "--crop",
        type=int,
        nargs=2,
        default=[256, 256],
        metavar=("W", "H"),
        help="crop size as width and height (e.g. --crop 256 256)",
    )

    parser.add_argument(
        "--size",
        type=int,
        nargs=2,
        default=[96, 96],
        metavar=("W", "H"),
        help="final output size as width and height (e.g. --size 96 96)",
    )

    args = parser.parse_args()

    K, D = load_camchain(args.camchain)

    crop_size = tuple(args.crop)
    out_size = tuple(args.size)

    print(
        f"Using crop: {crop_size[0]}x{crop_size[1]} → output: {out_size[0]}x{out_size[1]}"
    )

    process(args.input, args.output, K, D, crop_size, out_size)


if __name__ == "__main__":
    main()
