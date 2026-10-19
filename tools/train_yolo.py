"""Train the YOLO26-OBB shape detector and export it for the Pi (Stage B).

Runs on the PC only: the Pi has no torch and only loads the exported ONNX
file. Every hyper-parameter comes from the `training` section of
config/settings.yaml, so a retrain in the lab never needs a code edit (T11).

The dataset is the one written by tools/auto_label.py and contains only the
three assigned shapes, so the trained model cannot see the other two (T1).

Usage:
    python tools/train_yolo.py                 # train, then export
    python tools/train_yolo.py --export-only   # re-export the last best.pt
"""

import argparse
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.core.config import Config  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
EXPORTED_MODEL_STEM = "shapes_yolo26n_obb"


def best_weights_path(settings: Config) -> Path:
    """Checkpoint ultralytics keeps for the epoch with the best val mAP."""
    run_dir = Path(settings.get("runs_dir")) / settings.get("run_name")
    return run_dir / "weights" / "best.pt"


def augmentation_arguments(augment: Config) -> dict:
    """Map our unit-named settings onto ultralytics' short argument names."""
    return {
        "hsv_h": float(augment.get("hue_shift_fraction")),
        "hsv_s": float(augment.get("saturation_gain_fraction")),
        "hsv_v": float(augment.get("value_gain_fraction")),
        "degrees": float(augment.get("rotation_deg")),
        "flipud": float(augment.get("flip_up_down_probability")),
        "fliplr": float(augment.get("flip_left_right_probability")),
    }


def train(settings: Config) -> Path:
    """Fine-tune the pretrained weights on the lab dataset; return best.pt."""
    from ultralytics import YOLO  # imported late: the Pi never reaches here

    model = YOLO(settings.get("base_weights"))
    model.train(
        data=settings.get("dataset_yaml"),
        imgsz=int(settings.get("image_size_px")),
        epochs=int(settings.get("epochs")),
        patience=int(settings.get("patience_epochs")),
        batch=int(settings.get("batch_size")),
        workers=int(settings.get("dataloader_workers")),
        seed=int(settings.get("random_seed")),
        project=settings.get("runs_dir"),
        name=settings.get("run_name"),
        exist_ok=True,
        # Validate (and therefore early-stop) on the NMS-free one-to-one head
        # that is exported to the Pi. The default validates the one-to-many
        # head through NMS instead: measured 2026-09-18, that reported recall
        # 1.00 while the shipped head only reached 0.95.
        nms=False,
        **augmentation_arguments(settings.section("augment")),
    )
    return best_weights_path(settings)


def export(weights_path: Path, settings: Config) -> Path:
    """Write the .pt and .onnx pair into the delivery folder; return the .onnx."""
    from ultralytics import YOLO

    if not weights_path.is_file():
        raise FileNotFoundError(f"no trained weights at {weights_path}")
    export_dir = PROJECT_ROOT / settings.get("export_dir")
    export_dir.mkdir(parents=True, exist_ok=True)

    onnx_source = Path(
        YOLO(str(weights_path)).export(
            format="onnx",
            imgsz=int(settings.get("export_image_size_px")),
            simplify=True,
            # ultralytics 8.4 keeps the NMS-free one-to-one head only when nms
            # is exactly False; the default None silently exports the raw
            # (1, 8, 8400) head, which would need NMS code on the Pi.
            nms=False,
        )
    )
    onnx_target = export_dir / f"{EXPORTED_MODEL_STEM}.onnx"
    shutil.copy2(onnx_source, onnx_target)
    shutil.copy2(weights_path, export_dir / f"{EXPORTED_MODEL_STEM}.pt")
    return onnx_target


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train and export YOLO26-OBB.")
    parser.add_argument("--export-only", action="store_true")
    parser.add_argument("--config", default=None, type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    settings = Config.load(args.config).section("training")
    weights_path = best_weights_path(settings) if args.export_only else train(settings)
    onnx_path = export(weights_path, settings)
    print(f"[train_yolo] exported {onnx_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
