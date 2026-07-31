#!/usr/bin/env python3
"""Run the zero-observation ONNX reference for student_policy."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import NoReturn


EXPECTED_INPUT_SHAPE = [1, 55]
EXPECTED_OUTPUT_SHAPE = [1, 3]


def fail(message: str) -> NoReturn:
    print(f"ERROR: {message}", file=sys.stderr)
    raise SystemExit(1)


def main() -> int:
    project_root = Path(__file__).resolve().parents[1]
    model_path = project_root / "models" / "model_1100.onnx"
    reference_path = project_root / "tools" / "onnx_zero_reference.txt"

    if not model_path.is_file():
        fail(f"ONNX model is missing: {model_path}")

    try:
        import numpy as np
        import onnxruntime as ort
    except ImportError as exc:
        fail(
            "onnxruntime and numpy are required. Install them in the active "
            f"Python environment ({exc})."
        )

    try:
        session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    except Exception as exc:  # noqa: BLE001 - provide a useful model-load error
        fail(f"could not load {model_path}: {exc}")

    inputs = session.get_inputs()
    outputs = session.get_outputs()
    if len(inputs) != 1:
        fail(f"expected one model input, found {len(inputs)}")
    if len(outputs) != 1:
        fail(f"expected one model output, found {len(outputs)}")

    model_input = inputs[0]
    model_output = outputs[0]
    print("Model:", model_path)
    print(
        f"Input:  name={model_input.name!r}, shape={model_input.shape}, "
        f"type={model_input.type}"
    )
    print(
        f"Output: name={model_output.name!r}, shape={model_output.shape}, "
        f"type={model_output.type}"
    )

    if list(model_input.shape) != EXPECTED_INPUT_SHAPE:
        fail(
            f"input dimensions differ from {EXPECTED_INPUT_SHAPE}: "
            f"{model_input.shape}"
        )
    if list(model_output.shape) != EXPECTED_OUTPUT_SHAPE:
        fail(
            f"output dimensions differ from {EXPECTED_OUTPUT_SHAPE}: "
            f"{model_output.shape}"
        )
    if model_input.type != "tensor(float)":
        fail(f"input type is not float32: {model_input.type}")
    if model_output.type != "tensor(float)":
        fail(f"output type is not float32: {model_output.type}")

    observation = np.zeros(EXPECTED_INPUT_SHAPE, dtype=np.float32)
    try:
        actions = session.run([model_output.name], {model_input.name: observation})[0]
    except Exception as exc:  # noqa: BLE001 - provide a useful inference error
        fail(f"zero-input inference failed: {exc}")

    if list(actions.shape) != EXPECTED_OUTPUT_SHAPE:
        fail(f"runtime output dimensions differ from {EXPECTED_OUTPUT_SHAPE}: {actions.shape}")

    actions = np.asarray(actions, dtype=np.float32)
    print("Zero-input actions:")
    for index, value in enumerate(actions[0]):
        print(f"  actions[0][{index}] = {float(value):.12f}")

    reference = {
        "observation_shape": EXPECTED_INPUT_SHAPE,
        "observation": observation.reshape(-1).tolist(),
        "actions_shape": EXPECTED_OUTPUT_SHAPE,
        "actions": actions.reshape(-1).tolist(),
    }
    reference_path.write_text(json.dumps(reference, indent=2) + "\n", encoding="utf-8")
    print(f"Saved comparison reference: {reference_path}")
    return 0


if __name__ == "__main__":
    main()
