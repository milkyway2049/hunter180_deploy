#!/usr/bin/env python3
"""Verify and import an AMP_mjlab checkpoint's exported actor into this workspace.

Run with the training Python environment (torch, onnx, onnxruntime, PyYAML).
This reads training artifacts and never starts an environment or hardware driver.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import tempfile

import numpy as np
import onnx
import onnxruntime as ort
import torch
from torch import nn
import yaml


ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def import_policy(run_dir, checkpoint_name):
    if checkpoint_name == "latest":
        checkpoints = list(run_dir.glob("model_[0-9]*.pt"))
        if not checkpoints:
            raise FileNotFoundError(f"No model checkpoints under {run_dir}")
        checkpoint_name = max(checkpoints, key=lambda p: int(p.stem.split("_")[-1])).name
    checkpoint = run_dir / checkpoint_name
    source = run_dir / "policy.onnx"
    env_path = run_dir / "params/env.yaml"
    agent_path = run_dir / "params/agent.yaml"
    env = yaml.load(env_path.read_text(), Loader=yaml.BaseLoader)
    agent = yaml.load(agent_path.read_text(), Loader=yaml.BaseLoader)
    saved = torch.load(checkpoint, map_location="cpu", weights_only=False)
    metadata_source = source.read_bytes()
    model = onnx.load_model_from_string(metadata_source)
    onnx.checker.check_model(model)
    meta = {item.key: item.value for item in model.metadata_props}
    names = meta["joint_names"].split(",")
    count = len(names)
    actor_obs = env["observations"]["actor"]
    history = int(actor_obs["history_length"])
    frame_size = 9 + 3 * count
    input_size = history * frame_size
    assert count == 23 and history == 4 and frame_size == 78
    assert actor_obs["history_ordering"] == "time"
    assert list(actor_obs["terms"]) == meta["observation_names"].split(",")
    assert all(t["clip"] == "null" and t["scale"] == "null"
               for t in actor_obs["terms"].values())
    assert agent["clip_actions"] == "null"
    assert agent["policy"]["activation"] == "elu"
    assert agent["empirical_normalization"] == "true"
    policy_frequency = 1 / (int(env["decimation"]) * float(env["sim"]["mujoco"]["timestep"]))
    assert abs(policy_frequency - 50) < 1e-9

    values = {key: [float(v) for v in meta[key].split(",")]
              for key in ("default_joint_pos", "joint_stiffness", "joint_damping", "action_scale")}
    actuators = env["scene"]["entities"]["robot"]["articulation"]["actuators"]
    by_name = {a["target_names_expr"][0]: a for a in actuators}
    defaults = env["scene"]["entities"]["robot"]["init_state"]["joint_pos"]
    for i, name in enumerate(names):
        default = 0.0
        for pattern, value in defaults.items():
            if re.fullmatch(pattern, name):
                default = float(value)
        assert abs(values["default_joint_pos"][i] - default) < 1e-6
        for key, field in (("joint_stiffness", "stiffness"), ("joint_damping", "damping")):
            assert abs(values[key][i] - float(by_name[name][field])) < 1e-6
        assert abs(values["action_scale"][i] - float(env["actions"]["joint_pos"]["scale"][name])) < 1e-6

    dims = [input_size] + [int(v) for v in agent["policy"]["actor_hidden_dims"]] + [count]
    layers = []
    for i, (n_in, n_out) in enumerate(zip(dims[:-1], dims[1:])):
        layers.append(nn.Linear(n_in, n_out))
        if i < len(dims) - 2:
            layers.append(nn.ELU())
    actor = nn.Sequential(*layers).eval()
    actor.load_state_dict({k.removeprefix("actor."): v
                           for k, v in saved["model_state_dict"].items() if k.startswith("actor.")})
    norm = saved["obs_norm_state_dict"]
    mean, std = norm["_mean"], norm["_std"]

    def infer(obs):
        with torch.inference_mode():
            return actor((torch.as_tensor(obs, dtype=torch.float32) - mean) / (std + 1e-2)).numpy()

    class NormalizedActor(nn.Module):
        def __init__(self):
            super().__init__()
            self.actor = actor
            self.register_buffer("mean", mean)
            self.register_buffer("std", std)

        def forward(self, obs):
            return self.actor((obs - self.mean) / (self.std + 1e-2))

    # The training process overwrites policy.onnx on every save. Export from the
    # pinned checkpoint so its weights cannot race with a newer training save.
    with tempfile.TemporaryDirectory(prefix="hunter180_amp_export_") as directory:
        exported = Path(directory) / "policy.onnx"
        torch.onnx.export(
            NormalizedActor().eval(), torch.zeros(1, input_size), str(exported),
            opset_version=18, input_names=["obs"], output_names=["actions"],
            dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}},
            dynamo=False, external_data=False,
        )
        model = onnx.load(exported)
    onnx.checker.check_model(model)

    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    session = ort.InferenceSession(model.SerializeToString(), options, providers=["CPUExecutionProvider"])
    assert len(session.get_inputs()) == len(session.get_outputs()) == 1
    assert session.get_inputs()[0].shape[1] == input_size
    assert session.get_outputs()[0].shape[1] == count
    samples = np.random.default_rng(int(saved["iter"])).normal(size=(256, input_size)).astype(np.float32)
    samples = samples * std.numpy() + mean.numpy()
    samples[0] = 0
    expected = infer(samples)
    actual = session.run(None, {session.get_inputs()[0].name: samples})[0]
    np.testing.assert_allclose(actual, expected, rtol=3e-5, atol=2e-5)
    assert np.isfinite(actual).all()

    output = ROOT / "src/ec_controller/config/policy/hunter180_amp.onnx"
    extra = {"checkpoint": checkpoint_name, "checkpoint_sha256": sha256(checkpoint),
             "history_length": str(history), "history_ordering": "oldest_to_newest",
             "observation_frame_size": str(frame_size), "normalization": "embedded",
             "policy_frequency": "50"}
    onnx.helper.set_model_props(model, {**meta, **extra})
    onnx.save_model(model, output, save_as_external_data=False)
    manifest = {
        "training_repository": "AMP_mjlab", "run": run_dir.name,
        "checkpoint": checkpoint_name, "iteration": int(saved["iter"]),
        "checkpoint_sha256": extra["checkpoint_sha256"],
        "metadata_source_onnx_sha256": hashlib.sha256(metadata_source).hexdigest(),
        "onnx_sha256": sha256(output), "env_yaml_sha256": sha256(env_path),
        "agent_yaml_sha256": sha256(agent_path), "input_name": session.get_inputs()[0].name,
        "input_shape": [1, input_size], "output_name": session.get_outputs()[0].name,
        "output_shape": [1, count], "observation_names": list(actor_obs["terms"]),
        "history_length": history, "history_ordering": "oldest_to_newest",
        "normalization": "embedded", "normalization_epsilon": 0.01,
        "policy_frequency": policy_frequency, "joint_names": names, **values,
        "training_effort_limits": [float(by_name[n]["effort_limit"]) for n in names],
        "verification": {"samples": len(samples), "max_abs_error": float(np.max(np.abs(expected - actual))),
                         "rtol": 3e-5, "atol": 2e-5},
    }
    output.with_suffix(".json").write_text(json.dumps(manifest, indent=2) + "\n")

    print(json.dumps({"policy": str(output), "checkpoint": checkpoint_name, **manifest["verification"]}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--checkpoint", required=True)
    args = parser.parse_args()
    import_policy(args.run_dir.resolve(), args.checkpoint)
