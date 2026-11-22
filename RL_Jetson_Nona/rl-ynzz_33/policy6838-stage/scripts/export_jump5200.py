"""Export the archived Jump 5200 actor and check 256 fixed reference inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import onnx
import onnxruntime as ort
import torch
from torch import nn

class Policy(nn.Module):
    def __init__(self, state):
        super().__init__()
        def layers(prefix, count):
            result = []
            for index in range(count):
                weight = state[f"{prefix}.{2 * index}.weight"]
                result.append(nn.Linear(weight.shape[1], weight.shape[0]))
                if index < count - 1:
                    result.append(nn.ELU())
            return nn.Sequential(*result)
        self.encoder = layers("encoder", 3)
        self.actor = layers("actor", 4)
        self.load_state_dict({k: v for k, v in state.items()
                              if k.startswith(("encoder.", "actor."))}, strict=True)

    def forward(self, obs, obs_history):
        return self.actor(torch.cat((obs, self.encoder(obs_history)), dim=-1))

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint")
    parser.add_argument("output")
    args = parser.parse_args()
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    if checkpoint["contract"] != "fudan-jump-physical25-history125-v1":
        raise ValueError("Wrong Jump contract")
    if sha(args.checkpoint) != "1f1ae7d029c6f741a84b6b703d2ce496a4787c4d29c0934deeb9582c2aa17f80":
        raise ValueError("Wrong Jump 5200 checkpoint")
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    (output / ".gitattributes").write_text("* -text\n", encoding="utf-8", newline="\n")
    network = Policy(checkpoint["model_state_dict"]).eval()
    torch.manual_seed(5200)
    obs, history = torch.randn(256, 25), torch.randn(256, 125)
    path = output / "policy.onnx"
    torch.onnx.export(network, (obs[:1], history[:1]), path, dynamo=False,
                      opset_version=17, input_names=["obs", "obs_history"],
                      output_names=["actions"])
    onnx.checker.check_model(onnx.load(path))
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    with torch.no_grad():
        expected = network(obs, history).numpy()
    actual = np.concatenate([session.run(None, {"obs": obs[i:i+1].numpy(),
                              "obs_history": history[i:i+1].numpy()})[0] for i in range(256)])
    error = float(np.max(np.abs(expected - actual)))
    if not np.allclose(expected, actual, atol=2e-5, rtol=1e-5):
        raise ValueError(f"Export mismatch: {error}")
    np.concatenate((obs.numpy(), history.numpy()), axis=1).astype("<f4").tofile(output / "reference-inputs.bin")
    expected.astype("<f4").tofile(output / "reference-actions.bin")
    np.savez(output / "reference-vectors-256.npz", obs=obs.numpy(),
             obs_history=history.numpy(), actions=expected)
    metadata = dict(contract=checkpoint["contract"], checkpoint_sha256=sha(args.checkpoint),
                    onnx_sha256=sha(path), input={"obs": [1,25], "obs_history": [1,125]},
                    output={"actions": [1,6]}, command_scale=checkpoint["command_scale"],
                    jump_config=checkpoint["jump_config"], control=checkpoint["control"],
                    control_timing=checkpoint["control_timing"], reset_contract=checkpoint["reset_contract"],
                    runtime_height_command_m=1.0, max_abs_error=error, atol=2e-5, rtol=1e-5,
                    validation="Actor/encoder weights strict loaded; ELU matches archived base network. Jump task source remains unavailable.")
    (output / "interface.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8", newline="\n")
    (output / "SHA256SUMS").write_text("".join(f"{sha(p)}  {p.name}\n" for p in sorted(output.iterdir())), encoding="utf-8", newline="\n")
    print(json.dumps({"output":str(output), "max_abs_error":error}))
if __name__ == "__main__":
    main()
