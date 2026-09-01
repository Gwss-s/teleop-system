"""LeRobot-native policy server: serve any LeRobot policy over the
PolicyServer protocol.  Swapping backbones = swapping --ckpt.

Verified with pi0.5 (predict_action_chunk).  pi0 / SmolVLA share the chunk
API; ACT / Diffusion may need select_action-loop fallback (TODO below).

Camera key mapping is a CLI argument:
  --image-keys "base_0_rgb=head,right_wrist_0_rgb=wrist_right"
maps the checkpoint's image feature short-names to the client's image names.
Missing client cameras are simply omitted -> the policy's own missing-key
path (zero image + mask) applies, matching training-time behaviour.

Run:  python -m teleop_system.policy.server_lerobot --ckpt <pretrained_model_dir> \
        --port 5557 --image-keys "base_0_rgb=head,right_wrist_0_rgb=wrist_right"
"""
import argparse
import socket

import numpy as np
import torch

from .protocol import recv_msg, send_msg

TASK_DEFAULT = ""


def load_policy(ckpt, device):
    """Load a LeRobot policy from a pretrained_model dir, inferring its class
    from config.json (falls back to pi05)."""
    import json
    from pathlib import Path
    cfg = json.loads((Path(ckpt) / "config.json").read_text())
    ptype = cfg.get("type", "pi05")
    if ptype in ("pi05", "pi0.5"):
        from lerobot.policies.pi05 import PI05Policy as Cls
    elif ptype == "pi0":
        from lerobot.policies.pi0 import PI0Policy as Cls
    elif ptype == "smolvla":
        from lerobot.policies.smolvla import SmolVLAPolicy as Cls
    elif ptype == "act":
        from lerobot.policies.act import ACTPolicy as Cls
    elif ptype == "diffusion":
        from lerobot.policies.diffusion import DiffusionPolicy as Cls
    else:
        raise ValueError(f"unsupported policy type in config.json: {ptype}")
    policy = Cls.from_pretrained(ckpt)
    policy.to(device)
    policy.eval()
    return policy, ptype


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", required=True, help="pretrained_model directory")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--img-size", type=int, default=224)
    ap.add_argument("--image-keys", default="",
                    help="'ckpt_short=client_name,...' e.g. base_0_rgb=head,right_wrist_0_rgb=wrist_right; "
                         "empty = identity mapping")
    ap.add_argument("--task", default=TASK_DEFAULT, help="default task string if client omits it")
    args = ap.parse_args()

    from lerobot.policies import make_pre_post_processors
    from lerobot.policies.common.vla_utils import resize_with_pad_torch

    print(f"[policy-server] loading {args.ckpt}", flush=True)
    policy, ptype = load_policy(args.ckpt, args.device)
    pre, post = make_pre_post_processors(
        policy_cfg=policy.config,
        pretrained_path=args.ckpt,
        preprocessor_overrides={"device_processor": {"device": args.device}},
    )
    img_keys = list(policy.config.image_features.keys())
    rev = dict(kv.split("=") for kv in args.image_keys.split(",") if "=" in kv)
    S = args.img_size
    print(f"[policy-server] type={ptype} image_features={img_keys} map={rev} "
          f"chunk={getattr(policy.config, 'chunk_size', '?')}", flush=True)

    @torch.no_grad()
    def infer(state_np, images_np, task):
        obs = {"observation.state": torch.from_numpy(np.asarray(state_np, np.float32)).unsqueeze(0)}
        for k in img_keys:
            short = k.split(".")[-1]
            client_key = rev.get(short, short)
            if client_key not in images_np:
                continue  # missing camera -> policy's own zero-image+mask path
            hwc = torch.from_numpy(images_np[client_key]).float().div(255.0)
            chw = hwc.permute(2, 0, 1)
            r = resize_with_pad_torch(chw, S, S).squeeze(0).clamp(0, 1)
            obs[k] = r.unsqueeze(0)
        obs["task"] = [task]
        obs = pre(obs)
        if hasattr(policy, "predict_action_chunk"):
            chunk = policy.predict_action_chunk(obs)
        else:  # TODO: ACT/Diffusion single-step loop semantics
            chunk = policy.select_action(obs).unsqueeze(1)
        chunk = post(chunk)
        return chunk.squeeze(0).float().cpu().numpy()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(4)
    print(f"[policy-server] ready on {args.host}:{args.port}", flush=True)
    while True:
        conn, addr = srv.accept()
        print(f"[policy-server] client {addr}", flush=True)
        try:
            n = 0
            while True:
                req = recv_msg(conn)
                if req is None:
                    break
                chunk = infer(req["state"], req["images"], req.get("task") or args.task)
                send_msg(conn, chunk)
                n += 1
                if n % 20 == 1:
                    print(f"[policy-server] inference #{n} -> {chunk.shape}", flush=True)
        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            conn.close()
            print("[policy-server] client disconnected", flush=True)


if __name__ == "__main__":
    main()
