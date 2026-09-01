"""LingBot-VLA-V2 → PolicyServer 协议桥(独立进程,third_party 三铁律之进程隔离)。

模型代码 100% 留在官方仓库(sys.path 注入 + pixi 专属环境 torch2.8/flash-attn);
本文件是"薄壳适配器":只做观测翻译 + 出 chunk,复用官方 deploy.LingbotVLAv2Server
的加载/预处理/反归一化全链路。

观测契约(与官方 robotwin 客户端逐键一致,LIBERO 映射按 gq
train/libero/lingbot_robot_configs/libero.yaml):
  我们的协议 {"state"[8], "images"{"agentview","wrist"}, "task"}
    → org 键 {"observation.state", "observation.images.image",
              "observation.images.image2", "task"}
    → infer() → {"action": (chunk=50, 7)}  # 已反归一化的 LIBERO OSC 增量
官方 reset() 写死相对路径 configs/robot_configs/<name>.yaml,本桥用绝对路径直构
FeatureTransform,故无需在官方仓库目录下运行。

用法(lingbot pixi 环境的 python):
  python -m teleop_system.policy.server_lingbot --ckpt <hf_ckpt> \
    --robot-config <libero.yaml> --norm-stats <libero.json> \
    --lingbot-repo <官方仓库> --port 5580
"""
import argparse
import os
import socket
import sys
import threading

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", required=True, help="hf_ckpt 目录(其上三级须有 lingbotvla_cli.yaml)")
    ap.add_argument("--robot-config", required=True, help="本体映射 yaml 绝对路径")
    ap.add_argument("--norm-stats", required=True, help="归一化统计 json 绝对路径")
    ap.add_argument("--lingbot-repo", required=True, help="官方 lingbot-vla-v2 仓库路径")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5580)
    ap.add_argument("--use-compile", type=int, default=1,
                help="官方 main 默认开启 torch.compile;0 仅用于排障")
    ap.add_argument("--max-clients", type=int, default=1,
                help=">1 = 多客户端并发伺服:每连接一线程,infer 加锁串行,"
                     "响应与单客户端逐比特一致(模型无内部状态,0.3s/chunk 时 GPU 占空比<5%)")
    args = ap.parse_args()

    sys.path.insert(0, args.lingbot_repo)
    from deploy.lingbot_vla_v2_policy import LingbotVLAv2Server            # noqa: E402
    from lingbotvla.data.vla_data.utils import FeatureTransform            # noqa: E402

    from teleop_system.policy.protocol import recv_msg, send_msg        # noqa: E402

    model = LingbotVLAv2Server(
        path_to_pi_model=args.ckpt,
        robot_norm_path=args.norm_stats,
        chunk_ret=True,                    # 每次查询都前向,返回整条 chunk(无内部状态依赖)
        use_length=50,                     # 官方 main 同款:不传则默认 1,chunk 会被截成 1 帧!
        use_bf16=True,
        use_compile=bool(args.use_compile),
    )
    assert int(model.config.chunk_size) == 50, f"chunk_size={model.config.chunk_size}≠50,use_length 需同步"
    ft = FeatureTransform(args.robot_config, model.data_config, model.config,
                          model.processor, chunk_size=model.config.chunk_size,
                          norm_stats_path=args.norm_stats)
    model.vla.feature_transform = ft
    model.action_key = ft.org_features["actions"]
    print(f"[lingbot-server] org_features={ft.org_features} chunk={model.config.chunk_size}",
          flush=True)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(max(4, args.max_clients * 2))
    print(f"[lingbot-server] ready on {args.host}:{args.port} max_clients={args.max_clients}",
          flush=True)

    infer_lock = threading.Lock()   # 串行化 infer:共享 FeatureTransform/模型,保证与单客户端逐比特一致
    n_total = [0]

    def serve_conn(conn, addr):
        try:
            while True:
                req = recv_msg(conn)
                if req is None:
                    break
                obs = {
                    "observation.state": np.asarray(req["state"], np.float32),
                    "observation.images.image": np.asarray(req["images"]["agentview"], np.uint8),
                    "observation.images.image2": np.asarray(req["images"]["wrist"], np.uint8),
                    "task": req.get("task") or "",
                }
                with infer_lock:
                    out = model.infer(obs)
                    n_total[0] += 1
                    n = n_total[0]
                chunk = np.asarray(out["action"], np.float32)
                send_msg(conn, chunk)
                if n % 20 == 1:
                    print(f"[lingbot-server] inference #{n} -> {chunk.shape}", flush=True)
        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            conn.close()
            print(f"[lingbot-server] client {addr} disconnected", flush=True)

    if args.max_clients <= 1:
        while True:                          # 单客户端串行 accept(原行为,逐字保留)
            conn, addr = srv.accept()
            print(f"[lingbot-server] client {addr}", flush=True)
            serve_conn(conn, addr)
    else:
        while True:                          # 多客户端:每连接一线程,infer 全局锁
            conn, addr = srv.accept()
            print(f"[lingbot-server] client {addr}", flush=True)
            threading.Thread(target=serve_conn, args=(conn, addr), daemon=True).start()


if __name__ == "__main__":
    main()
