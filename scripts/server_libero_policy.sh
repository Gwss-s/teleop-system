#!/bin/bash
# 在 GPU 服务器(172.26.0.50)上启动 LIBERO 冻结基座服务(pi05_libero_finetuned_v044)
# 用法: bash scripts/server_libero_policy.sh [GPU_ID] [PORT]
# 注意: 不用 pi05_libero_base —— 该仓库缺归一化统计文件,评测恒 0%(lerobot#2533)
# 接口依据 ckpt config.json: image/image2 256x256, state[8], action[7], chunk 50
set -e
GPU=${1:-1}
PORT=${2:-5557}
# 以下均可用环境变量覆盖(默认值 = gws 车道的原路径,他的用法不受影响)
PIENV_PY="${PIENV_PY:-/workspace/yangtong/gq/WorkSpace/EBiM/frameworks/envs/pi_env/.pixi/envs/default/bin/python}"
export HF_HOME="${TRAIN_HF_HOME:-/workspace/yangtong/gws/hf_cache}"
SNAP="${SNAP:-/workspace/yangtong/gws/models/pi05_libero_finetuned}"
REPO="${REPO:-/workspace/yangtong/gws/teleop-system}"

echo "[launch] ckpt=$SNAP gpu=$GPU port=$PORT"
cd $REPO
CUDA_VISIBLE_DEVICES=$GPU PYTHONPATH=$REPO $PIENV_PY -m teleop_system.policy.server_lerobot \
  --ckpt "$SNAP" --port $PORT --img-size 256 \
  --image-keys "image=agentview,image2=wrist" \
  --task "default"
