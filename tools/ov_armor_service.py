#!/usr/bin/env python3
# OpenVINO YOLO armor detector service for autoaim002.
#
# Reads 1440x1080 BGR raw bytes from stdin (each frame prefixed by a 4-byte
# little-endian length), runs the yolo11 armor detector via OpenVINO, and
# writes a JSON line to stdout with the detected armors.
#
# Protocol (binary on stdin, lines on stdout):
#   in : <uint32 little-endian length><length bytes of raw BGR frame>
#   out: <json line: {"armors":[{"x":..,"y":..,"w":..,"h":..,"score":..,"cls":..},...]}>
#
# Model: tongji yolo11.xml (1x3x640x640, output 1x50x8400).

import sys
import os
import struct
import json

import numpy as np

import openvino as ov

MODEL = os.environ.get("AUTO_AIM_NN_MODEL", "/home/xqy/桌面/tongji/assets/yolo11.xml")
SCORE_THRESHOLD = float(os.environ.get("AUTO_AIM_NN_SCORE", "0.5"))
NMS_THRESHOLD = float(os.environ.get("AUTO_AIM_NN_NMS", "0.3"))
CLASS_NUM = int(os.environ.get("AUTO_AIM_NN_CLASSES", "38"))
INPUT = 640

core = ov.Core()
compiled = core.compile_model(core.read_model(MODEL), "CPU")


def nms(dets):
    dets.sort(key=lambda d: -d["score"])
    keep = []
    for d in dets:
        ok = True
        for k in keep:
            xa = max(d["x"], k["x"])
            ya = max(d["y"], k["y"])
            xb = min(d["x"] + d["w"], k["x"] + k["w"])
            yb = min(d["y"] + d["h"], k["y"] + k["h"])
            inter = max(0.0, xb - xa) * max(0.0, yb - ya)
            union = d["w"] * d["h"] + k["w"] * k["h"] - inter
            if union > 0 and inter / union > NMS_THRESHOLD:
                ok = False
                break
        if ok:
            keep.append(d)
    return keep


def detect(bgr_bytes):
    img = np.frombuffer(bgr_bytes, dtype=np.uint8)
    img = img.reshape((1080, 1440, 3))  # BGR (OpenCV frame)
    rgb = img[:, :, ::-1]  # BGR -> RGB
    H, W, _ = rgb.shape
    s = min(INPUT / H, INPUT / W)
    h, w = int(H * s), int(W * s)
    pad = np.full((INPUT, INPUT, 3), 114, np.uint8)
    # Use cv2 if available, else pure-numpy resize via PIL fallback.
    try:
        import cv2
        cv2.resize(rgb, (w, h), dst=pad[:h, :w])
    except Exception:
        from PIL import Image
        pad[:h, :w] = np.array(Image.fromarray(rgb).resize((w, h)))

    x = pad.astype(np.float32) / 255.0
    x = x.transpose(2, 0, 1)[None].copy()
    out = list(compiled(x).values())[0][0]  # (50, 8400)
    scores = out[4 : 4 + CLASS_NUM]

    dets = []
    for a in range(scores.shape[1]):
        cls = int(np.argmax(scores[:, a]))
        sc = float(scores[cls, a])
        if sc < SCORE_THRESHOLD:
            continue
        x1, y1, w1, h1 = (float(out[c, a]) for c in range(4))
        dets.append({"x": x1, "y": y1, "w": w1, "h": h1, "score": sc, "cls": cls})

    return nms(dets)


def main():
    stdin = sys.stdin.buffer
    stdout = sys.stdout.buffer
    while True:
        header = stdin.read(4)
        if not header or len(header) < 4:
            break
        (length,) = struct.unpack("<I", header)
        if length <= 0 or length > 1440 * 1080 * 3:
            break
        data = stdin.read(length)
        if len(data) < length:
            break
        try:
            dets = detect(data)
            line = json.dumps(
                {"armors": [{"x": d["x"], "y": d["y"], "w": d["w"],
                             "h": d["h"], "score": d["score"], "cls": d["cls"]}
                            for d in dets]}
            ).encode()
            stdout.write(line + b"\n")
            stdout.flush()
        except Exception as e:
            stdout.write((json.dumps({"error": str(e)}) + "\n").encode())
            stdout.flush()


if __name__ == "__main__":
    main()
