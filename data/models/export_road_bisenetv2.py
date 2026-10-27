#!/usr/bin/env python3
"""Exports BiSeNetV2 (ADE20K, MIT licence, github.com/CoinCheung/BiSeNet) as a road-only segmentation
model with the I/O of lane_seg_up.onnx, so object_detection uses it unchanged:

    input  [1, 3, 512, 768] float32  RGB, (pixel - 127) / 127, plain resize
    output [1, 1, 512, 768] int32    0 background, 2 road (ground classes of ADE20K); no lane class (1)

The ADE20K normalisation and the class merge are part of the exported graph.

    git clone --depth 1 https://github.com/CoinCheung/BiSeNet.git
    curl -L -o model_final_v2_ade20k.pth \
      https://github.com/CoinCheung/BiSeNet/releases/download/0.0.0/model_final_v2_ade20k.pth
    python3 models/export_road_bisenetv2.py --repo BiSeNet --weights model_final_v2_ade20k.pth \
      --out models/road_bisenetv2_ade20k.onnx
"""
import argparse
import sys

import torch

# ADE20K classes (0-based, = SceneParsing label - 1) that are drivable ground for the buggy
GROUND = {3: 'floor', 6: 'road', 11: 'sidewalk', 13: 'earth', 52: 'path', 54: 'runway', 91: 'dirt track'}
MEAN = [0.49343230, 0.46819794, 0.43106043]   # configs/bisenetv2_ade20k.py (RGB, 0..1)
STD = [0.25680755, 0.25506608, 0.27422913]


class RoadOnly(torch.nn.Module):
    """lane_seg_up input -> ADE20K normalisation -> BiSeNetV2 -> argmax -> 2 where ground, else 0."""

    def __init__(self, net):
        super().__init__()
        self.net = net
        self.register_buffer('mean', torch.tensor(MEAN).view(1, 3, 1, 1) * 255)
        self.register_buffer('std', torch.tensor(STD).view(1, 3, 1, 1) * 255)

    def forward(self, x):
        pixel = x * 127 + 127                       # back to RGB 0..255
        logits = self.net((pixel - self.mean) / self.std)[0]
        label = logits.argmax(dim=1, keepdim=True)
        ground = torch.zeros_like(label, dtype=torch.bool)
        for c in GROUND:
            ground = ground | (label == c)
        return ground.to(torch.int32) * 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo', required=True, help='BiSeNet repository checkout')
    ap.add_argument('--weights', required=True, help='model_final_v2_ade20k.pth')
    ap.add_argument('--out', default='models/road_bisenetv2_ade20k.onnx')
    ap.add_argument('--width', type=int, default=768)
    ap.add_argument('--height', type=int, default=512)
    a = ap.parse_args()
    sys.path.insert(0, a.repo)
    from lib.models.bisenetv2 import BiSeNetV2
    net = BiSeNetV2(150, aux_mode='eval')
    state = torch.load(a.weights, map_location='cpu', weights_only=True)
    missing = net.load_state_dict(state, strict=False).missing_keys
    if missing:
        sys.exit(f'weights do not match the model: missing {missing[:5]}')
    model = RoadOnly(net).eval()
    with torch.no_grad():
        torch.onnx.export(model, torch.zeros(1, 3, a.height, a.width), a.out, input_names=['input'],
                          output_names=['class_map'], opset_version=13, dynamo=False)
    print(f'wrote {a.out}: ground classes {sorted(GROUND.values())}')


if __name__ == '__main__':
    main()
