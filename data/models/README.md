# Camera models

| File | Use | Source |
|---|---|---|
| `yolox_s_18class.onnx` | object detection (default) | `OD_FP32/YOLOX_outputs/yolox_s_ti_18class_twillight_clean/yolox_s_ti_18class_twillight_clean_tda4vh.onnx` |
| `lane_seg_up.onnx` | lane / road segmentation (default) | `unified_pipeline_onnx_package_20261021/weights/up.onnx` |
| `yolox_s_ti_18class_ti_today_qat.onnx` | lead's YOLOX (QAT, TI) | was already here |
| `road_bisenetv2_ade20k.onnx` | road-only segmentation, open source (option: `lane_model:=$PWD/data/models/road_bisenetv2_ade20k.onnx`) | BiSeNetV2 ADE20K, exported by `export_road_bisenetv2.py` |

The files are byte-identical copies of the sources (sha256 checked when copied, 2026-10-22).

./buggy live / replay pick `yolox_s_18class` and `lane_seg_up` from here (`_fp16.engine` on the Orin, else `_fp32.engine`;
`.onnx` on the PC); another model: `detector_model:=<path>` / `lane_model:=<path>`.

Orin TensorRT engines (the `.engine` files here were built on the buggy's Orin; rebuild after a JetPack /
TensorRT update or for another Orin; an engine only runs on the GPU and TensorRT version it was built with):

    /usr/src/tensorrt/bin/trtexec --onnx=data/models/yolox_s_18class.onnx --saveEngine=data/models/yolox_s_18class_fp16.engine --fp16
    /usr/src/tensorrt/bin/trtexec --onnx=data/models/lane_seg_up.onnx --saveEngine=data/models/lane_seg_up_fp16.engine --fp16
    /usr/src/tensorrt/bin/trtexec --onnx=data/models/road_bisenetv2_ade20k.onnx --saveEngine=data/models/road_bisenetv2_ade20k_fp16.engine --fp16

(FP32: leave out `--fp16`, add `--noTF32`; about 2x slower.)

## yolox_s_18class.onnx

- Input `images` [1, 3, 640, 640] float32: BGR, 0..255 (no normalisation), letterbox
  at the **top-left** (resize by min(640 / h, 640 / w), pad right / bottom with 114).
- Output `output` [1, 8400, 23]: cx, cy, w, h (grid units), objectness, 18 class scores
  (sigmoid already applied). Decode per stride 8 / 16 / 32: xy = (xy + grid) * stride,
  wh = exp(wh) * stride; score = objectness * class score; then NMS, and divide by the
  letterbox scale.
- Classes (index 0..17): animal, autorickshaw, barricade, bicycle, bicyclist, bus, car,
  debris, motorcycle, motorcyclist, partial pedestrian, pedestrian, pothole, road damage,
  traffic light, traffic sign, truck, vehicle fallback.
- Reference settings: confidence 0.30, NMS 0.70 class-agnostic (video script);
  TIDL prototxt: NMS 0.20, confidence 0.01.

## lane_seg_up.onnx

- Input `input.1` [1, 3, 512, 768] float32: RGB, (pixel - 127) / 127, plain resize to
  768 x 512 (no letterbox).
- Outputs: `651` [1, 4, 128, 192], `657` [1, 2, 128, 192], `663` [1, 2, 128, 192] = a
  CenterNet detection head (lmv, person, two_wheeler; not used, detection is YOLOX);
  `746` [1, 1, 512, 768] int64 = class per pixel: 0 background, 1 lane, 2 road
  (TensorRT turns it into int32).

## road_bisenetv2_ade20k.onnx (open source, road only)

- Model: BiSeNetV2 trained on ADE20K, https://github.com/CoinCheung/BiSeNet (MIT licence,
  `LICENSE-BiSeNet`; weights `model_final_v2_ade20k.pth` of its release 0.0.0). ADE20K images have
  their own terms of use: fine for testing; check before shipping in a product.
- Same interface as `lane_seg_up.onnx`: input `input` [1, 3, 512, 768] RGB (pixel - 127) / 127;
  output `class_map` [1, 1, 512, 768] int32: 0 background, 2 road. No lane class (1).
- Road = the ADE20K ground classes floor, road, sidewalk, earth, path, runway, dirt track; the
  ADE20K normalisation and this merge are inside the exported graph (`export_road_bisenetv2.py`).
- Why this one: on bag_record_3 (paved campus, no markings) it covers the paving like lane_seg_up;
  YOLOP and TwinLiteNet (BDD100K drivable area, MIT) and BiSeNetV2 Cityscapes missed most of it.
  lane_seg_up stays the default: cleaner borders, and the hood is not labelled as ground.
- ONNX Runtime CPU ~300 ms per image (PC), so on the PC use precomputed masks or run the lane on the Orin.
