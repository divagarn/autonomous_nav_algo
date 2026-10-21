"""Class names -> Autoware classification labels, sizes and display colours.

Names come from the 18-class IDD road-object detector (camera_processing/yolox_detector_node)
and from radar_fusion ("unknown" until a camera votes a class).
"""
from autoware_perception_msgs.msg import ObjectClassification as OC

UNKNOWN = 'unknown'

# detector class name -> Autoware label
LABELS = {
    'car': OC.CAR, 'autorickshaw': OC.CAR, 'vehicle fallback': OC.CAR,
    'truck': OC.TRUCK, 'bus': OC.BUS,
    'motorcycle': OC.MOTORCYCLE, 'motorcyclist': OC.MOTORCYCLE,
    'bicycle': OC.BICYCLE, 'bicyclist': OC.BICYCLE,
    'pedestrian': OC.PEDESTRIAN, 'person': OC.PEDESTRIAN, 'partial pedestrian': OC.PEDESTRIAN,
    'animal': OC.ANIMAL,
    'barricade': OC.UNKNOWN, 'debris': OC.UNKNOWN, 'pothole': OC.UNKNOWN,
    'road damage': OC.UNKNOWN, 'traffic light': OC.UNKNOWN, 'traffic sign': OC.UNKNOWN,
}
LABEL_NAMES = {OC.CAR: 'car', OC.TRUCK: 'truck', OC.BUS: 'bus', OC.MOTORCYCLE: 'motorcycle',
               OC.BICYCLE: 'bicycle', OC.PEDESTRIAN: 'pedestrian', OC.ANIMAL: 'animal',
               OC.UNKNOWN: UNKNOWN}

# default (length, width, height) when a sensor gives no usable size
SIZES = {OC.CAR: (4.5, 1.8, 1.6), OC.TRUCK: (8.0, 2.5, 3.0), OC.BUS: (10.5, 2.5, 3.2),
         OC.MOTORCYCLE: (2.0, 0.8, 1.5), OC.BICYCLE: (1.8, 0.6, 1.5),
         OC.PEDESTRIAN: (0.6, 0.6, 1.7), OC.ANIMAL: (1.0, 0.5, 0.8), OC.UNKNOWN: (1.0, 1.0, 1.0)}

COLORS = {OC.CAR: (0.2, 0.5, 1.0), OC.TRUCK: (0.2, 0.8, 0.9), OC.BUS: (1.0, 0.7, 0.0),
          OC.MOTORCYCLE: (0.7, 0.4, 1.0), OC.BICYCLE: (0.4, 0.9, 0.4),
          OC.PEDESTRIAN: (1.0, 0.2, 0.6), OC.ANIMAL: (0.6, 0.4, 0.2), OC.UNKNOWN: (0.9, 0.1, 0.1)}


def label_of(name):
    return LABELS.get(str(name).strip().lower(), OC.UNKNOWN)
