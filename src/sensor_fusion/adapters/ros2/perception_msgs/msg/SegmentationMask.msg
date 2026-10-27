# Per-pixel classes of one camera image (lane / road segmentation of object_detection).
std_msgs/Header header       # stamp = image stamp, frame_id = camera frame
string source                # segmentation backend: "tensorrt" | "onnxruntime" | "replay"
uint32 image_width           # image the mask belongs to (rectified like the detection boxes)
uint32 image_height
string[] class_names         # index = mask value: background, lane, road
sensor_msgs/Image mask       # mono8, class per pixel; any size (pixel (u, v) of the image =
                             # mask pixel (u * mask.width / image_width, v * mask.height / image_height))
