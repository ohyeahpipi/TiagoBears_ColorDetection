# TiagoBears_ColorDetection

Change only in src/TiagoBears_ColorDetection/color_detector.py

To use bag file, change the image topic

```
rosbag play -l 2023-03-06-15-34-51.bag /xtion/rgb/image_raw:=/bag/image_raw
```

```
rosrun TiagoBears_ColorDetection color_detection_node.py
```