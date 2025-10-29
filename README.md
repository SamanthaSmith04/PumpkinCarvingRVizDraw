# PumpkinCarvingRVizDraw

A set of RViz plugins for drawing paths in a 3D environment and publishing the resulting pose arrays.

![Screencastfrom10-29-2025042431PM-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/715677d3-9e22-4bbb-b425-1ade1e974dd2)

These pose arrays can be exported as a YAML file, which was loaded into a different [application](https://github.com/SamanthaSmith04/pumpkin_carving/tree/f13442de0f7fb0ec442f48214bfd000b8b1f774f) that generated a motion plan from the paths.

<img width=300px src="https://github.com/user-attachments/assets/3982c343-809a-475f-9315-4070a367e572"/>


https://github.com/user-attachments/assets/a7806739-ed16-499e-a7a7-a64d404b7a38

# Building + Running
```
colcon build

source install/setup.bash

rviz2
```

Load in the rviz config file and publish a static tf from `world` to `pumpkin_face`

`ros2 run tf2_ros static_transform_publisher x y z x y z w world pumpkin_face`
