# Selection3D.msg

# The points of the selection region polygon 2D screen coords 
geometry_msgs/Point[] points

# The points of the selection region in 3D space
geometry_msgs/Point[] points_3d

# True if the tool was used to select, false if used to deselect
bool is_selecting

# Camera data at time of selection
rviz_selection_3d/CameraData camera

# viweport width
int32 viewport_width

# viweport height
int32 viewport_height