uint8 class_id
string class_name
float32 confidence
float32 pixel_x
float32 pixel_y
float32 circle_radius_px
float32 circle_fit_score
# Unit viewing ray in TargetDetectionArray.header.frame_id at the image stamp.
geometry_msgs/Vector3 camera_ray
# Deprecated compatibility field; the mission node computes map position.
geometry_msgs/Point ground_position
string localization_method
