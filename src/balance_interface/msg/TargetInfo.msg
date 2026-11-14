# Whether the target has been detected
bool detected

# Normalized center coordinates of the target in the image (-1.0 to 1.0) 1.0)
# x < 0 x < 0 means left, x > 0 means right, and 0 means exactly centered
float32 error_x
float32 error_y

# Target bounding-box information (in pixels, for debugging))
int32 center_x
int32 center_y
int32 width
int32 height

# Matched cosine similarity (0.0 to 1.0; higher means more similar))
float32 similarity

# Processing latency for one frame (seconds))
float32 delay_time