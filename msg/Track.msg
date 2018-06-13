uint8 STATUS_UNKNOWN=0
uint8 STATUS_INITED=1
uint8 STATUS_LOST=2
uint8 STATUS_TRACKING=3

uint32 uid
uint32 class_id
uint8 status
float32 confidence

float32 x
float32 y
float32 width
float32 height

sensor_msgs/RegionOfInterest roi
