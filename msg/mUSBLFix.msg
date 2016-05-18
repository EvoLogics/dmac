# The USBL fix message
# Valid range only
int32 RANGE_ONLY = 0
# Valid azimuth
int32 AZIMUTH_ONLY = 1
# Valid position (usually assumes valid range and azimuth)
int32 FULL_FIX = 2
# Relative Position (usually assumes valid range and azimuth)
int32 CARTESIAN = 3

# Header information
Header header

# Beacon ID and Name
int32 source_id
string source_name

# Fix type
int32 type

# Relative position in inertial
geometry_msgs/Point relative_position

# Range to target
float32 range
# Bearing to target
float32 bearing
# Elevation to target
float32 elevation
# Sound speed use for calculation
float32 sound_speed

# Raw angles
# Bearing to target
float32 bearing_raw
# Elevation to target
float32 elevation_raw 

