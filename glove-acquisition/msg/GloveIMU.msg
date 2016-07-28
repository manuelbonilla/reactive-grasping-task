# One sample data of each IMU

# Please notice that has not being used 'geometry_msgs/AccelStamped.msg' because of the angular velocity furnished
# by the IMU instead of the angular acceleration which it is proper of that type of message. The objective is to be
# as clear as possible.

# IMU identifier
int32 id

# 3D accelerometer data
geometry_msgs/Vector3 linear_acceleration
# 3D gyroscope data
geometry_msgs/Vector3 angular_velocity
