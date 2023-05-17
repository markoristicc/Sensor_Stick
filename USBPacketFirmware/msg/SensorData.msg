#ok lets think about what we need
# a lot of these will have unitinitialized covariance
sensor_msgs/Imu.msg #IMU
sensor_msgs/MagneticField.msg #magnetometer
rosflight_msgs/Barometer.msg #altitude, pressure, and temperature
rosflight_msgs/Airspeed #velocity & DP
sensor_msgs/NavSatFix #GPS
