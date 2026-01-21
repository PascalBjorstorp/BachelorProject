^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vesc_ackermann
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2024-XX-XX)
------------------
* Added IMU integration support for angular velocity and yaw (vesc_to_odom)
* Added configurable integration methods: euler, trapezoidal, analytical (vesc_to_odom)
* Added low-pass filter for IMU angular velocity with configurable alpha (vesc_to_odom)
* Added abnormal dt detection to prevent odometry jumps (vesc_to_odom)
* Added configurable odometry covariance parameters:
  - odom_x_covariance (default: 0.2)
  - odom_y_covariance (default: 0.2)
  - odom_yaw_covariance (default: 0.4)
* Added braking control system with sigmoid-based regenerative braking (ackermann_to_vesc)
* Added acceleration control modes: ACCEL_TO_CURRENT, VEL_TO_CURRENT, VEL_TO_ERPM (ackermann_to_vesc)
* Added odometry feedback for velocity-based control (ackermann_to_vesc)
* New parameters: use_imu, integration_method, imu_angular_velocity_alpha
* New parameters: operation_mode, enable_braking, brake_current_min/max, brake_speed_threshold
* New parameters: brake_sigmoid_steepness, acceleration_to_current_gain
* Added sensor_msgs, tf2, tf2_geometry_msgs dependencies
* Contributors: HYU, Ray-Quasar

1.1.0 (2020-12-12)
------------------
* Merge pull request `#1 <https://github.com/f1tenth/vesc/issues/1>`_ from f1tenth/melodic-devel
  Updating for Melodic
* Remove unused Boost dependency.
* Replacing boost::shared_ptr with Standard Library equivalent.
* Fixing roslint error.
* Updating package.xml to format 3 and setting C++ standard to 11.
* Contributors: Joshua Whitley

1.0.0 (2020-12-02)
------------------
* Applying roslint to vesc_ackerman.
* Adding roslint.
* Adding licenses.
* Updating maintainers, authors, and URLs.
* added onboard car
* Contributors: Joshua Whitley, billyzheng
