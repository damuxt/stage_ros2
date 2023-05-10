# 车辆底盘与传感器集成

include "robot/car_base.inc"
include "robot/laser.inc"
include "robot/camera.inc"

# 底盘集成单个雷达
define car_base_with_laser car_base
(
    my_laser()
)
# 底盘集成单个摄像头
define car_base_with_camera car_base
(
    my_camera()
)
# 底盘集成两个雷达
define car_base_with_two_laser car_base
(
    my_laser(pose [0.15 0 0 0])
    my_laser(pose [-0.15 0 0 -180])
)
# 底盘集成一个雷达，一个摄像头
define car_base_with_laser_camera car_base
(
    my_laser()
    my_camera(pose [0.15 0 0 0])
)
