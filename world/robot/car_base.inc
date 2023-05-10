# 机器人底盘配置
define car_base position 
(
  color "red"                   # 车身颜色
  drive "diff"                  # 车辆运动学模型
  gui_nose 1                    
  obstacle_return 1             
  ranger_return 0.5             
  blob_return 1                  
  fiducial_return 1             

  localization "odom"           # 定位方式
  odom_error [ 0.05 0.05 0.0 0.1 ]  # 里程计误差
  # localization_origin [0 0 0 0]   # 定位原点，默认为机器人的初始位置。

  # [ xmin xmax ymin ymax zmin zmax amin amax ]        
  velocity_bounds [-1 1 0 0 0 0 -45.0 45.0 ]         # 速度最值 
  acceleration_bounds [-0.5 0.5 0 0 0 0 -45 45.0 ]   # 加速度最值

 
  size [0.44 0.38 0.22]  # 车体尺寸
  origin [0 0 0 0] # 旋转中心与车体中心的偏移量
  mass 23.0 # 车体质量，单位时 kg

  gui_nose 0 # 是否绘制方向指示标记


  # 车身形状
  #block( 
  #  points 8
  #  point[0] [-0.2 0.12]
  #  point[1] [-0.2 -0.12]
  #  point[2] [-0.12 -0.2555]
  #  point[3] [0.12 -0.2555]
  #  point[4] [0.2 -0.12]
  #  point[5] [0.2 0.12]
  #  point[6] [0.12 0.2555]
  #  point[7] [-0.12 0.2555]
  #  z [0 0.22]
  #)

  block( 
    points 8
    point[0] [-0.2 0.18]
    point[1] [-0.2 -0.18]
    point[2] [-0.15 -0.27]
    point[3] [0.12 -0.23]
    point[4] [0.2 -0.12]
    point[5] [0.2 0.12]
    point[6] [0.12 0.23]
    point[7] [-0.15 0.27]
    z [0 0.22]
  )
)
