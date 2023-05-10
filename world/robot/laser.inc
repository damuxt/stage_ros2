define my_laser ranger
(
  sensor(  
    pose [ 0 0 0 0 ]
    size [ 0.07 0.07 0.05 ]      
    range [ 0.0  15.0 ]     # 雷达数据采集区间
    fov 360.0               # 视角
    samples 720             # 采样数
    color_rgba [ 0 0 1 0.15 ] # 可视化光束颜色以及透明度
  )
  model # 雷达外观
  (
    pose [ 0 0 0 0 ]        # 雷达位姿
    size [ 0.07 0.07 0.05 ] # 雷达尺寸信息
    color "blue"            # 雷达颜色
  )
)