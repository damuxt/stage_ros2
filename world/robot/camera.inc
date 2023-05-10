define my_camera camera
(
	range [ 0.3 3.0 ] # 相机采样范围
	resolution [ 160 90 ]  #相机分辨率 1280 × 720 / 8
 	fov [ 87 58 ] # 相机视场
	pantilt [ 0 0 ] # 相机姿态
	alwayson 1 # 是否一直处于启动状态
    size [ 0.025 0.09 0.025 ] # 相机尺寸 
    color "gray" # 相机颜色
)