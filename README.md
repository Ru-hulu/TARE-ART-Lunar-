修改机器人尺寸(碰撞检测)：  
path_generator.m中的searchRadius = 0.45;  
将生成文件替换到相同文件夹下。  
localPlanner.cpp searchRadius = 0.45;  
terrain_analysis.launch  
处理凹陷区域为障碍物：noDataObstacle  
传感器距离地面的高度,不是机器人的高度：vehicleHeight  
没有激光雷达数据的地方认为是障碍物点：noDataObstacle = true;一般给false，如果给true,点云稀疏，难以导航  
pathScale：计算轨迹的长度，用于避碰，是轨迹避碰检测的最长距离。越长一般越安全  
minPathScale：轨迹避碰检测的最短距离  
pathScaleStep：检测步长  
local_planner.launch  

useInclToStop：线性加速度停止  
slowDwnDisThre：距离目标点停止距离参数  
inclRateThre：停止前加速度(减速)大小  
adjacentRange：碰撞检测的最大范围(并不是最终的范围)，在这个范围内的点都会首先被加入到碰撞检测的点云内，  
但是这并不是最终碰撞检测的范围，最终检测的范围是和速度相关的  

修改机器人速度：  
	maxSpeed:最大速度  
	autonomySpeed:自主移动(探索)速度  
	maxAccel:最大加速度  
	string pathFolder;使用matlab生成路径集合的文件路径  
	double vehicleLength = 0.6;车辆的长度，单位m  
	double vehicleWidth = 0.6;车辆的宽度，单位m  
	double sensorOffsetX = 0;传感器坐标系与车体中心的偏移量  
	double sensorOffsetY = 0;传感器坐标系与车体中心的偏移量  
	double obstacleHeightThre = 0.15;障碍物高度阈值  
	double groundHeightThre = 0.08;地面高度阈值  
	yawRateGain、stopYawRateGain角加速度(度)  
	maxYawRate 最大角速度(度)  

手柄操作：  
手柄X切换模式(手动\探索)
手柄右上button清空地形图
