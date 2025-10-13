### 02解决IMU获取yaw轴为-pi到pi非连续问题

#### 目的

通过连续化保证控制机器人过程中没有数据跳变点

#### 思路

获取当前角度和记录上次角度，做差比较，若大于（或小于一定值说明存在跳变点）

#### 代码示例

```c
yaw_angle_last=yaw_angle;
yaw_angle=*(chassis_INS_angle+ INS_YAW_ADDRESS_OFFSET)
	if(yaw_angle-yaw_angle_last<-PI)
	{}
	else if(yaw_angle-yaw_angle_last>PI)
	{}
	else 
	{
		yaw_angle_total += yaw_angle-yaw_angle_last;
	}

```

