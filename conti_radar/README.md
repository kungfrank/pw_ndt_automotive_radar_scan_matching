# conti_radar
在看過網路上存Conti雷達的ros msg沒有辦法全部儲存nuScenens所有的資料，於是自己改寫msg。以下是我rosbag儲存的資料類型：
## Measurement.msg
```cpp=
Header header
ContiRadar[] points
```
## ContiRadar.msg
```cpp=
Header header
uint16 id
float32 longitude_dist	#x
float32 lateral_dist	#y
float32 longitude_vel	#vx
float32 lateral_vel	#vy
float32 longitude_vel_comp	#vx_comp,the velocities in m/s compensated by the ego motion
float32 lateral_vel_comp	#vy_comp,the velocities in m/s compensated by the ego motion
uint16 dynProp		#Dynamic property of cluster to indicate if is moving or not
float32 rcs		#radar criss section in dBm^2
uint32 is_quality_valid
uint32 ambig_state	#State of Doppler (radial velocity) ambiguity solution
uint32 x_rms
uint32 y_rms
uint32 vx_rms
uint32 vy_rms
uint32 invalid_state
uint32 ProbExists
```
