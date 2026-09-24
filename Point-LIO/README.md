## 各个点云数据解释：
1.customPoint: livox_driver2发送的初始点云
2.pl_surf:通过p_pre预处理：将有效的,根据point_filter_num下采样的，
                符合blind条件的，体素滤波过的点加入pl_surf中；并根据CustomPoint的offset_time，给出每个点的精确时间点curvature，表示每个点相对这一帧第一个点的时间戳。
2.lidar_buffer:通过livox_pcl_cbk，根据选项，将点云存入lidar_buffer中。可选
                1：将一帧点云按时间分开
                2：将连续con_frame_num帧点云存入lidar_buffer中
                3：将每帧点云存入lidar_buffer中
3.Measures:通过sync_packages，将lidar_buffer中的最早一份点云传给measures.