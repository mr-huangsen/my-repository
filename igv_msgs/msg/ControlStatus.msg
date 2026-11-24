int8 path_check        # 路径检查是否通过 0-通过，1-时间差过大，2-路径曲率过大 3-曲率、航向角的变化率过大 4-路径点尺寸的不一致
int8 trapped           # 是否被困 0-正常运动 1- 被困
int8 location_warp     # 定位 0-正常 1-定位时间与当前时间的差距过大 2-定位速度与底盘反馈速度差距过大 3-定位失败
int8 solution          # 控制求解 0-求解正常 1-求解失败
int8 deviation         # 有无跑偏 0-正常运动无跑偏 1-跑偏
int8 prediction_check  # 预测路线安全性 0-安全 1-不安全

