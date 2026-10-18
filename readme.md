# 杭电轮腿一队调试日志
起 2026.10.1 王赛
-----------------
# 必看（日志说明）
该文件存放基本的教程和调试日志记录
## 如何使用？
简单来说，就是存放你做了什么，改了什么，或者有什么需要交代的
调车的困难就在于参数和灵感，昨天的参数和灵感都是昨天的，不妨将它立刻写下来、记录下来，日积月累才能调好车，这也是我的习惯
所以，如果有任何文件的修改，新增或者删除，一定一定一定要在此文件记录下来，虽然有git，但此方式防遗忘，也好溯源

# AI代码说明
不建议AI参加日志的编写，另外不建议整个用AI调试
我不管你模型有多好，AI写的代码一定要review！或者足够调试，不然是绝对不会采用的！

# 连接小车
使用vscode ssh-remote
需要小车和电脑连接同一个局域网
目前暂时用热点：手机热点名字设置为robot，密码为123456789
输入命令：ssh orangepi@orangepi4pro.local
或，ssh orangepi@10.218.252.13

# git 仓库说明
仓库地址 https://github.com/soyorin01/wheel_robot
更新仓库
git add .
git commit -m "你想说的话"
git push

# 遥控器使用说明
模式	SWA	SWB	SWC	SWD	VRA	VRB	左摇杆	右摇杆
停止	0	0	任意	任意	最小	0	任意	任意
遥控模式1	1	0	1或者0	0	最大	0	前后与航向	高度与横滚
遥控模式2	1	0	1或者0	1	最大	0	前后与航向	高度与俯仰
ROS模式	2	接收到上位机数据控制移动底盘：前后，航向，横滚，俯仰，高度。上位机没有发布数据，遥控器会接管控制权。如果SWA是自动复位，SWA下拨动一次就是启动ROS控制模式
上电前，确保轮腿伸直成两条直线。确保遥控器处于初始位置
顺时针调节VRA旋钮，直到听见蜂鸣器
随后将SWA拨到1，将机器人放到地上即可

关机一定要放回架子上并把遥控器置初始位置！！

# 重新编译ESP32下位机
当前哈希值：e93a951b54ba9882fd2324134f9b406b37daa1fd589ef16a8967aa17993b59c9   ---10.9
跑通了，果然是官方固件的问题

# 键盘遥控机器人
1.确保SWA置于2
ros2 launch wheel_robot base.launch.py
ros2 run wheel_robot keyboard_teleop.py
/cmd_vel	Twist		前后 + 转向
/cmd_posture	JointState		腿高 / 横滚 / 俯仰 / 滑步
    posture.name = ["joint_height", "joint_roll", "joint_pitching", "joint_slide"]
    posture.position = [self.height, self.roll, self.pitch, self.slide]
/cmd_jump	Bool		跳跃触发
# 摄像头节点astra_camera.launch.py
/camera/color/image_raw 彩色图 640×480@30
/camera/color/camera_info	CameraInfo	彩色内参
/camera/depth/image_raw	Image	深度图 16UC1（mm）
/camera/depth/camera_info	CameraInfo	深度内参
/camera/depth/left_distance	Range	左前障碍距离
/camera/depth/center_distance	Range	正前障碍距离
/camera/depth/right_distance	Range	右前障碍距离

# 首次测试dual_line_follower.py
视频在 /home/orangepi/wheel_robot/install/recordings/dual_white_line_20261009_153506.avi 
┌──────────────────────┬──────────────────┐
│                      │  ROI 320×160     │  ← 处理图（地面区域缩放后）
│   RAW 640×480        │                  │
│   （相机原图）        ├──────────────────┤
│   蓝框 = 地面 ROI     │  MASK 320×160    │  ← 白线二值掩码
│                      │                  │
│   ●  ●  ●            ├──────────────────┤
│                      │  遥测文字面板      │  ← FPS/error/state...
│                      │                  │
└──────────────────────┴──────────────────┘
颜色	形状	含义
🟢 绿色实心圆	半径 6（RAW 上 8）	NEAR 段（画面下部 / 离车最近）检出的白线质心
🟠 橙黄实心圆	同上	MID 段（中距离）检出的质心
🔵 天蓝实心圆	同上	FAR 段（画面上部 / 最远）检出的质心
🟣 品红空心圈	半径 8（RAW 上 10，线宽 3）	远中近加权融合后的最终目标点 target_x —— 这个才是真正喂给控制器去追踪的点
正常跟踪时：品红空心圈应该落在青色竖线上（见下）；如果偏了，车就会打方向去追。
颜色	形状	含义
⚪ 白色竖线	竖线	画面正中 x = 160
🩵 青色竖线	竖线	控制参考线 x = 125（= 160 + offset(−35)）—— 品红圈要收敛到这里
   彩色横线	横线	远/中/近三段的分界线（画在每段顶部 y0）。**三条天生都是横的**，其中 MID 那条是橙黄，最容易被误认成"黄线"
🟦 蓝色矩形框	框	RAW 上的地面 ROI 范围（上 45% 被裁掉）

修改了文件，加上了回放视频功能：
ros2 run wheel_robot line_follower.py --ros-args \
  -p video_source:=$HOME/wheel_robot/recordings/course_20261010_160202.avi \
  -p video_loop:=true -p show_debug_window:=true
# 注意：带 video_source 时代码会强制 enable_motion=False，不会驱动真车
# 录制视频
ros2 run wheel_robot camera_recorder.py
注意输出目录是 ~/wheel_robot/recordings/
已录制：
recordings/course_20261009_164310.avi
/home/orangepi/wheel_robot/recordings/course_20261009_164445.avi

# 10.10 丢线排查（line_follower.py 调参）
现象：丢线严重。用新增的 tools/inspect_frames.py 逐帧量出白线真实值：
    白线 S=50~125、V=110~175；红地面 S>=153；画面整体偏暗（V p50=71）
改动都在 line_follower.py 里（带注释）：

1. RED_S_MIN 65 -> 145          【主因】白线被红地面染成 H=5~13/S=50~125，被当成红色剔除了
2. WHITE_S_MAX 72 -> 135        白线 S 中位 75，原值砍掉一半以上
3. DYNAMIC_V_MIN 115 -> 90      原下限直接高于白线的 V，白线永远过不了
4. DYNAMIC_V_OFFSET 16 -> 8
5. DYNAMIC_V_PERCENTILE 82 -> 75
6. STRONG_WHITE_V_MARGIN 28 -> 12
7. MIN_VERTICAL_COVERAGE 0.24 -> 0.10
8. MIN_COMPONENT_AREA 20 -> 12
9. OPEN_ITERATIONS 1 -> 0（关闭开运算）、CLOSE_KERNEL_SIZE 5 -> 7
   3x3 开运算会把细白线侵蚀成碎片，是掩码「断点」的元凶

效果（75 帧，t=5~20s）：整帧有效 0% -> 91%，≥2段 0% -> 87%，≥3段 0% -> 39%

新增 tools/inspect_frames.py（离线调 HSV 用）：
    从录像截帧，导出【ROI | V | S | MASK】四联图 + 掩码联络表，
    直接复用 line_follower.py 的检测器和阈值。
    python3 src/wheel_robot/tools/inspect_frames.py recordings/xxx.avi \
      --start 8.0 --duration 0.3 --save-raw
    输出默认在 ~/wheel_robot/mask_debug/

调参证据图在 ~/wheel_robot/mask_debug/：
    morph_fix.png    开运算修正前后对比（最直观：碎片 -> 连续曲线）
    best_f282_t9.40s.png  最佳帧，3 段全中、置信度 0.956
    best_params.md   参数表 + 每项依据

# 10.10 dual_line_follower.py 同步调参
前 7 项改动与 line_follower.py 完全相同（RED_S_MIN / WHITE_S_MAX /
DYNAMIC_V_* / STRONG_WHITE_V_MARGIN / 关闭开运算 + 闭核改 7）。
另外必须多改一项：

    MAX_WHITE_PIXEL_RATIO 0.20 -> 0.50
    关闭开运算后掩码变厚（白像素中位 19%、最高 43%），原上限会否掉 26/75 帧，
    抬高后整帧有效 65% -> 100%
    （单线版实测这一项只 91%->92%，没必要改，故保持原值）

效果（75 帧，t=5~20s）：
    基线: 100%  /  ≥2段 49  /  ≥3段 18  /  conf 均值 0.382
    现在: 100%  /  ≥2段 75  /  ≥3段 72  /  conf 均值 0.900

# 10.11 dual_line_follower.py ROI_TOP_RATIO（看台干扰）

    ROI_TOP_RATIO 0.42 -> 0.54
    （这是模块常量，不是 ROS 参数，不能用 -p 覆盖，改完必须重新 colcon build）  

原因：画面顶部 0~52% 是看台（亮白低饱和）。它有两个害处：
  1. 被当成白线，而 FAR 段在融合里占 0.24 权重，会把方向盘带偏；
  2. 把 _make_mask 里整个 ROI 的全局动态亮度分位数抬上去，反而把真白线压掉。
  
dual_line_follower.py 也加了 video_source 回放（用法同单线版）：
    ros2 run wheel_robot dual_line_follower.py --ros-args \
      -p video_source:=$HOME/wheel_robot/recordings/course_20261010_160202.avi \
      -p video_loop:=true -p show_debug_window:=true

# dual_line_follower_illum.py
光照归一化，解决了局部过亮的问题
图像处理基本没什么问题，主要是导线逻辑不太行，建议忽略far
install/recordings_illum/dual_white_line_20261011_162200.avi

# shiwai.py
加入了日志输出，改了一些yaml里的参数
调试第一天
opencv处理有待优化
开局直接看到单边桥的胶带，就默认进单边桥模式导致平地起舞？
不应该消失而且过一小会再翘吗？
没搞懂，明天再修

# /home/orangepi/wheel_robot/src/wheel_robot/scripts/shiwai_far_line_fixed.py
① 巡线专用黄线阈值（分离出来了）
参数组	用途	H	S	V
lane_yellow_*（新增）	巡线（黄泡沫边缘）	24~40	≥35	≥70
yellow_*（保留原值）	场景识别（黄黑胶带 / 人行道 / 终点）	12~42	≥50	≥70
注册成了 ROS 参数，并在 _validate_parameters() 里加了范围校验。DualYellowLineDetector 现在吃 lane_yellow_*，TrackSceneDetector 还是吃 yellow_*。


② 巡线掩膜里排除「黄黑条纹」
新增 _stripe_exclusion_mask()（约 78 行），只在巡线掩膜里工作，不碰 TrackSceneDetector。

它替换掉了原来更粗暴的做法：

黄边旁边总有地面阴影 → 必然误删正常黄边。

新版要求局部重复的黄/黑交替才删：至少 4 段 run、跨度 ≤ 画面宽 16%、单段 ≥2px、未知色间隔 ≤4px，横竖两个方向都扫，最后按黄色连通块删（要求该块 ≥15% 面积被条纹支持）。

这个判据比「靠近黑色」合理得多。

③ 掩膜改成在整幅图上算
注释说明：条纹证据要在完整图像里确认，否则搜索框会把胶带裁成一小块黄色，判据失效。巡线 ROI 本身没扩大。
# shiwai.py
修改了单边巡线的虚拟中点

 0.38(=243px) 是估出来的，实测偏大：shiwai_20261018_142252.avi 第 0 帧
 双边 pairs=146、lane=397px，真实半宽只有 198px = 0.31。
 偏大意味着“虚拟中点”比真中点靠右 → 车会一直往右抖：
   左线在 x=100 时，0.38 → 虚拟中点 343 > 320 → 判成“该右转”，方向反了。
SINGLE_LEFT_HALF_RATIO = 0.31
SINGLE_RIGHT_HALF_RATIO = 0.31
BRIDGE_OFFSET_CM=0,取消了偏置带来的不必要的右边

增加了俯仰角10度，但还是很难看到赛道