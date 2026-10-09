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

# 首次测试line_follower.py
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
  -p video_source:=$HOME/wheel_robot/recordings/
# 录制视频
ros2 run wheel_robot camera_recorder.py
注意输出目录是 ~/wheel_robot/recordings/
已录制：
recordings/course_20261009_164310.avi
/home/orangepi/wheel_robot/recordings/course_20261009_164445.avi