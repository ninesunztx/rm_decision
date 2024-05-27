## rm_decision

### 1.特点

- 哨兵机器人决策模块，采用状态机的方式完成。
- 逻辑相对简单，但主体框架比较完善，后续可以直接进行使用。
- 哨兵机器人的主体逻辑处理都可以该模块进行，包括裁判系统的解析，自瞄角度的限位等等。

### 2.代码分类

- auto_fsm.py   状态机的主文件
- callback_msg.py 回调函数文件
- nav_to_pose.py 常用函数文件
- robot_navigator.py  nav2官方pythonAPI

### 3.测试

```
ros2 launch rm_decision auto_fsm.py
```

```
ros2 topic pub -r 5 /refree_data rm_decision_interfaces/msg/Refree "{robot_hp: 600, base_hp: 1500, color: 1}"
```

启动一个Nav仿真即可，注意goal.yaml里的坐标要合法

### 4.适配自身队伍使用

可以修改自定义的消息类型，适配自身导航和自瞄算法

**此代码借鉴了华南农业自动步兵的决策代码和深北莫的哨兵决策代码**






