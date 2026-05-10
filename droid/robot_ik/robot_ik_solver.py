import numpy as np
from dm_control import mjcf
from dm_robotics.moma.effectors import arm_effector, cartesian_6d_velocity_effector

from droid.robot_ik.arm import FrankaArm


class RobotIKSolver:
    """
    机器人逆运动学（IK）求解器
    
    核心功能：在不同动作空间之间进行转换
    - 笛卡尔空间（末端执行器位置/速度）
    - 关节空间（7个关节的角度/速度）
    - 夹爪空间（夹爪位置/速度）
    
    设计哲学：
    - 策略层输出归一化动作 [-1, 1]，与具体机器人硬件无关
    - 执行层根据硬件参数（max_delta, control_hz）转换为真实物理量
    - 这样同一套策略可以适配不同的机器人
    
    关键参数说明（隐含时间概念）：
    - control_hz = 15：控制频率，每秒控制15次
    - control_period = 1/15 ≈ 0.067秒：每次控制的间隔
    - max_lin_delta = 0.075：每周期最大线性位移（米）
    - max_rot_delta = 0.15：每周期最大旋转位移（弧度）
    - max_joint_delta = 0.2：每周期最大关节位移（弧度）
    """

    def __init__(self):
        # ==================== 关节空间参数 ====================
        # 每个关节每周期最大位移（弧度），共7个关节
        # 0.05 rad/step × 15Hz = 0.75 rad/s，匹配 DROID 数据采集时的慢速精细操作
        self.relative_max_joint_delta = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])

        # 用于统一限速的关节最大位移（取数组最大值）
        self.max_joint_delta = self.relative_max_joint_delta.max()  # = 0.05
        
        # ==================== 夹爪参数 ====================
        # 每周期夹爪最大位移（归一化 0-1）
        self.max_gripper_delta = 0.25
        
        # ==================== 笛卡尔空间参数 ====================
        # 每周期末端执行器最大线性位移（米）
        # 最大真实线速度 = 0.075 / (1/15) = 1.125 m/s
        self.max_lin_delta = 0.075
        
        # 每周期末端执行器最大旋转位移（弧度）
        # 最大真实角速度 = 0.15 / (1/15) = 2.25 rad/s
        self.max_rot_delta = 0.15
        
        # ==================== 控制频率 ====================
        # 控制频率（Hz），每秒进行多少次控制
        self.control_hz = 15
        # 对应控制周期 = 1/15 ≈ 0.067秒

        # ==================== 初始化 DM Control 环境和 IK 求解器 ====================
        # Franka Arm 的 DM Control 模型
        self._arm = FrankaArm()
        self._physics = mjcf.Physics.from_mjcf_model(self._arm.mjcf_model)
        
        # DM Robotics 的 Arm Effector，用于 IK 求解
        self._effector = arm_effector.ArmEffector(
            arm=self._arm, 
            action_range_override=None, 
            robot_name=self._arm.name
        )

        # 笛卡尔 6D 速度控制器的模型参数
        self._effector_model = cartesian_6d_velocity_effector.ModelParams(
            self._arm.wrist_site, 
            self._arm.joints
        )

        # 笛卡尔 6D 速度控制器的控制参数
        self._effector_control = cartesian_6d_velocity_effector.ControlParams(
            control_timestep_seconds=1 / self.control_hz,  # 控制周期
            max_lin_vel=self.max_lin_delta,                 # 最大线性速度
            max_rot_vel=self.max_rot_delta,                 # 最大旋转速度
            joint_velocity_limits=self.relative_max_joint_delta,  # 关节速度限制
            nullspace_joint_position_reference=[0] * 7,     # 零空间关节位置参考
            nullspace_gain=0.025,                           # 零空间增益
            regularization_weight=1e-2,                      # 正则化权重
            enable_joint_position_limits=True,              # 启用关节位置限制
            minimum_distance_from_joint_position_limit=0.3, # 关节限制最小距离
            joint_position_limit_velocity_scale=0.95,        # 关节限制速度比例
            max_cartesian_velocity_control_iterations=300, # 笛卡尔速度控制最大迭代
            max_nullspace_control_iterations=300,           # 零空间控制最大迭代
        )

        # 创建笛卡尔 6D 速度效应器（核心 IK 求解组件）
        self._cart_effector_6d = cartesian_6d_velocity_effector.Cartesian6dVelocityEffector(
            self._arm.name, 
            self._effector, 
            self._effector_model, 
            self._effector_control
        )
        self._cart_effector_6d.after_compile(self._arm.mjcf_model, self._physics)

    def cartesian_velocity_to_joint_velocity(self, cartesian_velocity, robot_state):
        """
        【核心 IK 求解】将笛卡尔空间速度转换为关节空间速度
        
        这是整个转换链中最复杂的步骤，使用 DM Control 的 IK 求解器
        
        Args:
            cartesian_velocity: 6D 笛卡尔速度 [vx, vy, vz, vroll, vpitch, vyaw]
                               范围 [-1, 1]（归一化）
            robot_state: 当前机器人状态，包含关节位置和速度
        
        Returns:
            joint_velocity: 7个关节的归一化速度（在 [-1, 1] 范围内）
        
        完整转换流程：
            1. cartesian_velocity (归一化) 
               ↓ cartesian_velocity_to_delta()
            2. cartesian_delta (真实位移: 米/弧度)
               ↓ DM Control IK 求解
            3. joint_delta (关节位移: 弧度)
               ↓ joint_delta_to_velocity()
            4. joint_velocity (归一化)
        """
        # 第1步：归一化速度 → 真实位移
        cartesian_delta = self.cartesian_velocity_to_delta(cartesian_velocity)
        
        # 第2步：获取当前关节状态
        qpos = np.array(robot_state["joint_positions"])
        qvel = np.array(robot_state["joint_velocities"])

        # 第3步：使用 DM Control IK 求解
        # 更新机械臂状态
        self._arm.update_state(self._physics, qpos, qvel)
        # 设置目标笛卡尔位移
        self._cart_effector_6d.set_control(self._physics, cartesian_delta)
        # 获取求解出的关节位移
        joint_delta = self._physics.bind(self._arm.actuators).ctrl.copy()
        np.any(joint_delta)

        # 第4步：关节位移 → 归一化关节速度
        joint_velocity = self.joint_delta_to_velocity(joint_delta)

        return joint_velocity

    # ==================== 速度 → 位移增量 ====================
    
    def gripper_velocity_to_delta(self, gripper_velocity):
        """
        将归一化夹爪速度转换为夹爪位置增量
        
        Args:
            gripper_velocity: 归一化夹爪速度（在 [-1, 1] 范围内）
        
        Returns:
            gripper_delta: 夹爪位置增量（归一化 0-1）
        """
        gripper_vel_norm = np.linalg.norm(gripper_velocity)

        if gripper_vel_norm > 1:
            gripper_velocity = gripper_velocity / gripper_vel_norm

        gripper_delta = gripper_velocity * self.max_gripper_delta

        return gripper_delta

    def cartesian_velocity_to_delta(self, cartesian_velocity):
        """
        将归一化笛卡尔速度转换为笛卡尔位移增量
        
        Args:
            cartesian_velocity: 6D 笛卡尔速度 [vx, vy, vz, vroll, vpitch, vyaw]
                               范围 [-1, 1]（归一化）
        
        Returns:
            delta: 6D 笛卡尔位移 [dx, dy, dz, droll, dpitch, dyaw]
                  单位：米和弧度
        
        说明：
            - 输入是归一化速度（伪速度）
            - 输出是每周期真实位移
            - 时间已隐含在 max_*_delta 中（每周期最大位移）
        """
        if isinstance(cartesian_velocity, list):
            cartesian_velocity = np.array(cartesian_velocity)

        # 分离线速度 和 旋转速度
        lin_vel, rot_vel = cartesian_velocity[:3], cartesian_velocity[3:6]

        # 计算速度向量范数（用于限幅）
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)

        # 限幅：如果速度向量长度超过1，等比例缩放
        if lin_vel_norm > 1:
            lin_vel = lin_vel / lin_vel_norm
        if rot_vel_norm > 1:
            rot_vel = rot_vel / rot_vel_norm

        # 归一化速度 × 每周期最大位移 = 这次的真实位移
        lin_delta = lin_vel * self.max_lin_delta   # × 0.075 米
        rot_delta = rot_vel * self.max_rot_delta   # × 0.15 弧度

        return np.concatenate([lin_delta, rot_delta])

    def joint_velocity_to_delta(self, joint_velocity):
        """
        将归一化关节速度转换为关节位置增量
        
        Args:
            joint_velocity: 7个关节的归一化速度（在 [-1, 1] 范围内）
                         这是伪速度，不是真实物理速度
        
        Returns:
            joint_delta: 7个关节的位置增量（真实弧度值）
        
        转换逻辑（隐含时间概念）：
            - 输入：normalized_velocity（范围 [-1, 1]）
            - 输出：delta（弧度）
            
            物理关系：
                real_velocity = normalized_velocity × max_real_velocity
                delta = real_velocity × control_period
                
                因为 max_real_velocity = max_delta / control_period
                所以 delta = normalized_velocity × max_delta
            
            换句话说：max_joint_delta 已经包含了"每周期能走多远"的信息
        """
        if isinstance(joint_velocity, list):
            joint_velocity = np.array(joint_velocity)

        # 计算对应的真实关节速度（rad/s）
        # = max_delta(0.2) / control_period(1/15s) = 3 rad/s
        relative_max_joint_vel = self.joint_delta_to_velocity(self.relative_max_joint_delta)
        
        # 计算归一化速度的最大范数（用于检测是否超速）
        max_joint_vel_norm = (np.abs(joint_velocity) / relative_max_joint_vel).max()

        # 如果超速（最大范数 > 1），按比例缩放回 [-1, 1] 范围
        if max_joint_vel_norm > 1:
            joint_velocity = joint_velocity / max_joint_vel_norm

        # 归一化速度 × 每周期最大位移 = 这次的真实位移
        # max_joint_delta = 0.2 弧度，已经隐含了 control_period 的信息
        joint_delta = joint_velocity * self.max_joint_delta

        return joint_delta

    # ==================== 位移增量 → 速度 ====================
    
    def gripper_delta_to_velocity(self, gripper_delta):
        """
        将夹爪位置增量转换为归一化夹爪速度
        """
        return gripper_delta / self.max_gripper_delta

    def cartesian_delta_to_velocity(self, cartesian_delta):
        """
        将笛卡尔位移增量转换为归一化笛卡尔速度
        
        Args:
            cartesian_delta: 6D 笛卡尔位移 [dx, dy, dz, droll, dpitch, dyaw]
                           单位：米和弧度
        
        Returns:
            velocity: 6D 归一化速度（在 [-1, 1] 范围内）
        """
        if isinstance(cartesian_delta, list):
            cartesian_delta = np.array(cartesian_delta)

        cartesian_velocity = np.zeros_like(cartesian_delta)
        cartesian_velocity[:3] = cartesian_delta[:3] / self.max_lin_delta
        cartesian_velocity[3:6] = cartesian_delta[3:6] / self.max_rot_delta

        return cartesian_velocity

    def joint_delta_to_velocity(self, joint_delta):
        """
        将关节位置增量转换为归一化关节速度
        
        Args:
            joint_delta: 7个关节的位置增量（弧度）
        
        Returns:
            velocity: 7个归一化关节速度（在 [-1, 1] 范围内）
        
        说明：这是 cartesian_velocity_to_delta 的逆过程
        """
        if isinstance(joint_delta, list):
            joint_delta = np.array(joint_delta)

        return joint_delta / self.max_joint_delta
