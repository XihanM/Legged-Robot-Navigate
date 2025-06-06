from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class Go2RoughCfg( LeggedRobotCfg ):

    class env( LeggedRobotCfg.env ):
        num_envs = 4096
        num_observations = 235
        symmetric = True  #对称训练True :Actor和Critic共享观测  set num_privileged_obs = None;    false: Critic获得额外信息 num_privileged_obs = observations + 187 ,set "terrain.measure_heights" to true
        num_privileged_obs = 235#num_observations + 187 # if not None a priviledge_obs_buf will be returned by step() (critic obs for assymetric training). None is returned otherwise 
        num_actions = 12    #动作空间配置，每腿3个关节，控制方式有位置控制和扭矩控制
        env_spacing = 3.  # 仅适用于平面环境，机器人间间距3米 not used with heightfields/trimeshes 
        send_timeouts = True #帮助算法区分任务成功完成和任务失败； send time out information to the algorithm
        episode_length_s = 25 # 每个episode的最大持续时间。episode length in seconds
    class terrain( LeggedRobotCfg.env ):
        mesh_type = 'competition' # "heightfield" # none, plane, heightfield or trimesh
        horizontal_scale = 0.25 # [m]网格0.25m
        vertical_scale = 0.005 # [m]
        border_size = 25 # [m]
        curriculum = False
        static_friction = 1.0
        dynamic_friction = 1.0
        restitution = 0.
        # rough terrain only:
        measure_heights = True
        #measured_points_x = [-0.45, -0.3, -0.15, 0, 0.15, 0.3, 0.45, 0.6, 0.75, 0.9, 1.05, 1.2] # 1mx1.6m rectangle (without center line)
        #measured_points_y = [-0.75, -0.6, -0.45, -0.3, -0.15, 0., 0.15, 0.3, 0.45, 0.6, 0.75]
        measured_points_x = [-0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2] # 1mx1.6m rectangle (without center line)
        measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
        #measured_points_x = [-0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8] # 1mx1.6m rectangle (without center line)
        #measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
        selected = False # select a unique terrain type and pass all arguments
        terrain_kwargs = None # Dict of arguments for selected terrain
        max_init_terrain_level = 5 # starting curriculum state
        terrain_length = 12.
        terrain_width = 12.
        num_rows= 9 # number of terrain rows (levels)
        num_cols = 1 # number of terrain cols (types)
        # terrain types: [smooth slope, rough slope, stairs up, stairs down, discrete]
        terrain_proportions = [0,1, 0, 0, 0]   #terrain_proportions = [0,1, 0, 0, 0]
        # trimesh only:
        slope_treshold = 0.75 # slopes above this threshold will be corrected to vertical surfaces

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0,0.0, 0.42] # x,y,z [m]机器人初始位置
        default_joint_angles = { # = target angles [rad] when action = 0.0
            # 髋关节角度（控制腿部外展/内收）
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]
            # 大腿关节角度（控制腿部前摆/后摆）
            'FL_thigh_joint': 0.8,     # [rad]
            'RL_thigh_joint': 1.,   # [rad]
            'FR_thigh_joint': 0.8,     # [rad]
            'RR_thigh_joint': 1.,   # [rad]
            # 小腿关节角度（控制腿部伸展/弯曲）
            'FL_calf_joint': -1.5,   # [rad]
            'RL_calf_joint': -1.5,    # [rad]
            'FR_calf_joint': -1.5,  # [rad]
            'RR_calf_joint': -1.5,    # [rad]
        }
#机器人指令生成策略参数
    class commands( LeggedRobotCfg.commands ):
        curriculum = False   #是否启用指令难度递增的课程学习策略
        max_curriculum = 1.  #课程学习的最大难度等级
        num_commands = 4 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 10. #指令变更间隔时间（秒） time before command are changed[s]
        heading_command = True # if true:根据当前航向与目标航向的误差自动计算所需角速度
        class ranges:
            lin_vel_x = [0.5, 2.0] # min max [m/s]前进后退速度，正值表示前进
            lin_vel_y = [0., 0.]   # min max [m/s]侧向平移速度 不训练侧向移动能力
            ang_vel_yaw = [0., 0.]    # min max [rad/s]原地转向速度
            heading = [0, 0]  #目标航向角范围，当前设为0 表示机器人始终面向前方

#机器人控制策略配置参数
    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P' # P: position位置控制, V: velocity速度, T: torques力矩
        stiffness = {'joint': 50.}  #[N*m/rad]刚度系数
        damping = {'joint': 2}     # [N*m*s/rad]阻尼系数
        # action scale: 目标角度= actionScale * 策略网络输出的动作 + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4 #每四次仿真更新策略网络输出一次控制指令，可以降低计算量

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2/urdf/go2.urdf'
        name = "go2"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf", "base"] # penalize contacts on these links
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter自碰撞配置，0启用，机器人不同部位碰撞时会产生物理交互
        flip_visual_attachments = True
    class domain_rand:
        randomize_friction = False   #是否随机化地面摩擦系数
        friction_range = [0.2, 1.5]
        randomize_base_mass = False #是否随机化机器人躯干质量
        added_mass_range = [-4., 4.]
        push_robots = False #是否随机推机器人
        push_interval_s = 15 #推机器人间隔时间
        max_push_vel_xy = 1. # 最大推机器人速度

        randomize_base_com = False   #随机偏移机器人质心的默认位置
        added_com_range = [-0.15, 0.15]

        randomize_motor = False  #在每个 episode 开始时，随机缩放电机强度
        motor_strength_range = [0.8, 1.2]

    class rewards( LeggedRobotCfg.rewards ):
        class scales( LeggedRobotCfg.rewards.scales ):
            termination = -0.
            #任务奖励 线速度跟踪奖励、角速度跟踪奖励
            #tracking_goal_vel = 1.5
            #tracking_yaw = 0.5

            tracking_lin_vel = 2.0     #线速度跟踪奖励权重  鼓励机器人接近目标线速度
            tracking_ang_vel = 0.5     #角速度跟踪奖励权重 鼓励机器人接近目标角速度
            lin_vel_z = -0.0       #-1.0  # 鼓励机器人保持水平运动 垂直Z轴线速度惩罚权重
            ang_vel_xy = -0.01     #-0.05 # xy平面角速度惩罚权重 鼓励机器人保持水平运动
            orientation = -0.1     #-1.  # 姿态方向惩罚权重 鼓励机器人保持正确的姿态
            torques = -0.0002      #-0.00001 关节扭矩惩罚权重 鼓励机器人使用较小的关节扭矩 减少能量消耗
            dof_vel = -2.5e-7       #关节速度惩罚权重 鼓励机器人保持较小的关节速度
            dof_acc = -2.5e-7      # -2.5e-7 关节加速度惩罚权重 鼓励机器人保持较小的关节加速度
            base_height = -0. 
            feet_air_time =  1.0   #脚部悬空时间奖励权重 ？？？
            collision = -1.        #-10.     机器人与环境碰撞时触发
            feet_stumble = -0.0     # -1  脚部绊倒惩罚权重
            action_rate = -0.001   #-0.1   动作变化率惩罚权重（抑制动作突变）
            stand_still = -0.1     #抑制机器人长时间不动
            dof_pos_limits =-0.01  # 关节位置限制惩罚权重 鼓励机器人保持关节位置在URDF定义的限制范围内
            #goal_pos = 0.45

            #action_rate = -0.1
            #delta_torques = -1.0e-7
            #hip_pos = -0.5
            #dof_error = -0.04
            #feet_stumble = -1
            #feet_edge = -1

# step 1 
# negtive reward -> -0.001



# tracking_lin_vel 0.02
# dof_pos_limits -0.1   -10  -> -1 : -0.01 


# reward 100
# tracking_lin_vel 0.9

# base_height = -0.01 -> base_height = -0.05
# orientation =-0.0001  orientation= -0.1

        only_positive_rewards = True # if true 强制所有时间步的总奖励为非负数（即负值被裁剪为 0）,避免早期停止
        tracking_sigma = 0.25 # tracking reward = exp(-error^2/sigma)计算目标跟踪任务的奖励
        soft_dof_pos_limit = 0.9 # urdf限制百分比，高于此限制的值将收到惩罚；percentage of urdf limits, values above this limit are penalized
        soft_dof_vel_limit = 1.   #关节速度软限制
        soft_torque_limit = 1.  #机器人关节施加扭矩的软限制阈值
        base_height_target = 0.25   #机器人躯干（base）离地面的高度。
        max_contact_force = 100. # 高于此值的力将收到惩罚 forces above this value are penalized

class Go2RoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_go2'

  
        policy_class_name = 'ActorCritic'
        algorithm_class_name = 'PPO'
        num_steps_per_env = 48 # per iteration
        max_iterations = 6000 # number of policy updates

        # logging
        save_interval = 50 # check for potential saves every this many iterations

        resume = False
        load_run = -1 # -1 = last run
        checkpoint = -1 # -1 = last saved model
        resume_path = None # updated from load_run and chkpt