import ti5_isaaclab.tasks.locomotion.velocity.mdp as mdp
from omni.isaac.lab.managers import RewardTermCfg as RewTerm  # noqa: F401
from omni.isaac.lab.managers import SceneEntityCfg  # noqa: F401
from omni.isaac.lab.utils import configclass
from ti5_isaaclab.tasks.locomotion.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg,
    RewardsCfg,
)


USE_ORIGINAL_MODEL = True

##
# Pre-defined configs
##
# use local assets
from ti5_isaaclab.assets.ti5 import TI5_T170A_CFG, TI5_T170A_FIXED_CFG  # isort: skip


@configclass
class Ti5T170ARewards(RewardsCfg):
    """Reward terms for the MDP."""

    # add your own reward functions here


@configclass
class Ti5T170ARoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: Ti5T170ARewards = Ti5T170ARewards()

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # ------------------------------Sence------------------------------
        # switch robot to Ti5 T170A
        if USE_ORIGINAL_MODEL:
            self.scene.robot = TI5_T170A_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")  # Use original model
        else:
            self.scene.robot = TI5_T170A_FIXED_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")  # Use fixed model
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/base"

        # ------------------------------Observations------------------------------
        # self.observations.policy.base_lin_vel = None
        # self.observations.policy.height_scan = None

        # ------------------------------Actions------------------------------
        # reduce action scale
        # self.actions.joint_pos.scale = 1.0

        # ------------------------------Events------------------------------
        self.events.base_external_force_torque.params["asset_cfg"].body_names = ["Link_x3"]
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }
        self.events.push_robot = None
        self.events.add_base_mass = None
        self.events.randomize_actuator_gains = None
        self.events.randomize_joint_parameters = None

        # ------------------------------Rewards------------------------------
        # General
        # UNUESD self.rewards.is_alive.weight = 0
        self.rewards.is_terminated.weight = -200

        # Root penalties
        self.rewards.lin_vel_z_l2.weight = 0
        self.rewards.ang_vel_xy_l2.weight = -0.05
        self.rewards.flat_orientation_l2.weight = -1.0
        self.rewards.base_height_l2.weight = 0
        self.rewards.body_lin_acc_l2.weight = 0

        # Joint penaltie
        self.rewards.joint_torques_l2.weight = 0
        # UNUESD self.rewards.joint_vel_l1.weight = 0.0
        self.rewards.joint_vel_l2.weight = 0
        self.rewards.joint_acc_l2.weight = -1.25e-7
        self.rewards.joint_acc_l2.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=["Leg_[LR]_[34]"])
        # self.rewards.joint_torques_l2.weight = -1.5e-7
        # self.rewards.joint_torques_l2.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=["Leg_[LR]_[3456]"])
        # Penalize deviation from default of the joints that are not essential for locomotion
        self.rewards.create_joint_deviation_l1_rewterm("joint_deviation_foot_l1", -0.5, ["Leg_[LR]_[12]"])
        if USE_ORIGINAL_MODEL:
            self.rewards.create_joint_deviation_l1_rewterm("joint_deviation_waist_l1", -0.5, ["Waist_.*"])
            self.rewards.create_joint_deviation_l1_rewterm("joint_deviation_head_l1", -0.1, ["Head_.*"])
            self.rewards.create_joint_deviation_l1_rewterm("joint_deviation_arm_l1", -0.1, ["Arm_[LR]_[14]"])
            self.rewards.create_joint_deviation_l1_rewterm("joint_deviation_arm_fixed_l1", -0.2, ["Arm_[LR]_[23567]"])
        # Penalize ankle joint limits
        self.rewards.joint_pos_limits.weight = -1.0
        self.rewards.joint_pos_limits.params["asset_cfg"].joint_names = ["Leg_[LR]_[56]"]
        self.rewards.joint_vel_limits.weight = 0

        # Action penalties
        self.rewards.action_rate_l2.weight = -0.005
        # UNUESD self.rewards.action_l2.weight = 0.0

        # Contact sensor
        self.rewards.undesired_contacts.weight = 0
        self.rewards.contact_forces.weight = 0

        # Velocity-tracking rewards
        self.rewards.track_lin_vel_xy_exp.weight = 1.0
        self.rewards.track_lin_vel_xy_exp.func = mdp.track_lin_vel_xy_yaw_frame_exp
        self.rewards.track_lin_vel_xy_exp.params["std"] = 0.5
        self.rewards.track_ang_vel_z_exp.weight = 1.0
        self.rewards.track_ang_vel_z_exp.func = mdp.track_ang_vel_z_world_exp
        self.rewards.track_ang_vel_z_exp.params["std"] = 0.5

        # Others
        self.rewards.feet_air_time.weight = 0.5
        self.rewards.feet_air_time.func = mdp.feet_air_time_positive_biped
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = ["Link_[lr]6"]
        self.rewards.feet_air_time.params["threshold"] = 0.4
        self.rewards.foot_contact.weight = 0
        self.rewards.base_height_rough_l2.weight = 0
        self.rewards.feet_slide.weight = -0.2
        self.rewards.feet_slide.params["sensor_cfg"].body_names = ["Link_[lr]6"]
        self.rewards.feet_slide.params["asset_cfg"].body_names = ["Link_[lr]6"]
        self.rewards.joint_power.weight = 0
        self.rewards.stand_still_when_zero_command.weight = 0

        # If the weight of rewards is 0, set rewards to None
        if self.__class__.__name__ == "Ti5T170ARoughEnvCfg":
            self.disable_zero_weight_rewards()

        # ------------------------------Terminations------------------------------
        if USE_ORIGINAL_MODEL:
            self.terminations.illegal_contact.params["sensor_cfg"].body_names = ["base", "Link_[xsb].*", "Link_[lr][12345]"]
        else:
            self.terminations.illegal_contact.params["sensor_cfg"].body_names = ["base", "Link_[xsb].*"]

        # ------------------------------Commands------------------------------
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)
