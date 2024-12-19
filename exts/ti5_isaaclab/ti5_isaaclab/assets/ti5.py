"""Configuration for Ti5 robots.

The following configurations are available:

* :obj:`TI5_T170A_CFG`: Ti5 T170A robot

Reference: https://github.com/xxx
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg

from ti5_isaaclab.assets import ISAACLAB_ASSETS_DATA_DIR

##
# Configuration
##


TI5_T170A_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/Ti5/T170A/ti5_t170a.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.0),
        joint_pos={
            "Leg_L_1": 0.0, "Leg_L_2": 0.0, "Leg_L_3": 0.2, "Leg_L_4": -0.5, "Leg_L_5": -0.3, "Leg_L_6": 0.0,
            "Leg_R_1": 0.0, "Leg_R_2": 0.0, "Leg_R_3": 0.2, "Leg_R_4": -0.5, "Leg_R_5": 0.3, "Leg_R_6": 0.0,
            "Arm_L_1": 0.0, "Arm_L_2": -1.48, "Arm_L_3": 1.57, "Arm_L_4": 0.0, "Arm_L_5": 0.0, "Arm_L_6": 0.0, "Arm_L_7": 0.0,
            "Arm_R_1": 0.0, "Arm_R_2": -1.48, "Arm_R_3": -1.57, "Arm_R_4": 0.0, "Arm_R_5": 0.0, "Arm_R_6": 0.0, "Arm_R_7": 0.0,
            "Head_.*": 0.0,
            "Waist_.*": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "Leg": ImplicitActuatorCfg(
            joint_names_expr=["Leg_[LR]_[1234]"],
            stiffness={
                "Leg_[LR]_1": 150.0,
                "Leg_[LR]_2": 150.0,
                "Leg_[LR]_3": 150.0,
                "Leg_[LR]_4": 150.0,
            },
            damping={
                "Leg_[LR]_1": 15.0,
                "Leg_[LR]_2": 15.0,
                "Leg_[LR]_3": 15.0,
                "Leg_[LR]_4": 15.0,
            },
        ),
        "Feet": ImplicitActuatorCfg(
            joint_names_expr=["Leg_[LR]_[56]"],
            stiffness={
                "Leg_[LR]_5": 150.0,
                "Leg_[LR]_6": 150.0,
            },
            damping={
                "Leg_[LR]_5": 15.0,
                "Leg_[LR]_6": 15.0,
            },
        ),
        "Arm": ImplicitActuatorCfg(
            joint_names_expr=["Arm_.*"],
            stiffness={
                "Arm_[LR]_1": 150.0,
                "Arm_[LR]_2": 150.0,
                "Arm_[LR]_3": 150.0,
                "Arm_[LR]_4": 150.0,
                "Arm_[LR]_5": 150.0,
                "Arm_[LR]_6": 150.0,
                "Arm_[LR]_7": 150.0,
            },
            damping={
                "Arm_[LR]_1": 15.0,
                "Arm_[LR]_2": 15.0,
                "Arm_[LR]_3": 15.0,
                "Arm_[LR]_4": 15.0,
                "Arm_[LR]_5": 15.0,
                "Arm_[LR]_6": 15.0,
                "Arm_[LR]_7": 15.0,
            },
        ),
        "Waist": ImplicitActuatorCfg(
            joint_names_expr=["Waist_.*"],
            stiffness={
                "Waist_Pitch": 150.0,
                "Waist_Roll": 150.0,
                "Waist_Yaw": 150.0,
            },
            damping={
                "Waist_Pitch": 15.0,
                "Waist_Roll": 15.0,
                "Waist_Yaw": 15.0,
            },
        ),
        "Head": ImplicitActuatorCfg(
            joint_names_expr=["Head_.*"],
            stiffness={
                "Head_Pitch": 150.0,
                "Head_Roll": 150.0,
                "Head_Yaw": 150.0,
            },
            damping={
                "Head_Pitch": 15.0,
                "Head_Roll": 15.0,
                "Head_Yaw": 15.0,
            },
        ),
    },
)
"""Configuration for the Ti5 T170A Humanoid robot."""

TI5_T170A_FIXED_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/Ti5/T170A/ti5_t170a_fixed.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.0),
        joint_pos={
            "Leg_L_1": 0.0, "Leg_L_2": 0.0, "Leg_L_3": 0.2, "Leg_L_4": -0.5, "Leg_L_5": -0.3, "Leg_L_6": 0.0,
            "Leg_R_1": 0.0, "Leg_R_2": 0.0, "Leg_R_3": 0.2, "Leg_R_4": -0.5, "Leg_R_5": 0.3, "Leg_R_6": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "Leg": ImplicitActuatorCfg(
            joint_names_expr=["Leg_[LR]_[1234]"],
            stiffness={
                "Leg_[LR]_1": 150.0,
                "Leg_[LR]_2": 150.0,
                "Leg_[LR]_3": 150.0,
                "Leg_[LR]_4": 150.0,
            },
            damping={
                "Leg_[LR]_1": 15.0,
                "Leg_[LR]_2": 15.0,
                "Leg_[LR]_3": 15.0,
                "Leg_[LR]_4": 15.0,
            },
        ),
        "Feet": ImplicitActuatorCfg(
            joint_names_expr=["Leg_[LR]_[56]"],
            stiffness={
                "Leg_[LR]_5": 150.0,
                "Leg_[LR]_6": 150.0,
            },
            damping={
                "Leg_[LR]_5": 15.0,
                "Leg_[LR]_6": 15.0,
            },
        ),
    },
)
"""Configuration for the Ti5 T170A FIXED Humanoid robot."""
