from slicer.parameterNodeWrapper import (
    parameterNodeWrapper,
    WithinRange,
)

@parameterNodeWrapper
class RobotDescriptionNode:
    # Basic robot info
    robot_name: str = ""
    joint_names: str = ""
    link_names: str = ""
    segment_names: str = ""
    joint_mapping: str = ""

@parameterNodeWrapper
class RobotStateNode:
    """Parameter node wrapper for joint states."""
    time_stamp: float = 0.0
    joint_names: list[str] = []
    joint_positions: list[float] = []
    old_joint_positions: list[float] = []
    segment_names: list[str] = []
    segment_waypoints: str = ""
    old_segment_waypoints: str = ""
    segment_end_transforms: str = ""
    segment_global_waypoints: str = ""
