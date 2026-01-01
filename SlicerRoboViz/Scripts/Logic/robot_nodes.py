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
    segment_SPs: str = "" # sample points
    old_segment_SPs: str = ""
    segment_end_transforms: str = ""


@parameterNodeWrapper
class RobotNode:
    """Parameter node wrapper for robot."""
    robot_names: list[str] = []
    urdf_file_paths: list[str] = []