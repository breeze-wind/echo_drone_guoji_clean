"""Guard against OFFBOARD setpoint gaps across mission phases."""

import time
from unittest.mock import Mock

from coverage_mission.coverage_mission_node import CoverageMissionNode


def _mission_in(state):
    # Exercise the timer callback without starting ROS or touching hardware.
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.state = state
    mission.armed = True
    mission.offboard = True
    mission.require_offboard = True
    mission.goal_started_at = None
    mission.current_pose_time = time.monotonic()
    mission.pose_stale_timeout = 1.0
    mission.current_pose = object()
    mission.nav_state_pub = Mock()
    mission._publish_actuator_keepalive = Mock()
    mission._print_mission_status = Mock()
    mission._publish_position_target = Mock()
    return mission


def test_drop_dwell_refreshes_hold_target_and_position_mode():
    mission = _mission_in(CoverageMissionNode.DROP_DWELL)
    mission.drop_xy = (4.1, -0.2)
    mission.drop_height = 0.5
    mission.drop_commands = [2, 3, 4]
    mission.delivery_count = 0
    mission.actuator_command_sent_at = time.monotonic()
    mission.drop_dwell = 3.0
    mission.actuator_output_enabled = False
    mission.require_actuator_status = False

    CoverageMissionNode._tick(mission)

    mission._publish_position_target.assert_called_once_with(4.1, -0.2, 0.5)
    assert mission.nav_state_pub.publish.call_args.args[0].data is False


def test_search_repeats_navigation_mode_even_without_a_new_goal():
    mission = _mission_in(CoverageMissionNode.SEARCHING)
    mission.nav_retry_at = time.monotonic() + 10.0

    CoverageMissionNode._tick(mission)

    assert mission.nav_state_pub.publish.call_args.args[0].data is True
