"""Door traversal goes straight to the configured exit goal."""

from unittest.mock import Mock, patch

from coverage_mission.coverage_mission_node import CoverageMissionNode


def test_crossing_goal_goes_to_end_not_center():
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.fixed_heading_yaw = 0.0
    mission.door_end = (3.5, 0.3)
    mission._send_nav_goal = Mock()

    CoverageMissionNode._send_door_traverse_goal(mission)

    mission._send_nav_goal.assert_called_once_with('door_traverse', 3.5, 0.3, 0.0)


def test_successful_exit_goal_finishes_without_center_check():
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.goal_token = 1
    mission.state = CoverageMissionNode.DOOR_TRAVERSE
    mission.door_end = (3.5, 0.3)
    mission.nav_retry_count = 2
    mission.nav_retry_at = 10.0
    future = Mock()
    future.result.return_value.status = 4

    with patch.object(CoverageMissionNode, 'get_logger', return_value=Mock()):
        CoverageMissionNode._goal_result(mission, future, 1, 'door_traverse')

    assert mission.state == CoverageMissionNode.DOOR_DONE
    assert mission.nav_retry_count == 0
