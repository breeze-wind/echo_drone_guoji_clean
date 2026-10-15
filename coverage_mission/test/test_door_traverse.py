"""Ordered doorway route and final landing-goal behavior."""

from unittest.mock import Mock, patch

from coverage_mission.coverage_mission_node import CoverageMissionNode


def test_crossing_navigation_goal_is_the_landing_point():
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.fixed_heading_yaw = 0.0
    mission.door_waypoints = [(1.0, 0.3), (2.0, 0.3), (3.5, 0.3)]
    mission.door_waypoint_index = 1
    mission.landing_xy = (8.0, -2.0)
    mission._send_nav_goal = Mock()

    CoverageMissionNode._send_door_traverse_goal(mission)

    mission._send_nav_goal.assert_called_once_with('door_route', 8.0, -2.0, 0.0)


def test_begin_door_traverse_uses_current_teb_without_profile_switch():
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.state = CoverageMissionNode.DOOR_TEB_SWITCH
    mission.door_waypoints = [(1.0, 0.3), (2.0, 0.3)]
    mission.door_waypoint_index = -1
    mission.nav_state_pub = Mock()
    mission._send_door_traverse_goal = Mock()
    mission._publish_active_door_via_point = Mock()
    mission.passing_door_pub = Mock()
    mission.landing_xy = (4.0, -2.0)
    mission.door_via_point_tolerance = 0.25
    mission.get_logger = Mock(return_value=Mock())

    CoverageMissionNode._begin_door_traverse(mission)

    assert mission.state == CoverageMissionNode.DOOR_TRAVERSE
    assert mission.door_waypoint_index == 0
    mission.nav_state_pub.publish.assert_called_once()
    mission.passing_door_pub.publish.assert_called_once()
    mission._publish_active_door_via_point.assert_called_once()
    mission._send_door_traverse_goal.assert_called_once()


def test_entering_via_radius_advances_without_waiting_for_nav_result():
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.state = CoverageMissionNode.DOOR_TRAVERSE
    mission.door_waypoints = [(1.0, 0.3), (2.0, 0.3)]
    mission.door_waypoint_index = 0
    mission.current_pose = Mock()
    mission.current_pose.transform.translation.x = 1.1
    mission.current_pose.transform.translation.y = 0.3
    mission.door_via_point_tolerance = 0.25
    mission._publish_active_door_via_point = Mock()
    mission.get_logger = Mock(return_value=Mock())

    CoverageMissionNode._update_door_via_progress(mission)

    assert mission.door_waypoint_index == 1
    mission._publish_active_door_via_point.assert_called_once()


def test_outside_via_radius_does_not_advance():
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.state = CoverageMissionNode.DOOR_TRAVERSE
    mission.door_waypoints = [(1.0, 0.3), (2.0, 0.3)]
    mission.door_waypoint_index = 0
    mission.current_pose = Mock()
    mission.current_pose.transform.translation.x = 1.5
    mission.current_pose.transform.translation.y = 0.3
    mission.door_via_point_tolerance = 0.25
    mission._publish_active_door_via_point = Mock()

    CoverageMissionNode._update_door_via_progress(mission)

    assert mission.door_waypoint_index == 0
    mission._publish_active_door_via_point.assert_not_called()


def test_success_of_single_final_goal_starts_landing_alignment():
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.goal_token = 1
    mission.state = CoverageMissionNode.DOOR_TRAVERSE
    mission.nav_retry_count = 0
    mission.nav_retry_at = 0.0
    mission.door_waypoints = [(1.0, 0.3)]
    mission.door_waypoint_index = 1
    mission._begin_final_landing = Mock()
    future = Mock()
    future.result.return_value.status = 4

    with patch.object(CoverageMissionNode, 'get_logger', return_value=Mock()):
        CoverageMissionNode._goal_result(mission, future, 1, 'door_route')

    mission._begin_final_landing.assert_called_once()


def test_final_goal_success_does_not_land_if_door_via_point_was_skipped():
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.goal_token = 1
    mission.state = CoverageMissionNode.DOOR_TRAVERSE
    mission.nav_retry_count = 0
    mission.nav_retry_at = 0.0
    mission.door_waypoints = [(1.0, 0.3), (2.0, 0.3)]
    mission.door_waypoint_index = 1
    mission._begin_final_landing = Mock()
    mission._schedule_nav_retry = Mock()
    future = Mock()
    future.result.return_value.status = 4

    with patch.object(CoverageMissionNode, 'get_logger', return_value=Mock()):
        CoverageMissionNode._goal_result(mission, future, 1, 'door_route')

    mission._begin_final_landing.assert_not_called()
    mission._schedule_nav_retry.assert_called_once_with(
        'door_route', 'landing goal reached before all door via-points passed')


def test_landing_approach_goal_uses_separately_configured_point():
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.fixed_heading_yaw = 0.5
    mission.landing_xy = (4.2, -1.3)
    mission._send_nav_goal = Mock()

    CoverageMissionNode._send_landing_goal(mission)

    mission._send_nav_goal.assert_called_once_with(
        'landing_approach', 4.2, -1.3, 0.5)
