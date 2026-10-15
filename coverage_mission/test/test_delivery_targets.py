"""Regression tests for repeated targets of the same class at distinct sites."""

from coverage_mission.coverage_mission_node import CoverageMissionNode


def test_delivered_target_deduplication_is_by_position_not_class():
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.delivered_targets = [(2.0, -1.0)]
    mission.delivered_target_duplicate_radius = 0.5

    assert mission._is_delivered_target(2.2, -1.1)
    assert not mission._is_delivered_target(3.0, -1.0)


def test_search_continues_to_next_waypoint_after_a_delivery():
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.next_index = 1
    mission.route = [(1.0, 0.0, 1.4, 0.0), (2.0, 0.0, 1.4, 0.0)]
    mission._send_nav_goal = lambda *args: setattr(mission, "sent_goal", args)
    mission._begin_door_leg = lambda: setattr(mission, "went_to_door", True)

    CoverageMissionNode._send_search_goal(mission)

    assert mission.sent_goal == ("search", 2.0, 0.0, 0.0)
    assert not hasattr(mission, "went_to_door")


def test_delivery_resumes_interrupted_search_waypoint():
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.paused_search_goal = ("search", 2, 2.0, 0.0)
    mission.next_index = 2  # Already advanced when waypoint 2 was dispatched.
    mission.route = [(1.0, 0.0, 1.4, 0.0), (2.0, 0.0, 1.4, 0.0),
                     (3.0, 0.0, 1.4, 0.0)]
    mission.delivery_count = 1
    mission.max_deliveries = 3
    mission.state = CoverageMissionNode.DROP_ASCEND

    class Publisher:
        def publish(self, _message):
            pass

    class Logger:
        def warning(self, _message):
            pass

        def info(self, _message):
            pass

    mission.nav_state_pub = Publisher()
    mission.get_logger = lambda: Logger()
    mission._send_nav_goal = lambda *args: setattr(mission, "sent_goal", args)

    CoverageMissionNode._resume_search_after_delivery(mission)

    assert mission.next_index == 1
    assert mission.sent_goal == ("search", 2.0, 0.0, 0.0)
    assert mission.paused_search_goal is None
