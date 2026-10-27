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
    mission.record_targets_during_search = False
    mission.search_route_complete = False
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


def test_target_priority_matches_configured_hierarchy():
    priority = CoverageMissionNode._target_priority

    assert priority(0, "红十字") > priority(3, "车")
    assert priority(3, "车") == priority(5, "坦克")
    assert priority(5, "坦克") > priority(6, "桥")
    assert priority(6, "桥") > priority(4, "堡垒")
    assert priority(4, "堡垒") > priority(2, "营帐")
    assert priority(1, "黑H") is None


def test_recorded_targets_are_delivered_by_priority_and_red_cross_can_be_skipped():
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.delivery_count = 0
    mission.max_deliveries = 3
    mission.deliver_red_cross = False
    mission.recorded_targets = [
        (1.0, 0.0, 0.0, 2, "营帐", 0.99),
        (2.0, 0.0, 0.0, 6, "桥", 0.80),
        (3.0, 0.0, 0.0, 0, "红十字", 0.99),
        (4.0, 0.0, 0.0, 3, "车", 0.70),
    ]
    mission.drop_offsets = [0.0] * 6
    mission.drop_commands = [2, 3, 4]
    mission.state = CoverageMissionNode.SEARCHING
    mission.nav_state_pub = type("Publisher", (), {"publish": lambda *_: None})()
    mission.get_logger = lambda: type(
        "Logger", (), {"warning": lambda *_: None})()
    mission._send_drop_goal = lambda: setattr(mission, "sent_drop", mission.active_target)
    mission._begin_door_leg = lambda: setattr(mission, "went_to_door", True)
    mission._begin_clear_remaining_cargo = lambda: setattr(mission, "cleared_here", True)

    CoverageMissionNode._dispatch_next_recorded_target_or_clear(mission)

    assert mission.active_target[4] == "车"
    assert mission.sent_drop[4] == "车"
    assert any(item[3] == 0 for item in mission.recorded_targets)
    assert not hasattr(mission, "cleared_here")


def test_exhausted_recorded_targets_clear_remaining_cargo_in_place():
    mission = CoverageMissionNode.__new__(CoverageMissionNode)
    mission.delivery_count = 1
    mission.max_deliveries = 3
    mission.deliver_red_cross = True
    mission.recorded_targets = []
    mission._begin_clear_remaining_cargo = lambda: setattr(mission, "cleared_here", True)
    mission._begin_door_leg = lambda: setattr(mission, "went_to_door", True)

    CoverageMissionNode._dispatch_next_recorded_target_or_clear(mission)

    assert mission.cleared_here
    assert not hasattr(mission, "went_to_door")
