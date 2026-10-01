"""Lock the production bridge-to-C++ alias contract to approved entries."""

from pathlib import Path

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
import yaml


ALIAS_PATH = (
    Path(__file__).resolve().parents[2]
    / "irc_step_motion_executor"
    / "config"
    / "motion_aliases.yaml"
)
RUNTIME_CATALOG_PATH = (
    Path(__file__).resolve().parents[3]
    / "artifacts"
    / "robot_motions_runtime.json"
)


def test_production_alias_catalog_contains_only_approved_aliases():
    payload = yaml.safe_load(ALIAS_PATH.read_text(encoding="utf-8"))
    assert payload == {
        "motion_aliases": {
            "sdk_pickup": "공잡기리그랩까지 실전",
            "sdk_hurdle": "허들넘기 실전",
            "sdk_forward_4": "전진45도(4회)",
            "line_forward_2": "전진45도(2회)",
            "line_forward_4": "전진45도(4회)",
            "line_forward_6": "전진45도(6회)",
            "line_forward_8": "전진45도(8회)",
            "line_forward_10": "전진45도(10회)",
            "line_turn_left_4": "좌회전실실전(4회)",
            "line_turn_left_6": "좌회전실실전(6회)",
            "line_turn_left_8": "좌회전실실전(8회)",
            "line_turn_left_10": "좌회전실실전(10회)",
            "line_turn_left_12": "좌회전실실전(12회)",
            "line_turn_left_15": "좌회전실실전(15회)",
            "line_turn_right_2": "우회전실실전(4회)",
            "line_turn_right_4": "우회전실실전(6회)",
            "line_turn_right_6": "우회전실실전(8회)",
            "line_turn_right_8": "우회전실실전(10회)",
            "line_turn_right_10": "우회전실실전(12회)",
            "line_turn_right_large": "우회전실실전(15회)",
            "line_recovery_left_4": "라인복귀좌회전(4회)",
            "line_recovery_left_5": "라인복귀좌회전(5회)",
            "line_recovery_left_6": "라인복귀좌회전(6회)",
            "line_recovery_left_7": "라인복귀좌회전(7회)",
            "line_recovery_left_8": "라인보귀좌회전(8번)",
            "line_recovery_right_4": "라인복귀우회전(4회)",
            "line_recovery_right_5": "라인복귀우회전(5회)",
            "line_recovery_right_6": "라인복귀우회전(6회)",
            "line_recovery_right_7": "라인복귀우회전(7회)",
            "line_recovery_right_8": "라인보귀우회전(8번)",
            "stationary_turn_left": "제자리좌회전(6번)",
            "stationary_turn_right": "제자리우회전(9회)",
            "sdk_turn_right_15": "우회전실실전(15회)",
            "ball_camera_down_forward_2": "전진카메라내린거(2회)",
            "ball_camera_down_forward_4": "전진카메라내린거(4회)",
            "ball_camera_down_forward_6": (
                "전진카메라내린거(6회)"
            ),
            "ball_camera_down_forward_8": "전진카메라내린거(8회)",
            "ball_general_fine_forward_8": "미세실전(8회)",
            "pickup_fine_forward_0": "미세실전(0도)",
            "pickup_crab_right_0": "오옆꽃게(0도)",
            "pickup_crab_left_0": "왼쪽게걸음(0도)",
            "pickup_pre_backward_camera_down": (
                "공잡기전후진카메라내린거"
            ),
            "pickup_grasp_check_pose": "공확인자세",
            "pickup_retreat_3": "후진실전(3회)",
            "pickup_first_turn_right_9": "제자리우회전(9회)",
            "pickup_first_to_right_back_camera_45": (
                "왼뒤에서 오뒤로(카메라45도)"
            ),
            **{
                f"pickup_camera_down_turn_left_{count}": (
                    f"제좌카메라내린거({count}회)"
                )
                for count in range(1, 10)
            },
            **{
                f"pickup_camera_down_turn_right_{count}": (
                    f"제우카메라내린거({count}회)"
                )
                for count in range(1, 10)
            },
            "post_ball_forward_4": "전진45도(4회)",
            "post_ball_forward_8": "전진45도(8회)",
            "post_ball_camera_90": "오뒤카메라90도",
            "goal_camera_90_forward_2": "전진카메라90도(2회)",
            "goal_camera_90_forward_4": "전진카메라90도(4회)",
            "goal_camera_90_forward_6": "전진카메라90도(6회)",
            "goal_camera_90_forward_8": "전진카메라90도(8회)",
            "goal_camera_90_crab_right": "오옆꽃게(90도)",
            "goal_camera_90_crab_left": "왼쪽게걸음(90도)",
            **{
                f"post_ball_line_turn_right_{count}": (
                    f"제자리우회전({count}회)"
                )
                for count in range(1, 10)
            },
            "post_ball_line_turn_left_2": "제자리좌회전(2번)",
            "post_ball_line_turn_left_3": "제자리좌회전(3회)",
            "post_ball_line_turn_left_4": "제자리좌회전(4회)",
            "post_ball_line_turn_left_5": "제자리좌회전(5번)",
            "post_ball_line_turn_left_6": "제자리좌회전(6번)",
            **{
                f"goal_camera_90_turn_right_{count}": (
                    f"제자리우회전90도({count}회)"
                )
                for count in range(1, 10)
            },
            **{
                f"goal_camera_90_turn_left_{count}": (
                    f"제자리좌회전90도({count}회)"
                )
                for count in range(1, 7)
            },
            "goal_shot": "골넣기",
            "sdk_default_to_right_back": "기본자세에서 오뒤로",
            "pickup": "공잡기리그랩까지 실전",
            "hurdle": "허들넘기 실전",
            "forward": "전진45도(8회)",
        }
    }


def test_deprecated_left_turn_is_not_a_production_alias_target():
    aliases = yaml.safe_load(
        ALIAS_PATH.read_text(encoding="utf-8")
    )["motion_aliases"]
    forbidden = {
        "좌회전실전(9회)",
        "좌회전실전(2회)",
        "좌회전 실전(13회)",
    }
    assert forbidden.isdisjoint(aliases.values())


def test_pickup_does_not_use_intermediate_default_pose():
    aliases = yaml.safe_load(
        ALIAS_PATH.read_text(encoding="utf-8")
    )["motion_aliases"]

    assert "pickup_left_back_to_default_90" not in aliases
    assert "왼뒤에서기본자세(90도)" not in aliases.values()
    assert "왼뒤에서기본(0도)" not in aliases.values()
    assert aliases["goal_camera_90_turn_left_1"] == (
        "제자리좌회전90도(1회)"
    )
    assert aliases["goal_camera_90_turn_left_6"] == (
        "제자리좌회전90도(6회)"
    )


def test_every_production_bridge_motion_id_has_an_approved_alias():
    aliases = yaml.safe_load(
        ALIAS_PATH.read_text(encoding="utf-8")
    )["motion_aliases"]
    assert set(MotionCommandBridgeNode.ACTION_TO_MOTION_ID.values()) <= set(
        aliases
    )
    pickup_motion_ids = {
        *MotionCommandBridgeNode.PICKUP_CAMERA_DOWN_MOTION_IDS.values(),
        *MotionCommandBridgeNode.PICKUP_MOTION_TAIL,
        *MotionCommandBridgeNode.POST_BALL_GOAL_TRANSITION_SEQUENCE,
        "stationary_turn_left",
        "pickup_first_turn_right_9",
        "pickup_first_to_right_back_camera_45",
        *MotionCommandBridgeNode.PICKUP_FINE_ALIGN_MOTION_IDS.values(),
    }
    pickup_motion_ids.discard(None)
    pickup_motion_ids.discard(MotionCommandBridgeNode.FINE_ALIGN_MARKER)
    assert pickup_motion_ids <= set(aliases)


def test_counted_turn_aliases_match_family_direction_and_repeat_count():
    aliases = yaml.safe_load(
        ALIAS_PATH.read_text(encoding="utf-8")
    )["motion_aliases"]
    catalog = yaml.safe_load(
        RUNTIME_CATALOG_PATH.read_text(encoding="utf-8")
    )["motions"]
    motions_by_name = {motion["name"]: motion for motion in catalog}

    expected = {
        "stationary_turn_left": ("제자리좌회전(6번)", 6),
        "stationary_turn_right": ("제자리우회전(9회)", 9),
        "pickup_first_turn_right_9": ("제자리우회전(9회)", 9),
        **{
            f"post_ball_line_turn_right_{count}": (
                f"제자리우회전({count}회)",
                count,
            )
            for count in range(1, 10)
        },
        "post_ball_line_turn_left_2": ("제자리좌회전(2번)", 2),
        "post_ball_line_turn_left_3": ("제자리좌회전(3회)", 3),
        "post_ball_line_turn_left_4": ("제자리좌회전(4회)", 4),
        "post_ball_line_turn_left_6": ("제자리좌회전(6번)", 6),
        **{
            f"goal_camera_90_turn_right_{count}": (
                f"제자리우회전90도({count}회)",
                count,
            )
            for count in range(1, 10)
        },
        **{
            f"goal_camera_90_turn_left_{count}": (
                f"제자리좌회전90도({count}회)",
                count,
            )
            for count in range(1, 7)
        },
        **{
            f"pickup_camera_down_turn_left_{count}": (
                f"제좌카메라내린거({count}회)",
                count,
            )
            for count in range(1, 10)
        },
        **{
            f"pickup_camera_down_turn_right_{count}": (
                f"제우카메라내린거({count}회)",
                count,
            )
            for count in range(1, 10)
        },
    }

    for motion_id, (exact_name, repeat_count) in expected.items():
        assert aliases[motion_id] == exact_name
        assert motions_by_name[exact_name]["repeat_count"] == repeat_count

    assert motions_by_name["제자리좌회전90도(6회)"][
        "playback_speed"
    ] == 0.95


def test_production_motions_use_four_degree_final_tolerance():
    aliases = yaml.safe_load(
        ALIAS_PATH.read_text(encoding="utf-8")
    )["motion_aliases"]
    catalog = yaml.safe_load(
        RUNTIME_CATALOG_PATH.read_text(encoding="utf-8")
    )["motions"]
    motions_by_name = {motion["name"]: motion for motion in catalog}

    assert set(aliases.values()) <= set(motions_by_name)
    for motion in catalog:
        assert motion["completion"][
            "position_tolerance_deg"
        ] == 4.0

    production_tolerance = motions_by_name[
        "공잡기리그랩까지 실전"
    ]["completion"]["position_tolerance_deg"]
    assert 3.779274 <= production_tolerance
    assert 4.2 > production_tolerance


def test_latest_pickup_and_grasp_check_motion_definitions_are_mapped():
    aliases = yaml.safe_load(
        ALIAS_PATH.read_text(encoding="utf-8")
    )["motion_aliases"]
    motions = yaml.safe_load(
        RUNTIME_CATALOG_PATH.read_text(encoding="utf-8")
    )["motions"]
    by_name = {motion["name"]: motion for motion in motions}

    expected = {
        "미세실전(8회)": (7045, 5, 0.95, "미세오들400", "미세오뒤400"),
        "미세실전(0도)": (
            7045,
            5,
            0.95,
            "미세오들(0도)",
            "미세오뒤(0도)",
        ),
        "공잡기리그랩까지 실전": (
            8000,
            1,
            1.05,
            "음",
            "기본자세4",
        ),
        "공확인자세": (7045, 1, 1.0, "공확인자세", "공확인자세"),
    }
    for name, metadata in expected.items():
        motion = by_name[name]
        assert (
            motion["max_seq_ms"],
            motion["repeat_count"],
            motion["playback_speed"],
            motion["start_pose"],
            motion["end_pose"],
        ) == metadata

    check_pose = by_name[aliases["pickup_grasp_check_pose"]]
    assert len(check_pose["frames"]) == 1
    assert check_pose["frames"][0]["name"] == "공확인자세"
    assert check_pose["frames"][0]["time_ms"] == 400
