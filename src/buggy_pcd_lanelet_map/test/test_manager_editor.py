"""Buggy Manager app: the editor logic (no GUI, no ROS)."""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import manager_app  # noqa: E402
from buggy_pcd_lanelet_map import mission  # noqa: E402


def make_editor():
    d = tempfile.mkdtemp()
    os.makedirs(os.path.join(d, 'm'))
    os.environ['BUGGY_DATA_DIR'] = d
    return manager_app.Editor(os.path.join(d, 'm'))


def test_home_is_optional_and_a_b_come_from_goal_clicks():
    e = make_editor()
    assert e.next_free_name() == 'A', 'no fixed home is asked for'
    e.arm('A'); assert e.on_click('goal_pose', 10.0, 20.0)
    e.arm('B', wait_sec=20); assert e.on_click('goal_pose', 30.0, 40.0)
    data = mission.load(e.path)
    assert [s['name'] for s in data['stops']] == ['A', 'B']
    assert data['stops'][1]['wait_sec'] == 20.0
    assert mission.node_params(data)['stop_wait_sec'] == [10.0, 20.0]


def test_a_fixed_home_comes_from_the_pose_estimate_click():
    e = make_editor()
    assert e.arm('home')
    assert not e.on_click('goal_pose', 1, 1), 'a fixed home waits for the 2D Pose Estimate'
    assert e.on_click('initialpose', 1.0, 2.0)
    assert [s['name'] for s in e.stops()] == ['home']


def test_start_as_home_is_stored_and_passed_to_the_node():
    e = make_editor()
    e.arm('A'); e.on_click('goal_pose', 1, 1)
    e.set_start_as_home(True)
    data = mission.load(e.path)
    assert data['start_as_home'] is True and mission.node_params(data)['start_as_home'] is True


def test_click_without_arming_is_ignored_and_names_are_checked():
    e = make_editor()
    assert not e.on_click('goal_pose', 1, 1) and not e.stops()
    assert not e.arm('bad:name') and not e.arm('  ')


def test_replace_keeps_the_order_and_move_delete_work():
    e = make_editor()
    for n, x in (('home', 0), ('A', 10), ('B', 20)):
        e.arm(n); e.on_click(e.armed[1], x, 0)
    e.arm('A'); e.on_click('goal_pose', 11, 5)
    assert [s['name'] for s in e.stops()] == ['home', 'A', 'B'] and e.stops()[1]['x'] == 11
    assert e.move(2, -1) == 1 and [s['name'] for s in e.stops()] == ['home', 'B', 'A']
    e.delete(0)
    assert [s['name'] for s in e.stops()] == ['B', 'A']
    e.set_loop(False); e.set_wait(0, 7)
    data = mission.load(e.path)
    assert data['loop'] is False and data['stops'][0]['wait_sec'] == 7.0


def test_empty_mission_is_reported_not_saved_as_valid():
    e = make_editor()
    assert e.problems() == ['no stops yet']


def test_bag_info_reads_lidar_and_gnss_topics(tmp_path):
    meta = ('rosbag2_bagfile_information:\n  duration: {nanoseconds: 12000000000}\n  topics_with_message_count:\n'
            '  - {topic_metadata: {name: /velodyne_points}, message_count: 120}\n'
            '  - {topic_metadata: {name: /chcnav/devpvt}, message_count: 600}\n')
    (tmp_path / 'metadata.yaml').write_text(meta)
    info = manager_app.bag_info(str(tmp_path))
    assert info['ok'] and info['lidar'] == 120 and info['gnss'] == 600 and info['seconds'] == 12.0
    (tmp_path / 'metadata.yaml').write_text(meta.replace('/velodyne_points', '/other'))
    assert not manager_app.bag_info(str(tmp_path))['ok']
    assert not manager_app.bag_info(str(tmp_path / 'missing'))['ok']


def test_build_command_options():
    c = manager_app.build_command('/b/bag', '/o/out', True, False, 250)
    assert c[0] == 'bash' and c[1].endswith('tools/mapping/build_map_from_bag.sh')
    assert c[2:] == ['/b/bag', '/o/out', '--gnss', '--no-rviz', '--max-scans', '250']
    assert manager_app.build_command('/b/bag', '', False, True)[2:] == ['/b/bag']
    assert manager_app.build_command('/b/bag', '', False, True, 0, False)[2:] == ['/b/bag', '--no-closure']
