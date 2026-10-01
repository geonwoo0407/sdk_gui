from glob import glob
import os
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'step'

package_dir = Path(__file__).resolve().parent
model_files = ['models/best.onnx']

if (package_dir / 'models' / 'best.engine').is_file():
    model_files.append('models/best.engine')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/models',
            model_files,
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='geonwoo',
    maintainer_email='geonwoo@todo.todo',
    description='ROS 2 mission vision for the IRC STEP humanoid robot',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo26_detector=step.yolo26_detector:main',
            'ball_only_debug=step.ball_only_debug:main',
            'yolo_line_analyzer=step.yolo_line_analyzer:main',
            'ball_analyzer=step.ball_analyzer:main',
            'ball_navigation_controller=step.ball_navigation_controller:main',
            'goal_analyzer=step.goal_analyzer:main',
            'goal_navigation_controller=step.goal_navigation_controller:main',
            'hurdle_analyzer=step.hurdle_analyzer:main',
            'hurdle_navigation_controller=step.hurdle_navigation_controller:main',
            'unified_vision_node=step.unified_vision_node:main',
            'rgbd_visual_odometry=step.rgbd_visual_odometry:main',
            'line_debug_monitor=step.line_debug_monitor:main',
            'line_path_visualizer=step.line_path_visualizer:main',
            'line_navigation_controller=step.line_navigation_controller:main',
            'imu_line_pose_estimator=step.imu_line_pose_estimator:main',
            'step_motion_pose_test=step.step_motion_pose_test:main',
            'mission_state_estimator=step.mission_state_estimator:main',
            'mission_map_visualizer=step.mission_map_visualizer:main',
        ],
    },
)
