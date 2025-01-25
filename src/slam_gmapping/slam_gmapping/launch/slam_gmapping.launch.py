from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    param_substitutions = {
        'scan_topic': 'scan',
        'base_frame': 'base_link',
        'odom_frame': 'odom',
        'map_frame': 'map',
        'map_update_interval': 3.0,
        'maxUrange': 10.0,
        'sigma': 0.05,
        'kernelSize': 1,
        'lstep': 0.05,
        'astep': 0.05,
        'iterations': 5,
        'lsigma': 0.075,
        'ogain': 3.0,
        'lskip': 0,
        'srr': 0.1,
        'srt': 0.2,
        'str': 0.1,
        'stt': 0.2,
        'linearUpdate': 0.3,
        'angularUpdate': 3.14,
        'temporalUpdate': 5.0,
        'resampleThreshold': 0.5,
        'particles': 30,
        'xmin': -15.0,
        'ymin': -15.0,
        'xmax': 15.0,
        'ymax': 15.0,
        'delta': 0.025,
        'llsamplerange': 0.01,
        'llsamplestep': 0.01,
        'lasamplerange': 0.005,
        'lasamplestep': 0.005,
        #'use_sim_time': True
    }
    return LaunchDescription([
        launch_ros.actions.Node(
            package='slam_gmapping', 
            node_executable='slam_gmapping', 
            parameters=[param_substitutions],
            # arguments=['--ros-args','--log-level','debug'],
            output='screen'),
    ])
