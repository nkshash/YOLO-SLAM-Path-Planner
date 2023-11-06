from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Set to true to use simulation (Gazebo) clock'
        ),
        
        DeclareLaunchArgument(
            'deskewing', default_value='true',
            description='Enable lidar deskewing'
        ),
          
        Node(
            package='rtabmap_ros', executable='icp_odometry', output='screen',
            parameters=[
                {
                    'frame_id':'base_link',
                    'odom_frame_id':'odom',
                    'wait_for_transform':0.3,
                    'expected_update_rate':15.0,
                    'deskewing':LaunchConfiguration('deskewing'),
                    'use_sim_time':LaunchConfiguration('use_sim_time'),
                }
            ],
            remappings=[
                ('scan_cloud', '/filtered_velodyne_points')
            ],
            arguments=[
                'Icp/PointToPlane', 'true',
                'Icp/Iterations', '10',
                'Icp/VoxelSize', '0.1',
                'Icp/Epsilon', '0.001',
                'Icp/PointToPlaneK', '20',
                'Icp/PointToPlaneRadius', '0',
                'Icp/MaxTranslation', '2',
                'Icp/MaxCorrespondenceDistance', '1',
                'Icp/Strategy', '1',
                'Icp/OutlierRatio', '0.7',
                'Icp/CorrespondenceRatio', '0.01',
                'Odom/ScanKeyFrameThr', '0.6',
                'OdomF2M/ScanSubtractRadius', '0.1',
                'OdomF2M/ScanMaxSize', '15000',
                'OdomF2M/BundleAdjustment', 'false',
            ]
        ),
        Node(
            package='rtabmap_ros', executable='point_cloud_assembler', output='screen',
            parameters=[
                {
                    'max_clouds':10,
                    'fixed_frame_id':'',
                    'use_sim_time':LaunchConfiguration('use_sim_time'),
                }
            ],
            remappings=[
                ('cloud', 'odom_filtered_input_scan')
            ]
        ),
            
        Node(
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=[
                {
                    'frame_id':'velodyne',
                    'subscribe_depth':False,
                    'subscribe_rgb':False,
                    'subscribe_scan_cloud':True,
                    'approx_sync':True,
                    'wait_for_transform':0.3,
                    'use_sim_time':LaunchConfiguration('use_sim_time'),
                }
            ],
            remappings=[
                ('scan_cloud', 'assembled_cloud')
            ],
            arguments=[
                '-d',
                'RGBD/ProximityMaxGraphDepth', '0',
                'RGBD/ProximityPathMaxNeighbors', '1',
                'RGBD/AngularUpdate', '0.05',
                'RGBD/LinearUpdate', '0.05',
                'RGBD/CreateOccupancyGrid', 'false',
                'Mem/NotLinkedNodesKept', 'false',
                'Mem/STMSize', '30',
                'Mem/LaserScanNormalK', '20',
                'Reg/Strategy', '1',
                'Icp/VoxelSize', '0.1',
                'Icp/PointToPlaneK', '20',
                'Icp/PointToPlaneRadius', '0',
                'Icp/PointToPlane', 'true',
                'Icp/Iterations', '10',
                'Icp/Epsilon', '0.001',
                'Icp/MaxTranslation', '3',
                'Icp/MaxCorrespondenceDistance', '1',
                'Icp/Strategy', '1',
                'Icp/OutlierRatio', '0.7',
                'Icp/CorrespondenceRatio', '0.2',
            ]
        ), 
     
        Node(
            package='rtabmap_ros', executable='rtabmapviz', output='screen',
            parameters=[
                {
                    'frame_id':'velodyne',
                    'odom_frame_id':'odom',
                    'subscribe_odom_info':True,
                    'subscribe_scan_cloud':True,
                    'approx_sync':True,
                    'use_sim_time':LaunchConfiguration('use_sim_time'),
                }
            ],
            remappings=[
                ('scan_cloud', 'odom_filtered_input_scan')
            ]
        ),
    ])