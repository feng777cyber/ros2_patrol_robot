import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # get the path to the URDF file
    urdf_package_path=get_package_share_directory("fengbot_description")
    default_xacro_path=os.path.join(urdf_package_path,"URDF","fengbot","fengbot.urdf.xacro")
    default_config_path=os.path.join(urdf_package_path,"config","display_robot_model.rviz")
    default_gazebo_world_path=os.path.join(urdf_package_path,"world","custom_room.world")
    #set param for quick modification
    action_declare_arg_mode_path=launch.actions.DeclareLaunchArgument(
        name="model", default_value=str(default_xacro_path),description="XACRO file path"
        
    )
    #set param for quick modification ,use param for robot state publisher
    substitutions_command_result=launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
    robot_description_value=launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result, value_type=str)
    
    
    
    
    action_load_joint_state_controller=launch.actions.ExecuteProcess(
        # cmd=['ros2','control','load_controller','fengbot_joint_state_broadcaster','--set-state','active'],
        cmd='ros2 control load_controller fengbot_joint_state_broadcaster --set-state active'.split(' '),
        output='screen'
    )
    action_load_effort_controller=launch.actions.ExecuteProcess(
        cmd=['ros2','control','load_controller','fengbot_effort_controller','--set-state','active'],
        output='screen'
    )
    
    action_load_diff_drive_controller=launch.actions.ExecuteProcess(
        cmd=['ros2','control','load_controller','fengbot_diff_drive_controller','--set-state','active'],
        output='screen'
    )
    
    
    action_launch_gazebo=launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory("gazebo_ros"),'/launch','/gazebo.launch.py'
        ]),
        launch_arguments=[
            ('world',default_gazebo_world_path),('verbose','true')
    ]
    )
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity','fengbot'],
    )
    
    

    action_robot_state_publisher=launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]
        #map key and value use {:}dict format
    )
    


    return launch.LaunchDescription([   
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_controller]
            )
        ),
        launch.actions.RegisterEventHandler(
                    event_handler=launch.event_handlers.OnProcessExit(
                        target_action=action_load_joint_state_controller,
                        on_exit=[action_load_diff_drive_controller]
                    )
                )
    ])