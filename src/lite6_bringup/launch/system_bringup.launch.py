import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    desc_pkg = get_package_share_directory('lite6_description')
    hw_pkg = get_package_share_directory('lite6_hardware')

    urdf_file = os.path.join(desc_pkg, 'urdf', 'lite6.urdf')
    with open(urdf_file, 'r') as infp: 
        robot_desc = infp.read()
    
    controller_config = os.path.join(hw_pkg, 'config', 'ros2_controllers.yaml')

    # Shadow robot URDF generation: We will create a modified version of the original URDF for the shadow robot. 
    # This involves renaming the links to avoid conflicts with the main robot's links. 
    # The shadow robot will have its own set of links prefixed with "shadow_".
    shadow_desc = robot_desc
    links_to_rename = ['link_base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link_eef']
    for l in links_to_rename:
        shadow_desc = shadow_desc.replace(f'name="{l}"', f'name="shadow_{l}"')
        shadow_desc = shadow_desc.replace(f'link="{l}"', f'link="shadow_{l}"')
        
        # Replace the color of the shadow robot for visualization purposes in RViz
        shadow_desc = shadow_desc.replace(
            '<color rgba="1.0 1.0 1.0 1.0"/>', 
            '<color rgba="0.0 0.8 1.0 1.0"/>'
        )

        shadow_desc = shadow_desc.replace(
            '<color rgba="0.753 0.753 0.753 1.0"/>', 
            '<color rgba="0.0 0.4 0.8 1.0"/>'
        )

    # Core nodes for the robot system bringup
    control_node = Node(
        package="controller_manager", 
        executable="ros2_control_node", 
        parameters=[{'robot_description': robot_desc}, controller_config], 
        output="screen"
    )
    
    rsp_node = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        parameters=[{'robot_description': robot_desc}, {'publish_frequency': 200.0}]
    )

    # RViz and the shadow robot state publisher nodes
    rviz_config_file = os.path.join(desc_pkg, 'rviz', 'clean.rviz')
    rviz_node = Node(
        package='rviz2', 
        executable='rviz2', 
        name='rviz2', 
        output='screen', 
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )

    shadow_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='shadow_robot_state_publisher',
        parameters=[{
            'robot_description': shadow_desc,
            'publish_frequency': 200.0
        }],
        remappings=[
            ('/joint_states', '/shadow/joint_states'),
            ('/robot_description', '/shadow_robot_description')
        ]
    )

    shadow_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'link_base', 'shadow_link_base'],
        output='screen'
    )

    # Load controllers: Joint State Broadcaster, CTC Controller, and JTC Controller
    jsb_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    ctc_spawner = Node(package="controller_manager", executable="spawner", arguments=["ctc_controller"])
    jtc_spawner = Node(package="controller_manager", executable="spawner", arguments=["lite6_arm_controller"])

    delay_jtc_after_ctc = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=ctc_spawner, on_exit=[jtc_spawner])
    )

    # Planner and HMI nodes
    planner_node = Node(
        package='lite6_planner', 
        executable='lite6_planner_node', 
        name='lite6_industrial_planner',
        output='screen'
    )

    hmi_node = Node(
        package='lite6_hmi', 
        executable='gui_main', 
        output='screen'
    )

    delay_upper_layer_after_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=jtc_spawner, on_exit=[planner_node, hmi_node])
    )

    return LaunchDescription([
        # 1. Basic nodes: control, state publisher, RViz, shadow robot state publisher, and controller spawners
        control_node, rsp_node, rviz_node, shadow_rsp_node, shadow_tf_node, jsb_spawner, ctc_spawner,
        # 2. Mount the JTC controller after the CTC controller has been spawned, and then launch the planner and HMI nodes after the JTC controller is ready
        delay_jtc_after_ctc, delay_upper_layer_after_jtc
    ])