"""선박 블록 내부 시뮬레이션 실행.

    ros2 launch sp_gazebo sim.launch.py
    ros2 launch sp_gazebo sim.launch.py world:=ship_block_full
    ros2 launch sp_gazebo sim.launch.py gui:=false rviz:=false      # 헤드리스 배치실행

로봇은 도면 `모바일 로봇 ASSY.stp` (6안) 모델이다. 가운데 바퀴부가 론지 위를
달리고 양 끝단부는 바닥을 딛는다. 월드만 보려면 `robot:=false`.

    # 론지 위에 올려놓기 — y 는 론지 중심, z 는 론지 플랜지 상면 + 여유
    ros2 launch sp_gazebo sim.launch.py world:=ship_block_cad_world \
        x:=-1.97 y:=-0.455 z:=0.53

조종:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sp_gazebo = Path(get_package_share_directory("sp_gazebo"))
    sp_desc = Path(get_package_share_directory("sp_description"))
    ros_gz_sim = Path(get_package_share_directory("ros_gz_sim"))

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    robot = LaunchConfiguration("robot")
    publish_gz_tf = LaunchConfiguration("publish_gz_tf")
    spawn_x = LaunchConfiguration("x")
    spawn_y = LaunchConfiguration("y")
    spawn_z = LaunchConfiguration("z")
    spawn_yaw = LaunchConfiguration("yaw")
    render_engine = LaunchConfiguration("render_engine")
    software_render = LaunchConfiguration("software_render")
    gui_config = LaunchConfiguration("gui_config")
    arm_fixed = LaunchConfiguration("arm_fixed")

    # STEP 도면에서 변환한 모델(models/)을 gz 가 model:// 로 찾을 수 있게 한다.
    # step_to_gazebo.py 가 만든 월드는 <include><uri>model://...</uri></include> 를 쓴다.
    gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", str(sp_gazebo / "models"))

    # ⚠ URDF 의 `package://sp_description/meshes/*.stl` 은 gz 안에서
    #   `model://sp_description/...` 로 바뀐다. 그걸 풀려면 **패키지 share 의 부모**가
    #   리소스 경로에 있어야 한다. 없으면 로봇이 스폰은 되는데 아무것도 안 보인다
    #   ("Unable to find file with URI [model://sp_description/...]").
    gz_resource_path_desc = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", str(sp_desc.parent))

    args = [
        DeclareLaunchArgument(
            "world", default_value="ship_block_mockup",
            description="worlds/ 안의 SDF 이름 (확장자 제외). "
                        "파라메트릭 월드는 scripts/gen_world.py, "
                        "STEP 도면 변환 월드는 scripts/step_to_gazebo.py 로 만든다."),
        DeclareLaunchArgument("gui", default_value="true",
                              description="Gazebo GUI 표시 여부"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="RViz2 실행 여부"),
        DeclareLaunchArgument(
            "robot", default_value="true",
            description="로봇을 스폰할지. false 면 월드만 뜬다 — 블록 형상만 "
                        "볼 때 쓴다."),
        DeclareLaunchArgument(
            "publish_gz_tf", default_value="true",
            description="Gazebo DiffDrive 의 odom→base_footprint TF 를 ROS 로 브리지. "
                        "EKF(robot_localization)를 붙이면 반드시 false 로 둘 것 — "
                        "같은 TF 를 두 노드가 발행하면 충돌한다."),
        DeclareLaunchArgument("x", default_value="-3.0", description="스폰 X [m]"),
        DeclareLaunchArgument("y", default_value="0.0", description="스폰 Y [m]"),
        DeclareLaunchArgument("z", default_value="0.15",
                              description="스폰 Z [m]. 블록 바닥판 위에 올리려면 "
                                          "판 두께 + 여유만큼 띄울 것."),
        DeclareLaunchArgument("yaw", default_value="0.0", description="스폰 yaw [rad]"),
        DeclareLaunchArgument(
            "arm_fixed", default_value="true",
            description="팔 6축을 접은 자세로 굳힐지. true(기본) 면 관절·제어기가 "
                        "사라져 가볍고 과도상태가 없다 — 주행 드리프트 실험용. "
                        "팔 자세를 바꾸려면 false (그때만 /arm_j1 .. /arm_j6 가 생긴다)."),
        DeclareLaunchArgument(
            "render_engine", default_value="ogre2",
            description="렌더 엔진: ogre2(기본) 또는 ogre."),
        DeclareLaunchArgument(
            "gui_config",
            default_value=str(sp_gazebo / "config" / "gui_cad_world.config"),
            description="gz sim GUI 설정 파일. 기본값은 카메라를 **블록 안쪽**에 두는 "
                        "설정이다 — gz 기본 시점(-6 0 6)은 블록 바깥이라 사방이 막힌 "
                        "선체 블록에서는 회색 덩어리만 보인다."),
        DeclareLaunchArgument(
            "software_render", default_value="false",
            description="GPU 를 안 쓰고 CPU(llvmpipe)로 그린다. "
                        "VMware 게스트에서 화면이 깨지거나 깜빡이면 true 로 둘 것 — "
                        "SVGA3D 드라이버 버그를 통째로 우회한다. 대신 느리다."),
    ]

    world_path = PathJoinSubstitution([str(sp_gazebo), "worlds",
                                       [world, ".sdf"]])

    # xacro → URDF 문자열.
    # robot:=false 면 이 Node 자체가 안 뜨므로 xacro 도 실행되지 않는다.
    robot_description = ParameterValue(
        Command(["xacro ", str(sp_desc / "urdf" / "crawler.urdf.xacro"),
                 " arm_fixed:=", arm_fixed]),
        value_type=str,
    )

    # 로봇 관련 노드는 전부 robot 인자로 켜고 끈다.
    # 로봇이 없으면 robot_state_publisher/spawn 은 물론 브리지도 의미가 없다
    # (/cmd_vel·/odom·/imu 는 전부 로봇이 내는 토픽이다).
    with_robot = IfCondition(robot)

    # -r: 즉시 실행, -v4: 로그 레벨, -s: 서버만(GUI 없음)
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_sim / "launch" / "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": [world_path, " -r -v4 --render-engine ", render_engine,
                        " --gui-config ", gui_config],
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(gui),
    )

    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_sim / "launch" / "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": [world_path, " -r -v4 -s --headless-rendering "
                        "--render-engine ", render_engine],
            "on_exit_shutdown": "true",
        }.items(),
        condition=UnlessCondition(gui),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
        condition=with_robot,
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "crawler",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw,
        ],
        condition=IfCondition(robot),
    )

    # /clock 은 로봇과 무관한 월드 토픽이라 항상 브리지한다.
    # 로봇 브리지에 같이 묶으면 robot:=false 일 때 시뮬 시각이 안 나와서
    # use_sim_time 을 쓰는 노드가 전부 멈춘다.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_clock_bridge",
        output="screen",
        parameters=[{
            "config_file": str(sp_gazebo / "config" / "bridge_clock.yaml"),
        }],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{
            "config_file": str(sp_gazebo / "config" / "bridge.yaml"),
            "use_sim_time": True,
        }],
        condition=IfCondition(robot),
    )

    # DiffDrive TF 브리지 (EKF 도입 전 임시)
    tf_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_tf_bridge",
        output="screen",
        arguments=["/tf_gz@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
        remappings=[("/tf_gz", "/tf")],
        parameters=[{"use_sim_time": True}],
        # 로봇이 있고 publish_gz_tf 도 켜져 있을 때만
        condition=IfCondition(PythonExpression(
            ["'", robot, "' == 'true' and '", publish_gz_tf, "' == 'true'"])),
    )

    # 시저 링키지 종동 구동 — 상하축 행정을 보고 X자 바 각도를 풀어 넣는다.
    # 시저는 닫힌 루프라 URDF 로 표현할 수 없고, dartsim 이 mimic 관절을 지원하지
    # 않아서 이 노드가 필요하다 (scissor_driver.py 의 첫 주석 참조).
    # 안 떠도 시뮬은 정상 동작한다 — X자가 접힌 채 있을 뿐이다.
    scissor_driver = Node(
        package="sp_gazebo",
        executable="scissor_driver.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=with_robot,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(sp_gazebo / "config" / "sim.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
    )

    # VMware 게스트의 SVGA3D 드라이버는 ogre2 에서 화면이 깨지고 ogre 에서는 멈춘다.
    # CPU 렌더링(llvmpipe)으로 내리면 정상 동작한다. gz 프로세스에만 적용된다.
    force_software_gl = SetEnvironmentVariable(
        "LIBGL_ALWAYS_SOFTWARE", "1", condition=IfCondition(software_render))

    return LaunchDescription(args + [
        force_software_gl,
        gz_resource_path,
        gz_resource_path_desc,
        gz,
        gz_headless,
        clock_bridge,
        robot_state_publisher,
        spawn,
        bridge,
        tf_bridge,
        scissor_driver,
        rviz_node,
    ])
