"""slip 주입 + ESKF + 평가 노드를 같이 띄운다.

    /joint_states --(휠 엔코더 기구학)--> /odom_enc
                  --(slip 주입)-->         /odom_slip
                  --(ESKF, +/imu)-->       /odom_eskf

  gz DiffDrive 의 /odom 은 쓰지 않는다. 시뮬레이터가 대신 풀어준 값이라
  엔코더가 실제로 무엇을 아는지가 가려진다. 좌우 바퀴 관절각에서 직접 푼다.

gz DiffDrive 는 이상적인 구동을 가정해 /odom 이 사실상 완벽하다. slip_injector 가
그 자리를 메워야 센서융합을 비교할 대상이 생긴다. slip:=false 로 끄면 ESKF 가
깨끗한 /odom 을 직접 본다.

    ros2 launch sp_localization eskf.launch.py
    ros2 launch sp_localization eskf.launch.py csv:=/tmp/run1.csv

Gazebo 는 따로 띄워 둘 것:
    ros2 launch sp_gazebo sim.launch.py world:=ship_block_cad_world \
        x:=-1.97 y:=-0.0325 z:=0.53

⚠ publish_tf 는 기본 false 다. Gazebo DiffDrive 도 odom->base_footprint TF 를
  내므로 둘 다 켜면 충돌한다. ESKF 쪽 TF 를 쓰려면 sim.launch.py 를
  publish_gz_tf:=false 로 띄우고 여기를 true 로 올릴 것.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    csv = LaunchConfiguration("csv")
    publish_tf = LaunchConfiguration("publish_tf")
    slip = LaunchConfiguration("slip")
    odom_topic = LaunchConfiguration("odom_topic")
    enc_gyro = LaunchConfiguration("enc_gyro")
    scale = LaunchConfiguration("scale")
    return LaunchDescription([
        DeclareLaunchArgument("csv", default_value="",
                              description="평가 결과 CSV 경로. 비우면 안 쓴다"),
        DeclareLaunchArgument("publish_tf", default_value="false",
                              description="ESKF 가 odom->base_footprint TF 를 낼지. "
                                          "gz DiffDrive TF 와 충돌하니 둘 중 하나만"),
        DeclareLaunchArgument("slip", default_value="true",
                              description="slip 주입기를 쓸지. false 면 ESKF 가 "
                                          "깨끗한 /odom 을 직접 본다"),
        DeclareLaunchArgument(
            "enc_gyro", default_value="true",
            description="엔코더 각속도를 관측으로 쓸지. **바퀴부를 돌리는 동작"
                        "(patrol.py)에서는 반드시 false 로 둘 것** — 잭업해서 돌리면 "
                        "바퀴가 안 구르므로 엔코더는 '안 돈다'고 하는데, 그 말을 믿으면 "
                        "필터가 90° 회전을 통째로 놓친다 (실측 방위각 오차 88°). "
                        "직진만 할 때는 true 가 맞다 (자이로 바이어스를 잡아 준다)."),
        DeclareLaunchArgument(
            "scale", default_value="true",
            description="엔코더 스케일 오차 k 를 필터 상태로 추정할지. 바퀴 반경 오차·"
                        "상시 slip 처럼 **거리에 비례해 쌓이는 계통 오차**를 잡는다. "
                        "가감속·정지가 있어야 관측되므로 등속 한 방향 주행만 할 때는 "
                        "효과가 없다 (그때는 false 로 두고 봐도 같다)."),
        DeclareLaunchArgument("odom_topic", default_value="/odom_slip",
                              description="ESKF·평가가 볼 오도메트리 토픽. "
                                          "slip:=false 면 /odom_enc 로 줄 것"),
        Node(package="sp_localization", executable="wheel_odom", output="screen",
             parameters=[{"use_sim_time": True}]),
        Node(package="sp_localization", executable="slip_injector", output="screen",
             condition=IfCondition(slip),
             parameters=[{"use_sim_time": True}]),
        Node(package="sp_localization", executable="eskf", output="screen",
             parameters=[{"use_sim_time": True, "publish_tf": publish_tf,
                          "odom_topic": odom_topic,
                          "use_enc_gyro": enc_gyro,
                          "estimate_scale": scale}]),
        Node(package="sp_localization", executable="eval", output="screen",
             parameters=[{"use_sim_time": True, "csv": csv,
                          "odom_topic": odom_topic}]),
    ])
