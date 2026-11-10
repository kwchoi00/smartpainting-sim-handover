cmake_minimum_required(VERSION 3.8)
project(sp_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

# STEP 도면에서 변환한 모델. step_to_gazebo.py 를 돌리기 전엔 없으므로 OPTIONAL.
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(
  PROGRAMS scripts/gen_world.py scripts/step_to_gazebo.py scripts/inspect_block.py
           scripts/step_to_robot.py scripts/maneuver.py
           scripts/scissor_driver.py scripts/patrol.py scripts/cleanup.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
