add_service_files(
  DIRECTORY
  ./
  FILES
  PlanToArmGoal.srv
  ExecuteToArmGoal.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  moveit_msgs
)

message ("generated service files!")
