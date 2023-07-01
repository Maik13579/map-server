cmake_minimum_required(VERSION 3.0.2)
project(ros_map_server_interfaces)

find_package(catkin REQUIRED COMPONENTS
  rospy
  #std_msgs
  message_generation
)

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Example.msg
 )

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   Example2.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Example.action
# )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   #std_msgs  # Or other packages containing msgs
 )
catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
