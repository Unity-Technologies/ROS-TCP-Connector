cmake_minimum_required(VERSION 2.8.3)
project(unity_interfaces)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(FILES
	RosUnityError.msg
	RosUnitySrvMessage.msg
	RosUnitySysCommand.msg
)

add_service_files(FILES
	RosUnityTopicList.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
