cmake_minimum_required(VERSION 3.0.2)
project(humanoid_msgs)

# set of packages whose message types are used
set(MSG_DEP_SET 
	actionlib_msgs
	geometry_msgs 
	std_msgs
)


find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	message_generation
) 

add_message_files(FILES
	ServoCommand.msg
	ServoFeedback.msg
	PressureSensors.msg
)

generate_messages(DEPENDENCIES
	${MSG_DEP_SET}
)

catkin_package(CATKIN_DEPENDS 
	${MSG_DEP_SET}
	message_runtime
)
