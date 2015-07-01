cmake_minimum_required(VERSION 2.8.3)
project(nmea_navsat_driver)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  AVR.msg
  GGK.msg
)

catkin_python_setup()

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package()

install(PROGRAMS
   scripts/nmea_serial_driver.py
   scripts/nmea_topic_driver.py
   scripts/nmea_topic_serial_reader.py
   scripts/log_publisher.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Check package for pep8 style, add a test to fail on violations.
#roslint_python()
#roslint_add_test()
