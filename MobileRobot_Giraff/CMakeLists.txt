PROJECT(MobileRobot_Giraff)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED slam gui  hwdrivers)

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CRobotGiraffApp.cpp CRobotGiraffApp.h
	CGiraffMotorsCom.cpp CGiraffMotorsCom.h
	CGiraffCommunication.cpp CGiraffCommunication.h 
	CommonData.cpp CommonData.h
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} )

