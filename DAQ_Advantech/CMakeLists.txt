IF(WIN32)

PROJECT(DAQ_Advantech)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED slam gui  ) # hmtslam hwdrivers ... any other sub-library

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS})

#Use Advantech lib
find_path(DAQ_Advantech_LIB Adsapi32.lib DOC "Path to the containing folder of the Advantech Adsapi32.lib library")
if (${DAQ_Advantech_LIB} STREQUAL  "DAQ_Advantech_LIB-NOTFOUND")
	message ("DAQ_Advantech 'Adsapi.lib' not Found!. \nPlease, provide the path to the Advantech library.")
endif (${DAQ_Advantech_LIB} STREQUAL  "DAQ_Advantech_LIB-NOTFOUND")


# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CDataAcquisition.h 
	CDataAcquisition.cpp	
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} "${DAQ_Advantech_LIB}/Adsapi32.lib")

ENDIF(WIN32)

