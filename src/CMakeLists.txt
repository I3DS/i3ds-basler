find_package (Boost COMPONENTS program_options log REQUIRED)

if (NOT(BUILD_EMBEDDED) AND NOT(BUILD_PETALINUX))
  find_path(PCD_TESTDATA images/ HINT ${CMAKE_SOURCE_DIR})


  set (PYLON_ROOT /opt/pylon5)

  execute_process(COMMAND ${PYLON_ROOT}/bin/pylon-config --cflags OUTPUT_VARIABLE BASLER_HR_CPPFLAGS)
  string(STRIP ${BASLER_HR_CPPFLAGS} BASLER_HR_CPPFLAGS)
  string(REGEX REPLACE "\n$" "" BASLER_HR_CPPFLAGS "${BASLER_HR_CPPFLAGS}") #Strip newline
  message(STATUS "BASLER_HR_CPPFLAGS2=${BASLER_HR_CPPFLAGS}")

  set (BASLER_HR_CXXFLAGS -g -O0)  #e.g., CXXFLAGS=-g -O0 for debugging
  message(STATUS "BASLER_HR_CXXFLAGS=${BASLER_HR_CXXFLAGS}")

  execute_process(COMMAND ${PYLON_ROOT}/bin/pylon-config --libs-rpath OUTPUT_VARIABLE BASLER_HR_LDFLAGS)
  string(STRIP ${BASLER_HR_LDFLAGS} ${BASLER_HR_LDFLAGS})
  string(REGEX REPLACE "\n$" "" BASLER_HR_LDFLAGS "${BASLER_HR_LDFLAGS}")  #Strip newline
  message(STATUS "BASLER_HR_LDFLAGS=${BASLER_HR_LDFLAGS}")

  execute_process(COMMAND ${PYLON_ROOT}/bin/pylon-config --libs OUTPUT_VARIABLE BASLER_HR_LDLIBS)
  #-lopencv_core -lopencv_highgui -lpthread
  string(STRIP "${BASLER_HR_LDLIBS}" ${BASLER_HR_LDLIBS})
  string(REGEX REPLACE "\n$" "" BASLER_HR_LDLIBS "${BASLER_HR_LDLIBS}") #Strip newline
  message(STATUS "BASLER_HR_LDLIBS=${BASLER_HR_LDLIBS}")

  # Build tools and flags
  #LD         := $(CXX)
  #CPPFLAGS   := $(shell $(PYLON_ROOT)/bin/pylon-config --cflags) -std=c++11
  #CXXFLAGS   := -g -O0  #e.g., CXXFLAGS=-g -O0 for debugging
  #LDFLAGS    := $(shell $(PYLON_ROOT)/bin/pylon-config --libs-rpath)
  #LDLIBS     := $(shell $(PYLON_ROOT)/bin/pylon-config --libs) -lopencv_core -lopencv_highgui -lpthread
#  message(FATAL_ERROR "Not embedded nor petalinux")
  # Basler HR sample END
else (NOT(BUILD_EMBEDDED) AND NOT(BUILD_PETALINUX))
  find_file(pylon_INCLUDE_DIR include/pylon5)
  find_file(pylon_LIB_DIR lib/pylon5)
  
  include_directories (SYSTEM "${pylon_INCLUDE_DIR}")
  set (BASLER_HR_LDFLAGS "-Wl,--enable-new-dtags -Wl,-rpath,/opt/pylon5/lib64")
  set (BASLER_HR_LDLIBS "-L${pylon_LIB_DIR} -Wl,-E -lpylonbase -lpylonutility -lGenApi_gcc_v3_0_Basler_pylon_v5_0 -lGCBase_gcc_v3_0_Basler_pylon_v5_0 -lLog_gcc_v3_0_Basler_pylon_v5_0 -lMathParser_gcc_v3_0_Basler_pylon_v5_0 -lXmlParser_gcc_v3_0_Basler_pylon_v5_0 -lNodeMapData_gcc_v3_0_Basler_pylon_v5_0")
  #-L${CMAKE_FIND_ROOT_PATH}/usr/lib/pylon5 -L/usr/lib/pylon5 
endif (NOT(BUILD_EMBEDDED) AND NOT(BUILD_PETALINUX))

set (SRCS
  basler_camera.cpp
  )

set (LIBS
  i3ds
  i3ds_clients
  zmq
  pthread
  ${BASLER_HR_LDLIBS}
  ${Boost_LIBRARIES}
  )

include_directories ("../include/")

add_executable (i3ds_basler_hr i3ds_basler_hr.cpp ${SRCS})
target_compile_options(i3ds_basler_hr PRIVATE ${BASLER_HR_CPPFLAGS} -Wno-unknown-pragmas)
target_link_libraries (i3ds_basler_hr ${BASLER_HR_LIB_FLAGS} ${BASLER_HR_LDFLAGS} ${LIBS})
install(TARGETS i3ds_basler_hr DESTINATION bin)
