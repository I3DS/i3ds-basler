cmake_minimum_required(VERSION 3.0)

################################################################################
# Set some project wide defintions
################################################################################

project (i3ds-basler)

set (PROJECT_AUTHOR "SINTEF Digital")
set (PROJECT_URL "http://i3ds-h2020.eu/")
set (PROJECT_LICENSE "MPLv2")
set (PROJECT_VERSION "1.0")

enable_testing ()

option(BUILD_EMBEDDED "Build for embedded systems" OFF)
option(BUILD_PETALINUX "Build for petalinux using bitbake" OFF)
option(BUILD_DEBUG "Enable debug-build with lots and lots of warnings" OFF)

if (BUILD_EMBEDDED)
  if(NOT DEFINED CMAKE_FIND_ROOT_PATH)
    MESSAGE(FATAL_ERROR "CMAKE_FIND_ROOT_PATH path must be set when compiling for embedded")
  endif(NOT DEFINED CMAKE_FIND_ROOT_PATH)

  if (DEFINED BUILD_SYSROOT)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --sysroot=${BUILD_SYSROOT}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=${BUILD_SYSROOT}")
  else ()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --sysroot=${CMAKE_FIND_ROOT_PATH}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=${CMAKE_FIND_ROOT_PATH}")
  endif ()
endif (BUILD_EMBEDDED)

################################################################################
# Check that the C++ compiler supports C++11 or C++0X, else issue warning.
################################################################################

include (CheckCXXCompilerFlag REQUIRED)

function(set_cxx_flag FLAG)
  check_cxx_compiler_flag (${FLAG} CXX_SUPPORT_FLAG)
  if (${CXX_SUPPORT_FLAG})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAG}" PARENT_SCOPE)
  else ()
    message(FATAL_ERROR "Failed setting ${FLAG}, please update CMakeLists")
  endif ()
endfunction(set_cxx_flag)

if (NOT CMAKE_CXX_FLAGS)

  set (DEBUG $ENV{DEBUG})

  if (${DEBUG})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
  else ()
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
  endif ()

  check_cxx_compiler_flag ("-std=c++11" CXX_SUPPORT_STD_11)

  if (${CXX_SUPPORT_STD_11})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  else ()
    message (WARNING "${CMAKE_CXX_COMPILER} does not support -std=c++11 flag")
  endif ()

  check_cxx_compiler_flag ("-Wall" CXX_SUPPORT_WALL)

  if (${CXX_SUPPORT_WALL})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
  endif ()


  check_cxx_compiler_flag ("-fpic" CXX_SUPPORT_FPIC_1)
  check_cxx_compiler_flag ("-fPIC" CXX_SUPPORT_FPIC_2)

  if (${CXX_SUPPORT_FPIC_1})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")
  elseif (${CXX_SUPPORT_FPIC_2})
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  else ()
    message (WARNING "${CMAKE_CXX_COMPILER} does not support -fpic or -fPIC flags")
  endif ()

endif (NOT CMAKE_CXX_FLAGS)

if (${BUILD_DEBUG})
  set_cxx_flag("-ggdb3")
  set_cxx_flag("-Wall")
  set_cxx_flag("-pedantic")
  set_cxx_flag("-Wextra")
  set_cxx_flag("-Werror")
  set_cxx_flag("-Wfatal-errors")
  set_cxx_flag("-fsanitize=address")
endif ()


################################################################################
# Add subdirectories with sources
################################################################################

add_subdirectory (src)
