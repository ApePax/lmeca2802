#
#   Universite catholique de Louvain
#   Mechatronic, Electrical Energy, and Dynamic systems (iMMC/MEED) 
#   Robotran - MBsysC
#   https://www.robotran.be
#   Contact : info@robotran.be
#
#
# CMake for compiling a simple robotran project in C
#
# For us:
# cmake .. -DCMAKE_PREFIX_PATH="~/.robotran/mbsysc/MBsysC/install"
#
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                       PROJECT MAIN CONFIGURATIONS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# CMake minimum version
cmake_minimum_required(VERSION 3.11)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# project name
project (philippides_c)

# message to display the project name
message(STATUS "Processing ${PROJECT_NAME}")

find_package(mbsysc 1.28 COMPONENTS module load utilities numerics realtime)
# Error message if mbsysc is not found, it is required ! 
if(NOT EXISTS ${MBSYSC_DIRECTORY})
    message("The given path to MBsysC install directory was not found for this project.\n")
    message("(1) Either the given path(s) is/are not correct(s).\n")
    message("(2) Either the version number found is not correct.\n")
   
    message("\t [HINT] Please do specify the path to correct version when invoking 'cmake' by adding the option '-DCMAKE_PREFIX_PATH=PATH_TO_MBSYSC_INSTALLED/'\n")
    message("\t [HINT] Note: 'PATH_TO_MBSYSC_INSTALLED/' is the full path to your MBsysC install/ folder, it contains the installed files\n")
    message("\t [HINT] Current 'PATH_TO_MBSYSC_INSTALLED/' is/are ${CMAKE_PREFIX_PATH}\n")
    
    message("--> See above messages \n")
    message(FATAL_ERROR "mbsysc package is not found ! ")
else()
    message("Using MBsysC ${MBSYSC_VERSION} from ${MBSYSC_DIRECTORY}")
endif ( )
add_definitions( "-DMBSYSC_PATH=\"${MBSYSC_DIRECTORY}\"")

# Change default installation directory in case of not specified by user
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)  # Requires v3.7.1
    set (CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install"
         CACHE PATH "default install path" FORCE)
endif()

include_directories(${MBSYSC_INCLUDE_DIRECTORIES})

file(GLOB SYMBOLIC_SRC "../symbolicR/*.c")
file(GLOB_RECURSE USER_SRC "../userfctR/*.c")
file(GLOB MAIN_SRC "src/*.c")
include_directories("../symbolicR")
include_directories("../userfctR")
include_directories("src")


add_compile_definitions("BUILD_PATH=\"${CMAKE_CURRENT_BINARY_DIR}\"")
add_compile_definitions("PROJECT_SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

# Windows M_PI definitions
if (WIN32)
    add_definitions(-D _USE_MATH_DEFINES)
endif (WIN32)

# generate the libraries
add_library (Project_user SHARED ${USER_SRC})
add_library (Project_symbolic SHARED ${SYMBOLIC_SRC})
target_link_libraries(Project_user ${MBSYSC_LIBRARIES})
target_link_libraries(Project_symbolic ${MBSYSC_LIBRARIES})

# generate the executable
add_executable (exe_${PROJECT_NAME} ${MAIN_SRC})
add_dependencies(exe_${PROJECT_NAME} Project_user Project_symbolic)
target_link_libraries(exe_${PROJECT_NAME} ${MBSYSC_LIBRARIES})

# Windows: default start up project is the executable
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "${exe_${PROJECT_NAME}}")

# installation process
install(TARGETS Project_user DESTINATION .)
install(TARGETS Project_symbolic DESTINATION .)
install(TARGETS exe_${PROJECT_NAME} DESTINATION .)