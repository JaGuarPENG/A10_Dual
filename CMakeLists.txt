cmake_minimum_required(VERSION 3.12)

project(A10)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
if(UNIX)
	add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS /usr/aris/aris-2.3.10.240826)
endif(UNIX)
IF(WIN32)
#更改路径以适配已安装aris版本
        find_package(aris REQUIRED PATHS C:/aris/aris-2.3.10.240826)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})


add_executable(A10 src/main.cpp  src/robot.cpp src/robot.hpp src/plan.cpp src/plan.hpp src/gravcomp.cpp src/gravcomp.hpp)



if(UNIX)
    target_link_libraries(A10 ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
    target_link_libraries(A10 ${aris_LIBRARIES})
endif(WIN32)


IF(WIN32)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        
    COMMAND ${CMAKE_COMMAND} -E copy  
        "${aris_DIR}/../../../lib/${CMAKE_BUILD_TYPE}/aris_lib.dll"      
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)    
    
endif(WIN32)




