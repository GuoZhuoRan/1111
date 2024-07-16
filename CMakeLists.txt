cmake_minimum_required(VERSION 3.16)
project(OGHR_mujoco)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++17)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")
find_package(pinocchio REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(quill REQUIRED)
find_package(fmt)
# find_package(qpOASES)

get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

set(MuJoCo_Include_Dir ${PARENT_DIR}/mujoco-3.1.3/include)
set(MuJoCo_Library_Dir ${PARENT_DIR}/mujoco-3.1.3/lib)
set(MuJoCo_Libraries mujoco)

add_subdirectory(ThirdParties/qpOASES)
include_directories("ThirdParties/qpOASES/include")
include_directories("Algorithm")
include_directories("Common")
include_directories("Math")
include_directories("SimInterFace")

include_directories(
    ${MuJoCo_Include_Dir}
)

link_directories(
    ${MuJoCo_Library_Dir}
)

# 添加库目录到链接器搜索路径
# link_directories(${CMAKE_SOURCE_DIR})

# Get C++ sources in the assigned directory
file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp Algorithm/*.cpp Common/*.cpp Math/*.cpp SimInterFace/*.cpp)
file(GLOB HEADER_FILES *.h  Algorithm/*.h Common/*.h Math/*.h SimInterFace/*.h)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES} ${HEADER_FILES})

# add_executable(Walk_mpc_wbc Demo/Walk_mpc_wbc.cpp ${SOURCES})
# target_link_libraries(Walk_mpc_wbc ${MuJoCo_Libraries} glfw Eigen3::Eigen pinocchio::pinocchio
#         JsonCpp::JsonCpp quill::quill fmt::fmt qpOASES)

# add_executable(Walk_wbc Demo/Walk_wbc.cpp ${SOURCES})
# target_link_libraries(Walk_wbc ${MuJoCo_Libraries} glfw Eigen3::Eigen pinocchio::pinocchio
#         JsonCpp::JsonCpp quill::quill fmt::fmt qpOASES)

# add_executable(Jump_mpc Demo/Jump_mpc.cpp ${SOURCES})
# target_link_libraries(Jump_mpc ${MuJoCo_Libraries} glfw Eigen3::Eigen pinocchio::pinocchio
#         JsonCpp::JsonCpp quill::quill fmt::fmt qpOASES)

# add_executable(floatControl Demo/floatControl.cpp ${SOURCES})
# target_link_libraries(floatControl ${MuJoCo_Libraries} glfw Eigen3::Eigen pinocchio::pinocchio
#         JsonCpp::JsonCpp quill::quill fmt::fmt qpOASES )


# add_executable(floatControl_old Demo/floatControl_old.cpp ${SOURCES})
# target_link_libraries(floatControl_old ${MuJoCo_Libraries} glfw Eigen3::Eigen pinocchio::pinocchio
#         JsonCpp::JsonCpp quill::quill fmt::fmt qpOASES )

# add_executable(floatControl_new Demo/floatControl_new.cpp ${SOURCES})
# target_link_libraries(floatControl_new ${MuJoCo_Libraries} glfw Eigen3::Eigen pinocchio::pinocchio
#         JsonCpp::JsonCpp quill::quill fmt::fmt qpOASES )

# add_executable(floatControl_f Demo/floatControl_f.cpp ${SOURCES})
# target_link_libraries(floatControl_f ${MuJoCo_Libraries} glfw Eigen3::Eigen pinocchio::pinocchio
#         JsonCpp::JsonCpp quill::quill fmt::fmt qpOASES )

add_executable(floatControl_new_1 Demo/floatControl_new_1.cpp ${SOURCES})

target_link_libraries(floatControl_new_1 ${MuJoCo_Libraries} glfw Eigen3::Eigen pinocchio::pinocchio
# JsonCpp::JsonCpp        
jsoncpp
         quill::quill fmt::fmt qpOASES )

# 链接动态库
# target_link_libraries(floatControl /home/byz/sim_-oghr_mujoco-master/OGHR_project/ik_7dof_F2O_OLR3/libik_7dof_F2O_OLR3.so)
