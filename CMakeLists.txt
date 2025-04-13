cmake_minimum_required(VERSION 3.16)
project(My_KF_IMPLEMENTATION)

# Source files that need to be included
# - Header files (.h/.hpp) should go in the include directory
# - Implementation files (.cpp) should go in src_with_class directory 
# - Third party Eigen library headers are in thirdparty/eigen

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

set(INCUDE_DIRS 
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src_with_class
        ${PROJECT_SOURCE_DIR}/thirdparty/eigen)

include_directories(${INCUDE_DIRS})

# Add this to suppress the deprecated warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wno-deprecated-declarations)
endif()

add_subdirectory(src_with_class)

