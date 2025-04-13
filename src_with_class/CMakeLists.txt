# Defining executables directly within src

# Set the output directory for the executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/exe)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 NEW)
endif()

# Add this section to find NumPy include directory
execute_process(
    COMMAND python3 -c "import numpy; print(numpy.get_include())"
    OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)


add_executable(kf_with_class main.cpp my_kf.cpp)

target_include_directories(kf_with_class PRIVATE 
                                ${NUMPY_INCLUDE_DIR}
                                ${Python3_INCLUDE_DIRS}
                                ${Python3_NumPy_INCLUDE_DIRS}
                                )

target_link_libraries(kf_with_class 
                            Python3::Python
                            Python3::Module
                            )

#set(INSTALL_BIN_DIR "${CMAKE_SOURCE_DIR}/exe")
#install(TARGETS kf_with_class RUNTIME DESTINATION ${INSTALL_BIN_DIR})

#Link the Eigen library (Note: Eigen is header-only, so linking is generally not needed)
#target_link_libraries(EigenExample Eigen3::Eigen) # Uncomment if needed
