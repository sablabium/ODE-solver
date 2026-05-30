cmake_minimum_required (VERSION 3.14)

project("ODE_solver" VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

# Defining the executable target
add_executable(ODE_solver "ODE_solver.cpp" "ODE_solver.h")

# Telling CMake where to look for the Eigen headers
# This maps the root of your project directory
target_include_directories(ODE_solver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Compiler Performance Optimizations (Highly recommended for Eigen)
# Eigen relies heavily on compiler optimizations. Without this, your simulations will run very slowly.
if(MSVC)
    # Only use /O2 optimization flags for Release configurations
    # This prevents the conflict with Debug mode's /RTC1 checks
    target_compile_options(ODE_solver PRIVATE 
        $<$<CONFIG:Release>:/O2>
        $<$<CONFIG:RelWithDebInfo>:/O2>
        $<$<CONFIG:MinSizeRel>:/O1>
        /W4
    )
else()
    target_compile_options(ODE_solver PRIVATE -O3 -Wall -Wextra)
endif()
