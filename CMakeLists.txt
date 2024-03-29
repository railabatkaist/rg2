cmake_minimum_required(VERSION 3.10)
project(_rg2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rg2/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rg2/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rg2/bin)

######################
### help functions ###
######################
MACRO(SUBDIRLIST result curdir)
    FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
    SET(dirlist "")
    FOREACH(child ${children})
        IF(IS_DIRECTORY ${curdir}/${child})
            LIST(APPEND dirlist ${child})
        ENDIF()
    ENDFOREACH()
    SET(${result} ${dirlist})
ENDMACRO()

####################
### dependencies ###
####################

set(Dependencies)
add_subdirectory(extern/pybind11 pybind11)
find_package(Eigen3 CONFIG REQUIRED)

if (APPLE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else ()
    find_package(OpenMP REQUIRED)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif()

if (UNIX AND NOT APPLE AND NOT ANDROID AND NOT WEBGL AND NOT WIN32)
    set(RAISIM_OS linux)
    list(APPEND CMAKE_PREFIX_PATH rg2/${RAISIM_OS})
# elseif(APPLE)
#     # on macOS "uname -m" returns the architecture (x86_64 or arm64)
#     execute_process(
#             COMMAND uname -m
#             RESULT_VARIABLE result
#             OUTPUT_VARIABLE OSX_NATIVE_ARCHITECTURE
#             OUTPUT_STRIP_TRAILING_WHITESPACE
#     )
#     set(RAISIM_ARCH ${OSX_NATIVE_ARCHITECTURE} CACHE STRING "ARCH")

#     if(${RAISIM_ARCH} STREQUAL "arm64")
#         set(RAISIM_OS m1)
#     else()
#         set(RAISIM_OS mac)
#     endif()

#     list(APPEND CMAKE_PREFIX_PATH ../raisim/${RAISIM_OS})
# elseif(WIN32)
#     set(RAISIM_OS win32)
#     list(APPEND CMAKE_PREFIX_PATH ../raisim/${RAISIM_OS})
#     list(APPEND CMAKE_PREFIX_PATH ../thirdParty)
#     list(APPEND Dependencies Ws2_32)
#     set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)
#     set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)
#     set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)
endif()

find_package(raisim CONFIG REQUIRED)



#######################
### src and linking ###
#######################
# collect all source files
file(GLOB SOURCE_FILES
    rg2/csrc/src/*.cpp
)

pybind11_add_module(${PROJECT_NAME} ${SOURCE_FILES} rg2/csrc/binding.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
target_link_libraries(${PROJECT_NAME} PRIVATE raisim::raisim ${Dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS} ${RAISIMGYM_ENV_DIR}/include)
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC -O3 -flto -mno-avx2)
target_compile_definitions(${PROJECT_NAME} PRIVATE "$<$<CONFIG:Release>:EIGEN_NO_DEBUG>")
target_compile_definitions(${PROJECT_NAME} PRIVATE EIGEN_DONT_PARALLELIZE)

message("raisim os ${RAISIM_OS}")
message("raisim arch ${RAISIM_ARCH}")

if(${RAISIM_OS} STREQUAL "m1")
    if(${RAISIM_ARCH} STREQUAL "arm64")
        message("building with correct lib")
        target_compile_options(${subdir} PRIVATE -mcpu=apple-m1)
    endif()
endif()

# if (CMAKE_BUILD_TYPE STREQUAL "DEBUG" OR CMAKE_BUILD_TYPE STREQUAL "Debug")
#     message("[RAISIM_GYM] BUILDING THE DEBUG APP for ${subdir}")
#     add_executable(${subdir}_debug_app raisimGymTorch/env/debug_app.cpp raisimGymTorch/env/Yaml.cpp)
#     target_link_libraries(${subdir}_debug_app PRIVATE raisim::raisim)
#     target_include_directories(${subdir}_debug_app PUBLIC raisimGymTorch/env/envs/${subdir} ${EIGEN3_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env)
#     if(WIN32)
#         target_link_libraries(${subdir}_debug_app PRIVATE Ws2_32)
#     else()
#         target_compile_options(${subdir}_debug_app PRIVATE -fPIC -g -O0 -mno-avx2)
#     endif()
# endif()