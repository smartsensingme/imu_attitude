if(COMMAND idf_component_register)
    idf_component_register(SRCS "imu_attitude.c" INCLUDE_DIRS "include" REQUIRES kalman_attitude)
else()
    cmake_minimum_required(VERSION 3.16)
    project(imu_attitude LANGUAGES C)
    option(IMU_ATTITUDE_BUILD_TESTS "Build native imu_attitude tests" ON)
    option(IMU_ATTITUDE_ENABLE_SANITIZERS "Enable AddressSanitizer and UBSan" OFF)
    set(IMU_ATTITUDE_KALMAN_ATTITUDE_SOURCE_DIR ""
        CACHE PATH "Path to the sibling kalman_attitude source checkout")
    if(NOT IMU_ATTITUDE_KALMAN_ATTITUDE_SOURCE_DIR)
        set(_kalman_attitude_sibling
            "${CMAKE_CURRENT_SOURCE_DIR}/../kalman_attitude")
        if(EXISTS "${_kalman_attitude_sibling}/CMakeLists.txt")
            set(IMU_ATTITUDE_KALMAN_ATTITUDE_SOURCE_DIR
                "${_kalman_attitude_sibling}")
        endif()
    endif()
    if(NOT IMU_ATTITUDE_KALMAN_ATTITUDE_SOURCE_DIR OR
       NOT EXISTS "${IMU_ATTITUDE_KALMAN_ATTITUDE_SOURCE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR
            "Set IMU_ATTITUDE_KALMAN_ATTITUDE_SOURCE_DIR to the "
            "kalman_attitude checkout when building imu_attitude natively")
    endif()

    set(KALMAN_ATTITUDE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(KALMAN_ATTITUDE_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    add_subdirectory("${IMU_ATTITUDE_KALMAN_ATTITUDE_SOURCE_DIR}"
                     "${CMAKE_CURRENT_BINARY_DIR}/kalman_attitude")
    add_library(imu_attitude STATIC imu_attitude.c)
    target_include_directories(imu_attitude PUBLIC include)
    target_link_libraries(imu_attitude PUBLIC kalman_attitude)
    if(UNIX)
        target_link_libraries(imu_attitude PUBLIC m)
    endif()
    target_compile_features(imu_attitude PUBLIC c_std_99)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(imu_attitude PRIVATE -Wall -Wextra -Werror)
    endif()
    if(IMU_ATTITUDE_ENABLE_SANITIZERS AND
       CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(imu_attitude PUBLIC
            -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(imu_attitude PUBLIC
            -fsanitize=address,undefined)
    endif()
    if(IMU_ATTITUDE_BUILD_TESTS)
        enable_testing()
        add_executable(test_imu tests/test_imu.c)
        if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(test_imu PRIVATE -UNDEBUG)
            if(IMU_ATTITUDE_ENABLE_SANITIZERS)
                target_compile_options(test_imu PRIVATE
                    -fsanitize=address,undefined -fno-omit-frame-pointer)
                target_link_options(test_imu PRIVATE
                    -fsanitize=address,undefined)
            endif()
        endif()
        target_link_libraries(test_imu PRIVATE imu_attitude)
        add_test(NAME imu_attitude COMMAND test_imu)
    endif()
endif()
