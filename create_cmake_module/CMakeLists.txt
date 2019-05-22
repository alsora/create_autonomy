cmake_minimum_required(VERSION 3.5)

project(create_cmake_module)

find_package(ament_cmake REQUIRED)
find_package(libcreate QUIET)


macro(build_libcreate)
    set(extra_cmake_args)

    if(DEFINED CMAKE_BUILD_TYPE)
        list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
    endif()

    if(DEFINED CMAKE_TOOLCHAIN_FILE)
        list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
    endif()

    list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${POCO_CXX_FLAGS}")
    list(APPEND extra_cmake_args "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
    list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")

    include(ExternalProject)
    ExternalProject_Add(libcreate-pkg
        GIT_REPOSITORY "https://github.com/AutonomyLab/libcreate.git"
        GIT_TAG "4e8d49702434de952daa19bf9b4a3904b15cea68"
        TIMEOUT 60
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/libcreate_install
        ${extra_cmake_args}
    )

    # The external project will install to the build folder, but we'll install that on make install.
    install(
        DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libcreate_install/
        DESTINATION ${CMAKE_INSTALL_PREFIX}
    )
endmacro()

build_libcreate()

install(FILES
    package.xml
    DESTINATION share/${PROJECT_NAME}
)

ament_package()