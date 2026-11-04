cmake_policy(SET CMP0094 NEW)
find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 QUIET)

option(COMPILE_PYTHON_BINDINGS "Compile python bindings (requires pybind11)" TRUE)

if(${pybind11_FOUND} AND ${COMPILE_PYTHON_BINDINGS})

    message(STATUS "Found python (${Python3_EXECUTABLE}) will compile python bindings")
    if(SKBUILD)
        set(XBOT2_PY_PKG_DIR "${SKBUILD_PLATLIB_DIR}/opensot_model_interface")
        # set(XBOT2_PY_PKG_ROOT_DIR "${SKBUILD_PLATLIB_DIR}")
    else()
        execute_process(
            COMMAND ${Python3_EXECUTABLE} -c "import site, os; print(os.path.relpath(site.USER_SITE, site.USER_BASE))"
            OUTPUT_VARIABLE PYTHON_SITE
            OUTPUT_STRIP_TRAILING_WHITESPACE)

        message(STATUS "Python install dir: ${PYTHON_SITE}")
        set(XBOT2_PY_PKG_DIR "${PYTHON_SITE}/opensot_model_interface")
        # set(XBOT2_PY_PKG_ROOT_DIR "${PYTHON_SITE}")
    endif()

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    pybind11_add_module(pyxbot2_interface pyxbot2_interface.cpp)

    target_link_libraries(pyxbot2_interface PUBLIC xbot2_interface)

    install(TARGETS pyxbot2_interface
        DESTINATION ${XBOT2_PY_PKG_DIR})

    pybind11_add_module(pyaffine3 pyaffine3.cpp)

    target_link_libraries(pyaffine3 PUBLIC Eigen3::Eigen)

    install(TARGETS pyaffine3
        DESTINATION ${XBOT2_PY_PKG_DIR})

    if (${XBOT2_IFC_BUILD_COLLISION})
        pybind11_add_module(pyxbot2_collision pyxbot2_collision.cpp)

        target_link_libraries(pyxbot2_collision PUBLIC xbot2_interface::collision)

        install(TARGETS pyxbot2_collision
            DESTINATION ${XBOT2_PY_PKG_DIR})
    endif()

    install(FILES __init__.py DESTINATION ${XBOT2_PY_PKG_DIR})

endif()
