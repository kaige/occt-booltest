cmake_minimum_required(VERSION 3.16)
project(occt_booltest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 COMPONENTS Widgets OpenGL OpenGLWidgets REQUIRED)

set(OCCT_BUILD "/Users/kai/Projects/occt/build-full" CACHE PATH "OCCT build dir")

set(OCCT_LIBS_CORE
    TKernel TKMath TKG2d TKG3d TKGeomBase TKGeomAlgo TKBRep TKTopAlgo
    TKPrim TKBO TKBool TKFillet TKOffset TKShHealing TKMesh
    TKDESTEP TKXSBase)

# native modeling mode (in-memory chain + offscreen screenshots)
add_executable(booltest main.cpp common.cpp)
target_include_directories(booltest PRIVATE ${OCCT_BUILD}/include/opencascade)
target_link_directories(booltest PRIVATE ${OCCT_BUILD}/mac64/clang/lib)
target_link_libraries(booltest PRIVATE Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets ${OCCT_LIBS_CORE})
set_target_properties(booltest PROPERTIES
    BUILD_RPATH "${OCCT_BUILD}/mac64/clang/lib")

# STEP mode: read STEP -> copy -> rotate -> boolean -> write STEP (no Qt)
add_executable(boolstep step_main.cpp common.cpp)
target_include_directories(boolstep PRIVATE ${OCCT_BUILD}/include/opencascade)
target_link_directories(boolstep PRIVATE ${OCCT_BUILD}/mac64/clang/lib)
target_link_libraries(boolstep PRIVATE ${OCCT_LIBS_CORE})
set_target_properties(boolstep PROPERTIES
    BUILD_RPATH "${OCCT_BUILD}/mac64/clang/lib")

# pure corpus runner: read STP pair -> boolean -> record status (no Qt)
add_executable(boolrun run_main.cpp common.cpp)
target_include_directories(boolrun PRIVATE ${OCCT_BUILD}/include/opencascade)
target_link_directories(boolrun PRIVATE ${OCCT_BUILD}/mac64/clang/lib)
target_link_libraries(boolrun PRIVATE ${OCCT_LIBS_CORE})
set_target_properties(boolrun PROPERTIES
    BUILD_RPATH "${OCCT_BUILD}/mac64/clang/lib")

# prepare the STEP input model via native modeling (tutorial bottle)
add_executable(mkbottle mkbottle.cpp common.cpp)
target_include_directories(mkbottle PRIVATE ${OCCT_BUILD}/include/opencascade)
target_link_directories(mkbottle PRIVATE ${OCCT_BUILD}/mac64/clang/lib)
target_link_libraries(mkbottle PRIVATE ${OCCT_LIBS_CORE})
set_target_properties(mkbottle PROPERTIES
    BUILD_RPATH "${OCCT_BUILD}/mac64/clang/lib")

add_executable(diag diag.cpp)
target_include_directories(diag PRIVATE ${OCCT_BUILD}/include/opencascade)
target_link_directories(diag PRIVATE ${OCCT_BUILD}/mac64/clang/lib)
target_link_libraries(diag PRIVATE TKernel TKMath TKG2d TKG3d TKGeomBase TKGeomAlgo TKBRep TKTopAlgo TKPrim TKBO TKBool TKFillet TKOffset TKShHealing TKMesh)
set_target_properties(diag PROPERTIES BUILD_RPATH "${OCCT_BUILD}/mac64/clang/lib")
