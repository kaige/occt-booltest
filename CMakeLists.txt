cmake_minimum_required(VERSION 3.16)
project(occt_booltest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 COMPONENTS Widgets OpenGL OpenGLWidgets REQUIRED)

set(OCCT_BUILD "/Users/kai/Projects/occt/build-full" CACHE PATH "OCCT build dir")

add_executable(booltest main.cpp)
target_include_directories(booltest PRIVATE ${OCCT_BUILD}/include/opencascade)
target_link_directories(booltest PRIVATE ${OCCT_BUILD}/mac64/clang/lib)
target_link_libraries(booltest PRIVATE
    Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets
    TKernel TKMath TKG2d TKG3d TKGeomBase TKGeomAlgo TKBRep TKTopAlgo
    TKPrim TKBO TKBool TKFillet TKOffset TKShHealing TKMesh
    TKDESTEP TKXSBase)
set_target_properties(booltest PROPERTIES
    BUILD_RPATH "${OCCT_BUILD}/mac64/clang/lib")

add_executable(diag diag.cpp)
target_include_directories(diag PRIVATE ${OCCT_BUILD}/include/opencascade)
target_link_directories(diag PRIVATE ${OCCT_BUILD}/mac64/clang/lib)
target_link_libraries(diag PRIVATE TKernel TKMath TKG2d TKG3d TKGeomBase TKGeomAlgo TKBRep TKTopAlgo TKPrim TKBO TKBool TKFillet TKOffset TKShHealing TKMesh)
set_target_properties(diag PROPERTIES BUILD_RPATH "${OCCT_BUILD}/mac64/clang/lib")
