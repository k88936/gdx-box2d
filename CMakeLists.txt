cmake_minimum_required(VERSION 3.30)
set(CMAKE_CXX_STANDARD  17)
project(gdx-box2d)
add_library(gdx-box2d STATIC
        build/jnigen/jni/com_badlogic_gdx_box2d_Box2d.cpp
        build/jnigen/jni/com_badlogic_gdx_box2d_Box2dPlus.cpp
        build/jnigen/jni/com_badlogic_gdx_box2d_FFITypes.cpp
)
include_directories(
        build/jnigen/jni
        build/jnigen/jni/jni-headers
        build/jnigen/jni/jni-headers/linux
        box2d/include
)