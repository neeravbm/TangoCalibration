# Sets the minimum version of CMake required to build the native
# library. You should either keep the default value or only pass a
# value of 3.4.0 or lower.

cmake_minimum_required(VERSION 3.4.1)

# Define the directory where the tango libraries such as tango_client_api, tango_support_api and
# tango_gl are placed.
set(TANGO_LIB_ROOT /Users/neeravbm/Documents/libs/tango)

# Get all the source files from tango_gl library.
# file(GLOB tango-gl-src "${TANGO_LIB_ROOT}/tango_gl/*.cc")
set(
            tango-gl-src # Variable name

            # List of files to compile
            ${TANGO_LIB_ROOT}/tango_gl/drawable_object.cc
            ${TANGO_LIB_ROOT}/tango_gl/shaders.cc
            ${TANGO_LIB_ROOT}/tango_gl/transform.cc
            ${TANGO_LIB_ROOT}/tango_gl/util.cc
            ${TANGO_LIB_ROOT}/tango_gl/video_overlay.cc)

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds it for you.
# Gradle automatically packages shared libraries with your APK.

add_library( # Sets the name of the library.
             tango-calibration-lib

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             # Associated headers in the same location as their source
             # file are automatically included.
             src/main/cpp/jni_interface.cpp
             src/main/cpp/TangoCalibrationApp.cpp

             # Add the source files from tango_gl
             ${tango-gl-src})

# Searches for a specified prebuilt library and stores the path as a
# variable. Because system libraries are included in the search path by
# default, you only need to specify the name of the public NDK library
# you want to add. CMake verifies that the library exists before
# completing its build.

find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log )

# Add tango_client_api library.
add_library(client-api-lib SHARED IMPORTED)
set_target_properties(
                client-api-lib
                PROPERTIES IMPORTED_LOCATION
                ${TANGO_LIB_ROOT}/tango_client_api/lib/${ANDROID_ABI}/libtango_client_api.so)

# Add tango_support_api library.
add_library(support-api-lib SHARED IMPORTED)
set_target_properties(
                support-api-lib
                PROPERTIES IMPORTED_LOCATION
                ${TANGO_LIB_ROOT}/tango_support_api/lib/${ANDROID_ABI}/libtango_support_api.so)

# Add OpenGL ES2 library.
find_library(opengl-lib GLESv2)

# Add OpenCV
find_package(OpenCV REQUIRED)

target_include_directories(
                tango-calibration-lib
                PRIVATE
                ${TANGO_LIB_ROOT}/tango_client_api/include
                ${TANGO_LIB_ROOT}/tango_support_api/include
                ${TANGO_LIB_ROOT}/tango_gl/include
                ${TANGO_LIB_ROOT}/third_party/glm
                ${OpenCV_INCLUDE_DIRS})

# Specifies libraries CMake should link to your target library. You
# can link multiple libraries, such as libraries you define in the
# build script, prebuilt third-party libraries, or system libraries.

target_link_libraries( # Specifies the target library.
                       tango-calibration-lib

                       # Links the target library to the log library
                       # included in the NDK.
                       ${log-lib}

                       client-api-lib

                       support-api-lib

                       ${opengl-lib}

                       ${OpenCV_LIBS})

# Copying libtango_support_api.so to ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} because according to
# http://stackoverflow.com/questions/39202936/how-do-you-link-third-party-library-in-android-cmake-external-build-system,
# all files in this directory get packaged into the API. Without this, this library is missing
# when trying to execute on Tango device.
# An example of ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} is
# app/build/intermediates/cmake/debug/obj/armeabi-v7a.
# We do not need to include libtango_client_api.so in the APK because it is already present in
# Tango device and is loaded by the function TangoInitializationHelper.loadTangoSharedLibrary()
# in the cpp_example_utils project.
add_custom_command(
                TARGET tango-calibration-lib POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy
                ${TANGO_LIB_ROOT}/tango_support_api/lib/${ANDROID_ABI}/libtango_support_api.so
                ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtango_support_api.so)

# add_custom_command(
#                TARGET tango-calibration-lib POST_BUILD
#                COMMAND ${CMAKE_COMMAND} -E copy
#                ${TANGO_LIB_ROOT}/tango_client_api/lib/${ANDROID_ABI}/libtango_client_api.so
#                ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtango_client_api.so)