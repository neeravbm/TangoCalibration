/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define GLM_FORCE_RADIANS

#include <jni.h>
#include "includes/TangoCalibrationApp.hpp"

static tango_calibration::TangoCalibrationApp app;

#ifdef __cplusplus
extern "C" {
#endif
JNIEXPORT void JNICALL
Java_com_redcrackle_tangocalibration_TangoJniNative_onCreate(JNIEnv *env, jobject, jobject activity,
                                                             int activity_rotation,
                                                             int sensor_rotation) {
    app.OnCreate(env, activity, activity_rotation, sensor_rotation);
}

JNIEXPORT void JNICALL
Java_com_redcrackle_tangocalibration_TangoJniNative_onTangoServiceConnected(JNIEnv *env, jobject,
                                                                            jobject binder) {
    app.OnTangoServiceConnected(env, binder);
}

JNIEXPORT void JNICALL
Java_com_redcrackle_tangocalibration_TangoJniNative_onPause(JNIEnv *, jobject) {
    app.OnPause();
}

JNIEXPORT void JNICALL
Java_com_redcrackle_tangocalibration_TangoJniNative_onGlSurfaceCreated(JNIEnv *, jobject) {
    app.OnSurfaceCreated();
}

JNIEXPORT void JNICALL
Java_com_redcrackle_tangocalibration_TangoJniNative_onGlSurfaceChanged(JNIEnv *, jobject,
                                                                       jint width, jint height) {
    app.OnSurfaceChanged(width, height);
}

JNIEXPORT void JNICALL
Java_com_redcrackle_tangocalibration_TangoJniNative_onGlSurfaceDrawFrame(JNIEnv *, jobject, jboolean undistort) {
    app.OnDrawFrame(undistort);
}

JNIEXPORT void JNICALL
Java_com_redcrackle_tangocalibration_TangoJniNative_setYuvMethod(JNIEnv *, jobject,
                                                                 jboolean use_yuv_method) {
    if (use_yuv_method) {
        app.SetTextureMethod(tango_calibration::TangoCalibrationApp::TextureMethod::kYuv);
    } else {
        app.SetTextureMethod(tango_calibration::TangoCalibrationApp::TextureMethod::kTextureId);
    }
}

JNIEXPORT void JNICALL
Java_com_redcrackle_tangocalibration_TangoJniNative_clickPhoto(JNIEnv *, jobject) {
    app.clickPhoto();
}

#ifdef __cplusplus
}
#endif
