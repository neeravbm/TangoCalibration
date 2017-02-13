package com.redcrackle.tangocalibration;

import android.opengl.GLSurfaceView;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import static java.lang.Boolean.FALSE;

public class CalibrationRenderer implements GLSurfaceView.Renderer {

    private boolean undistort = FALSE;

    CalibrationRenderer() {}

    CalibrationRenderer(boolean undistort) {
        this.undistort = undistort;
    }

    @Override
    public void onSurfaceCreated(GL10 gl10, EGLConfig eglConfig) {
        TangoJniNative.onGlSurfaceCreated();
    }

    @Override
    public void onSurfaceChanged(GL10 gl10, int width, int height) {
        TangoJniNative.onGlSurfaceChanged(width, height);
    }

    @Override
    public void onDrawFrame(GL10 gl10) {
        TangoJniNative.onGlSurfaceDrawFrame(undistort);
    }
}