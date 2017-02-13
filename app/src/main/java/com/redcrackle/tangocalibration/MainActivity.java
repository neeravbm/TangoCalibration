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

package com.redcrackle.tangocalibration;

import android.app.Activity;
import android.content.ComponentName;
import android.content.ServiceConnection;
import android.hardware.Camera;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;
import android.view.Display;
import android.view.TextureView;
import android.view.View;
import android.view.WindowManager;
import android.widget.ToggleButton;

import com.projecttango.examples.cpp.util.TangoInitializationHelper;

/**
 * Main activity shows video overlay scene.
 */
public class MainActivity extends Activity {
    private static final int CAMERA_ID = 0;

    private TextureView mTextureView;
    private GLSurfaceView mSurfaceView;
    private GLSurfaceView mSurfaceView2;
    private ToggleButton mYuvRenderSwitcher;

    private ServiceConnection mTangoServiceCoonnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder binder) {
            TangoJniNative.onTangoServiceConnected(binder);
        }

        @Override
        public void onServiceDisconnected(ComponentName componentName) {
            // Handle this if you need to gracefully shutdown/retry
            // in the event that Tango itself crashes/gets upgraded while running.
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_main);

        // Check the current screen rotation and set it to the renderer.
        WindowManager windowManager = (WindowManager) getSystemService(WINDOW_SERVICE);
        Display display = windowManager.getDefaultDisplay();
        Camera.CameraInfo info = new Camera.CameraInfo();
        Camera.getCameraInfo(CAMERA_ID, info);

        TangoJniNative.onCreate(this, display.getRotation(), info.orientation);

        // Configure OpenGL renderer
        mSurfaceView = (GLSurfaceView) findViewById(R.id.gl_surface_view_original_color_camera);
        mSurfaceView.setEGLContextClientVersion(2);
        mSurfaceView.setRenderer(new CalibrationRenderer());

        mSurfaceView2 = (GLSurfaceView) findViewById(R.id.gl_surface_view_undistorted_color_camera);
        mSurfaceView2.setEGLContextClientVersion(2);
        mSurfaceView2.setRenderer(new CalibrationRenderer(Boolean.TRUE));

        mYuvRenderSwitcher = (ToggleButton) findViewById(R.id.yuv_switcher);
    }

    @Override
    protected void onResume() {
        super.onResume();
        mSurfaceView.onResume();
        mSurfaceView2.onResume();
        TangoInitializationHelper.bindTangoService(this, mTangoServiceCoonnection);
        TangoJniNative.setYuvMethod(mYuvRenderSwitcher.isChecked());
    }

    @Override
    protected void onPause() {
        super.onPause();
        mSurfaceView.onPause();
        mSurfaceView2.onPause();
        TangoJniNative.onPause();
        unbindService(mTangoServiceCoonnection);
    }

    /**
     * The render mode toggle button was pressed.
     */
    public void renderModeClicked(View view) {
        TangoJniNative.setYuvMethod(mYuvRenderSwitcher.isChecked());
    }

    public void onPhotoButtonClicked(View view) {
        TangoJniNative.clickPhoto();
    }
}
