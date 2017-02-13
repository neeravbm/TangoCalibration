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

#include <tango_support_api.h>
#include <vector>

#include "includes/TangoCalibrationApp.hpp"

namespace {
    constexpr int kTangoCoreMinimumVersion = 9377;

    void OnFrameAvailableRouter(void *context, TangoCameraId cameraId,
                                const TangoImageBuffer *buffer) {
        //if (cameraId == TANGO_CAMERA_FISHEYE) {
        tango_calibration::TangoCalibrationApp *app =
                static_cast<tango_calibration::TangoCalibrationApp *>(context);
        app->OnFrameAvailable(buffer);
        //}
    }

// We could do this conversion in a fragment shader if all we care about is
// rendering, but we show it here as an example of how people can use RGB data
// on the CPU.
    inline void Yuv2Rgb(uint8_t yValue, uint8_t uValue, uint8_t vValue, uint8_t *r,
                        uint8_t *g, uint8_t *b) {
        *r = (uint8_t) (yValue + (1.370705 * (vValue - 128)));
        *g = (uint8_t) (yValue - (0.698001 * (vValue - 128)) - (0.337633 * (uValue - 128)));
        *b = (uint8_t) (yValue + (1.732446 * (uValue - 128)));
    }
}  // namespace

namespace tango_calibration {
    void TangoCalibrationApp::OnCreate(JNIEnv *env, jobject caller_activity,
                                       int activity_rotation, int sensor_rotation) {
        // Check the installed version of the TangoCore.  If it is too old, then
        // it will not support the most up to date features.
        int version = 0;
        TangoErrorType err = TangoSupport_GetTangoVersion(env, caller_activity, &version);
        if (err != TANGO_SUCCESS || version < kTangoCoreMinimumVersion) {
            LOGE("TangoCalibrationApp::OnCreate, Tango Core version is out of date.");
            std::exit(EXIT_SUCCESS);
        }

        // Initialize variables
        is_yuv_texture_available_ = false;
        swap_buffer_signal_ = false;
        is_service_connected_ = false;
        is_texture_id_set_ = false;
        video_overlay_drawable_ = NULL;
        yuv_drawable_ = NULL;
        activity_rotation_ = activity_rotation;
        sensor_rotation_ = sensor_rotation;
    }

    void TangoCalibrationApp::OnTangoServiceConnected(JNIEnv *env, jobject binder) {
        if (TangoService_setBinder(env, binder) != TANGO_SUCCESS) {
            LOGE("TangoCalibrationApp::OnTangoServiceConnected, TangoService_setBinder error");
            std::exit(EXIT_SUCCESS);
        }

        // Here, we'll configure the service to run in the way we'd want. For this
        // application, we'll start from the default configuration
        // (TANGO_CONFIG_DEFAULT). This enables basic motion tracking capabilities.
        tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
        if (tango_config_ == nullptr) {
            LOGE("TangoCalibrationApp::OnTangoServiceConnected, Failed to get default config form");
            std::exit(EXIT_SUCCESS);
        }

        // Enable color camera from config.
        int ret = TangoConfig_setBool(tango_config_, "config_enable_color_camera", true);
        if (ret != TANGO_SUCCESS) {
            LOGE("TangoCalibrationApp::OnTangoServiceConnected, config_enable_color_camera() failed with error code: %d",
                 ret);
            std::exit(EXIT_SUCCESS);
        }

        ret = TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, this,
                                                   OnFrameAvailableRouter);
        if (ret != TANGO_SUCCESS) {
            LOGE("TangoCalibrationApp::OnTangoServiceConnected, Error connecting color frame %d",
                 ret);
            std::exit(EXIT_SUCCESS);
        }

        /*int ret = TangoService_connectOnFrameAvailable(TANGO_CAMERA_FISHEYE, this, OnFrameAvailableRouter);

        if (ret != TANGO_SUCCESS) {
            LOGE("TangoCalibrationApp::OnTangoServiceConnected, Error connecting fish eye frame %d", ret);
            std::exit(EXIT_SUCCESS);
        }*/

        // Connect to Tango Service, service will start running, and
        // pose can be queried.
        ret = TangoService_connect(this, tango_config_);
        if (ret != TANGO_SUCCESS) {
            LOGE("TangoCalibrationApp::OnTangoServiceConnected, Failed to connect to the Tango service with error code: %d",
                 ret);
            std::exit(EXIT_SUCCESS);
        }

        is_service_connected_ = true;
    }

    void TangoCalibrationApp::OnPause() {
        // Free TangoConfig structure
        if (tango_config_ != nullptr) {
            TangoConfig_free(tango_config_);
            tango_config_ = nullptr;
        }

        // Disconnect from the Tango service
        TangoService_disconnect();

        // Free buffer data
        is_yuv_texture_available_ = false;
        swap_buffer_signal_ = false;
        is_service_connected_ = false;
        is_texture_id_set_ = false;
        rgb_buffer_.clear();
        yuv_buffer_.clear();
        yuv_temp_buffer_.clear();
        this->DeleteDrawables();
    }

    void TangoCalibrationApp::OnFrameAvailable(const TangoImageBuffer *buffer) {
        if (current_texture_method_ != TextureMethod::kYuv) {
            return;
        }

        if (yuv_drawable_->GetTextureId() == 0) {
            LOGE("TangoCalibrationApp::yuv texture id not valid");
            return;
        }

        if (buffer->format != TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP) {
            LOGE("TangoCalibrationApp::yuv texture format is not supported by this app");
            return;
        }

        LOGE("width: %d", buffer->width);
        LOGE("height: %d", buffer->height);
        LOGE("stride: %d", buffer->stride);
        LOGE("format: %d", buffer->format);

        // The memory needs to be allocated after we get the first frame because we
        // need to know the size of the image.
        if (!is_yuv_texture_available_) {
            // yuv_width_ = buffer->width;
            yuv_width_ = buffer->width;
            yuv_height_ = buffer->height;
            yuv_stride_ = buffer->stride;
            // uv_buffer_offset_ = yuv_width_ * yuv_height_;
            uv_buffer_offset_ = yuv_stride_ * yuv_height_;

            // yuv_size_ = yuv_width_ * yuv_height_ + yuv_width_ * yuv_height_ / 2;
            yuv_size_ = yuv_stride_ * yuv_height_ + yuv_stride_ * yuv_height_ / 2;

            // Reserve and resize the buffer size for RGB and YUV data.
            yuv_buffer_.resize(yuv_size_);
            yuv_temp_buffer_.resize(yuv_size_);
            rgb_buffer_.resize(yuv_width_ * yuv_height_ * 3);

            AllocateTexture(yuv_drawable_->GetTextureId(), yuv_width_, yuv_height_);
            is_yuv_texture_available_ = true;
        }

        std::lock_guard<std::mutex> lock(yuv_buffer_mutex_);
        memcpy(&yuv_temp_buffer_[0], buffer->data, yuv_size_);
        swap_buffer_signal_ = true;
    }

    void TangoCalibrationApp::DeleteDrawables() {
        delete video_overlay_drawable_;
        delete yuv_drawable_;
        video_overlay_drawable_ = NULL;
        yuv_drawable_ = NULL;
    }

    void TangoCalibrationApp::OnSurfaceCreated() {
        if (video_overlay_drawable_ != NULL || yuv_drawable_ != NULL) {
            this->DeleteDrawables();
        }

        TangoSupportDisplayRotation color_camera_to_display_rotation =
                tango_gl::util::GetAndroidRotationFromColorCameraToDisplay(
                        activity_rotation_, sensor_rotation_);

        video_overlay_drawable_ = new tango_gl::VideoOverlay(
                GL_TEXTURE_EXTERNAL_OES, color_camera_to_display_rotation);
        yuv_drawable_ = new tango_gl::VideoOverlay(GL_TEXTURE_2D,
                                                   color_camera_to_display_rotation);
    }

    void TangoCalibrationApp::OnSurfaceChanged(int width, int height) {
        glViewport(0, 0, width, height);
    }

    void TangoCalibrationApp::OnDrawFrame(bool undistort) {
        if (is_service_connected_ && !is_texture_id_set_) {
            is_texture_id_set_ = true;
            // Connect color camera texture. TangoService_connectTextureId expects a
            // valid texture id from the caller, so we will need to wait until the GL
            // content is properly allocated.
            int texture_id = static_cast<int>(video_overlay_drawable_->GetTextureId());
            TangoErrorType ret = TangoService_connectTextureId(
                    TANGO_CAMERA_COLOR, texture_id, nullptr, nullptr);
            /*TangoErrorType ret = TangoService_connectTextureId(
                    TANGO_CAMERA_FISHEYE, texture_id, nullptr, nullptr);*/
            if (ret != TANGO_SUCCESS) {
                LOGE("TangoCalibrationApp: Failed to connect the texture id with error code: %d",
                     ret);
            }
        }

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
        switch (current_texture_method_) {
            case TextureMethod::kYuv:
                RenderYuv(undistort);
                break;
            case TextureMethod::kTextureId:
                RenderTextureId();
                break;
        }
    }

    void TangoCalibrationApp::AllocateTexture(GLuint texture_id, int width, int height) {
        glBindTexture(GL_TEXTURE_2D, texture_id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE,
                     rgb_buffer_.data());
    }

    void TangoCalibrationApp::RenderYuv(bool undistort) {
        if (!is_yuv_texture_available_) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(yuv_buffer_mutex_);
            if (swap_buffer_signal_) {
                std::swap(yuv_buffer_, yuv_temp_buffer_);
                swap_buffer_signal_ = false;
            }
        }

        cv::Mat yuv_frame, rgb_img, gray_img;
        yuv_frame.create(yuv_height_ * 3 / 2, yuv_width_, CV_8UC1);
        memcpy(yuv_frame.data, yuv_buffer_.data(), yuv_width_ * yuv_height_ * 3 / 2);
        cv::cvtColor(yuv_frame, rgb_img, CV_YUV2RGB_NV21);
        cv::cvtColor(rgb_img, gray_img, CV_RGB2GRAY);

        glBindTexture(GL_TEXTURE_2D, yuv_drawable_->GetTextureId());

        if (!undistort) {
            std::vector<cv::Point2f> featureLocationsInXYPlane;
            patternFound = cv::findChessboardCorners(gray_img, cv::Size(14, 21),
                                                     featureLocationsInXYPlane,
                                                     CV_CALIB_CB_FAST_CHECK);
            if (patternFound) {
                cv::cornerSubPix(gray_img, featureLocationsInXYPlane, cv::Size(11, 11), cv::Size(-1, -1),
                                 cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                cv::drawChessboardCorners(rgb_img, cv::Size(14, 21), featureLocationsInXYPlane,
                                          patternFound);
            }

            // For color
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, yuv_width_, yuv_height_, 0, GL_RGB,
                         GL_UNSIGNED_BYTE, rgb_img.data);
        }
        else {
            // For grayscale
            glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, yuv_width_, yuv_height_, 0, GL_LUMINANCE,
                         GL_UNSIGNED_BYTE, gray_img.data);
        }

        yuv_drawable_->Render(glm::mat4(1.0f), glm::mat4(1.0f));
    }

    void TangoCalibrationApp::RenderTextureId() {
        double timestamp;
        // TangoService_updateTexture() updates target camera's
        // texture and timestamp.
        int ret = TangoService_updateTexture(TANGO_CAMERA_COLOR, &timestamp);
        //int ret = TangoService_updateTexture(TANGO_CAMERA_FISHEYE, &timestamp);
        if (ret != TANGO_SUCCESS) {
            LOGE("TangoCalibrationApp: Failed to update the texture id with error code: %d", ret);
        }
        video_overlay_drawable_->Render(glm::mat4(1.0f), glm::mat4(1.0f));
    }

    void TangoCalibrationApp::clickPhoto() {
        LOGE("Photo clicked in native code");
    }

}  // namespace tango_calibration
