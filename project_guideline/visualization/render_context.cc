// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "project_guideline/visualization/render_context.h"

#include <functional>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <opencv2/core.hpp>
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "project_guideline/util/status.h"
#include "project_guideline/visualization/gl_util.h"

namespace guideline::visualization {

namespace {

#define RETURN_IF_EGL_ERROR(expr)        \
  {                                      \
    (expr);                              \
    GL_RETURN_IF_ERROR(CheckEglError()); \
  }

absl::Status CheckEglError() {
  auto error = eglGetError();
  if (error != EGL_SUCCESS) {
    return absl::InternalError(absl::StrCat(
        "EGL error: 0x", absl::Hex(error, absl::PadSpec::kZeroPad4)));
  }
  return absl::OkStatus();
}
}  // namespace

absl::StatusOr<std::unique_ptr<RenderContext>> RenderContext::Create(
    int width, int height) {
  EGLScopedPtr<EGLDisplay> egl_display(&eglTerminate);

  RETURN_IF_EGL_ERROR(*egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY));
  if (*egl_display == EGL_NO_DISPLAY) {
    return absl::InternalError("Failed to get EGL display");
  }

  bool success;
  RETURN_IF_EGL_ERROR(success =
                          eglInitialize(egl_display.ptr, nullptr, nullptr));
  if (!success) {
    return absl::InternalError("Failed to initialize EGL");
  }

  RETURN_IF_EGL_ERROR(success = eglBindAPI(EGL_OPENGL_ES_API));
  if (!success) {
    return absl::InternalError("Failed to bind OpenGL API.");
  }

  EGLint num_configs;
  EGLConfig egl_config;
  constexpr EGLint kConfigAttribs[] = {EGL_RED_SIZE,
                                       8,
                                       EGL_GREEN_SIZE,
                                       8,
                                       EGL_BLUE_SIZE,
                                       8,
                                       EGL_ALPHA_SIZE,
                                       8,
                                       EGL_SURFACE_TYPE,
                                       EGL_PBUFFER_BIT,
                                       EGL_RENDERABLE_TYPE,
                                       EGL_OPENGL_ES3_BIT,
                                       EGL_DEPTH_SIZE,
                                       0,
                                       EGL_STENCIL_SIZE,
                                       0,
                                       EGL_NONE};
  RETURN_IF_EGL_ERROR(success = eglChooseConfig(*egl_display, kConfigAttribs,
                                                &egl_config, 1, &num_configs));
  if (!success) {
    return absl::InternalError("Failed to choose a valid an EGLConfig.");
  }

  EGLScopedPtr<EGLSurface> egl_surface(
      [egl_display = egl_display.ptr](EGLSurface egl_surface) {
        eglDestroySurface(egl_display, egl_surface);
      });
  EGLint pixel_buffer_attributes[] = {
      EGL_WIDTH, width, EGL_HEIGHT, height, EGL_NONE,
  };
  RETURN_IF_EGL_ERROR(*egl_surface = eglCreatePbufferSurface(
                          *egl_display, egl_config, pixel_buffer_attributes));

  EGLScopedPtr<EGLContext> egl_context([egl_display = egl_display.ptr](
                                           EGLContext egl_context) {
    eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(egl_display, egl_context);
  });
  constexpr EGLint kContextAttrList[] = {
      EGL_CONTEXT_CLIENT_VERSION,
      3,
      EGL_NONE,
  };
  RETURN_IF_EGL_ERROR(*egl_context =
                          eglCreateContext(*egl_display, egl_config,
                                           EGL_NO_CONTEXT, kContextAttrList));

  RETURN_IF_EGL_ERROR(
      eglMakeCurrent(*egl_display, *egl_surface, *egl_surface, *egl_context));

  return absl::WrapUnique(
      new RenderContext(width, height, egl_display, egl_surface, egl_context));
}

RenderContext::RenderContext(int width, int height,
                             EGLScopedPtr<EGLDisplay>& egl_display,
                             EGLScopedPtr<EGLSurface>& egl_surface,
                             EGLScopedPtr<EGLContext>& egl_context)
    : width_(width),
      height_(height),
      egl_display_(std::move(egl_display)),
      egl_surface_(std::move(egl_surface)),
      egl_context_(std::move(egl_context)) {
  CHECK_GT(width, 0);
  CHECK_GT(height, 0);
  const size_t image_size = width * height;
  rgba_buffer_.resize(image_size * 4);
}

util::Image RenderContext::GetImage() {
  glReadPixels(0, 0, width_, height_, GL_RGBA, GL_UNSIGNED_BYTE,
               rgba_buffer_.data());
  CheckGlError("glReadPixels");
  auto rgba = std::make_unique<cv::Mat>(
      cv::Mat(height_, width_, CV_8UC4, rgba_buffer_.data()).clone());
  cv::flip(*rgba, *rgba, 0);
  return util::Image(util::ImageFormat::kRGBA, width_, height_,
                     std::move(rgba));
}

}  // namespace guideline::visualization
