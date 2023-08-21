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

#ifndef PROJECT_GUIDELINE_VISUALIZATION_RENDER_CONTEXT_H_
#define PROJECT_GUIDELINE_VISUALIZATION_RENDER_CONTEXT_H_

#include <string>

#include <EGL/egl.h>
#include <GL/gl.h>
#include <opencv2/core.hpp>
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "project_guideline/util/image.h"

namespace guideline::visualization {

// Utility that automatically cleans up an EGL pointer using a deleter function
// when it goes out of scope.
template <typename T>
struct EGLScopedPtr {
  EGLScopedPtr(EGLScopedPtr&& other)
      : ptr(other.release()), deleter_(other.deleter_) {}
  EGLScopedPtr(const EGLScopedPtr&) = delete;
  EGLScopedPtr& operator=(const EGLScopedPtr&) = delete;

  EGLScopedPtr(std::function<void(T ptr)> deleter) : deleter_(deleter) {}

  ~EGLScopedPtr() {
    if (ptr) {
      deleter_(ptr);
      ptr = nullptr;
    }
  }

  T& operator*() { return ptr; }

  T ptr = nullptr;

 private:
  T release() {
    T copy = ptr;
    ptr = nullptr;
    return copy;
  }

  std::function<void(T ptr)> deleter_;
};

// Initializes an EGL rendering context for rendering OpenGL to a pixel buffer
// with support for grabbing the rendered image. Can be used with SwiftShader
// for headless rendering.
class RenderContext {
 public:
  static absl::StatusOr<std::unique_ptr<RenderContext>> Create(int width,
                                                               int height);

  // Gets the current frame buffer as an image.
  util::Image GetImage();

 private:
  RenderContext(int width, int height, EGLScopedPtr<EGLDisplay>& egl_display,
                EGLScopedPtr<EGLSurface>& egl_surface,
                EGLScopedPtr<EGLContext>& egl_context);

  const int width_;
  const int height_;

  EGLScopedPtr<EGLDisplay> egl_display_;
  EGLScopedPtr<EGLSurface> egl_surface_;
  EGLScopedPtr<EGLContext> egl_context_;

  std::vector<uint8_t> rgba_buffer_;
};

}  // namespace guideline::visualization

#endif  // PROJECT_GUIDELINE_VISUALIZATION_RENDER_CONTEXT_H_
