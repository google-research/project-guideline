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

#include "project_guideline/visualization/camera_feed_renderer.h"

#include <memory>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "project_guideline/testing/images.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/image_io.h"
#include "project_guideline/visualization/render_context.h"

namespace guideline::visualization {
namespace {

const char kTestDataDir[] =
    "/project_guideline/project_guideline/util/testdata/";

std::string GetTestDataPath(const std::string& filename) {
  return std::string(getenv("TEST_SRCDIR")) + kTestDataDir + filename;
}

TEST(CameraFeedRenderer, Render) {
  GL_ASSERT_OK_AND_ASSIGN(auto test_image_rgba,
                          util::LoadImage(GetTestDataPath("landscape.png")));

  std::shared_ptr<util::Image> test_image_rgb =
      std::make_shared<util::Image>(util::RGBAToRGB(test_image_rgba));

  GL_ASSERT_OK_AND_ASSIGN(auto render_context, RenderContext::Create(640, 480));

  CameraFeedRenderer renderer(/*use_gl_texture=*/false);
  GL_ASSERT_OK(renderer.OnGlInit());
  renderer.SetViewport(640, 480);
  renderer.OnImage(test_image_rgb);
  renderer.Render();

  util::Image rendered_image = render_context->GetImage();
  ASSERT_TRUE(ImagesAlmostEqual(test_image_rgba, rendered_image));
}

TEST(CameraFeedRenderer, Rotate180) {
  GL_ASSERT_OK_AND_ASSIGN(auto test_image_rgba,
                          util::LoadImage(GetTestDataPath("landscape.png")));

  std::shared_ptr<util::Image> test_image_rgb =
      std::make_shared<util::Image>(util::RGBAToRGB(test_image_rgba));

  GL_ASSERT_OK_AND_ASSIGN(auto render_context, RenderContext::Create(640, 480));

  CameraFeedRenderer renderer(/*use_gl_texture=*/false,
                              /*texture_rotation_degrees=*/180);
  GL_ASSERT_OK(renderer.OnGlInit());
  renderer.SetViewport(640, 480);
  renderer.OnImage(test_image_rgb);
  renderer.Render();

  cv::rotate(test_image_rgba.data(), test_image_rgba.data(), cv::ROTATE_180);

  util::Image rendered_image = render_context->GetImage();
  ASSERT_TRUE(ImagesAlmostEqual(test_image_rgba, rendered_image));
}

}  // namespace
}  // namespace guideline::visualization
