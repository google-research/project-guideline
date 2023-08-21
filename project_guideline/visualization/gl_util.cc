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

#include "project_guideline/visualization/gl_util.h"

#include <vector>

#include "absl/log/check.h"

namespace guideline::visualization {

namespace {
constexpr size_t kMaxGlLogMessageLength = 1024;
}  // namespace

void CheckGlError(const char* error_message) {
  GLint error = glGetError();
  CHECK(error == GL_NO_ERROR) << "GL Error (" << error << ") " << error_message;
}

GLuint LoadShader(GLenum type, const char* shader_name, const GLchar* source) {
  GLuint shader = glCreateShader(type);
  CheckGlError("create shader");

  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  GLint status = GL_FALSE;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &status);

  if (!status) {
    GLint info_len = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &info_len);
    if (info_len > 0) {
      std::vector<char> buffer(info_len);
      glGetShaderInfoLog(shader, info_len, nullptr, buffer.data());
      glDeleteShader(shader);
      CHECK(false) << "Failed to compile shader " << shader_name << ": "
                   << buffer.data();
    } else {
      glDeleteShader(shader);
      CHECK(false) << "Failed to compile shader " << shader_name;
    }
  }

  return shader;
}

void LinkProgram(GLuint program) {
  glLinkProgram(program);
  GLint status = GL_FALSE;
  glGetProgramiv(program, GL_LINK_STATUS, &status);
  if (!status) {
    GLint length = 0;
    GLchar message[kMaxGlLogMessageLength];
    glGetProgramInfoLog(program, kMaxGlLogMessageLength, &length, message);
    CHECK(false) << "Failed to link program: " << message;
  }
}

}  // namespace guideline::visualization
