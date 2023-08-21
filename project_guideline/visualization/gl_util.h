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

#ifndef PROJECT_GUIDELINE_VISUALIZATION_GL_UTIL_H_
#define PROJECT_GUIDELINE_VISUALIZATION_GL_UTIL_H_

#include <GLES3/gl3.h>

#define SHADER_VERSION_PREAMBLE "#version 300 es\n"
#define _STRINGIFY(_x) #_x
#define SHADER(_x) SHADER_VERSION_PREAMBLE _STRINGIFY(_x)
#define SHADER_WITH_EXT(_ext, _x) SHADER_VERSION_PREAMBLE _ext _STRINGIFY(_x)

namespace guideline::visualization {

void CheckGlError(const char* error_message);

GLuint LoadShader(GLenum type, const char* shader_name, const GLchar* source);

void LinkProgram(GLuint program);

}  // namespace guideline::visualization

#endif  // PROJECT_GUIDELINE_VISUALIZATION_GL_UTIL_H_
