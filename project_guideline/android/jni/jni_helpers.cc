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

#include "project_guideline/android/jni/jni_helpers.h"

#include <jni.h>

#include "absl/log/check.h"

namespace guideline::jni {

ThreadGuard::ThreadGuard() {
  if (++count_ > 1) {
    return;
  }
  JavaVM* jvm = JvmRef::GetJvm();
  JNIEnv* env = nullptr;
  int status = jvm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
  if (status == JNI_OK) {
    jvm->AttachCurrentThread(&env, nullptr);
    detach_ = true;
  }
  CHECK(env) << "Failed to attach current thread";
  EnvRef::SetEnv(env);
}

ThreadGuard::~ThreadGuard() {
  if (--count_ == 0 && detach_) {
    JvmRef::GetJvm()->DetachCurrentThread();
    detach_ = false;
  }
}

GlobalRef NewGlobalRef(jobject object) {
  return GlobalRef(EnvRef::GetEnv()->NewGlobalRef(object));
}

WeakGlobalRef NewWeakGlobalRef(jobject object) {
  return WeakGlobalRef(EnvRef::GetEnv()->NewWeakGlobalRef(object));
}

}  // namespace guideline::jni
