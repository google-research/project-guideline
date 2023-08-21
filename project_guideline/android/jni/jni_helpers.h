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

#ifndef PROJECT_GUIDELINE_ANDROID_JNI_JNI_HELPERS_H_
#define PROJECT_GUIDELINE_ANDROID_JNI_JNI_HELPERS_H_

#include <jni.h>

#include <atomic>
#include <memory>

#include "absl/log/check.h"

namespace guideline::jni {

class EnvRef {
 public:
  static JNIEnv* GetEnv() {
    CHECK(env_) << "Thread JNIEnv not initialized.";
    return env_;
  }

  static void SetEnv(JNIEnv* env) { env_ = env; }
  static inline thread_local JNIEnv* env_ = nullptr;
};

class ThreadGuard {
 public:
  ThreadGuard(ThreadGuard&) = delete;
  ThreadGuard(ThreadGuard&&) = delete;

  explicit ThreadGuard();
  ~ThreadGuard();

 private:
  static inline thread_local int count_ = 0;
  static inline thread_local bool detach_ = false;
};

class JvmRef {
 public:
  explicit JvmRef(JavaVM* jvm) {
    jvm_ = jvm;
    thread_guard_ = std::make_unique<ThreadGuard>();
  }
  static JavaVM* GetJvm() { return jvm_; }

 private:
  static inline std::atomic<JavaVM*> jvm_;
  std::unique_ptr<ThreadGuard> thread_guard_ = nullptr;
};

class GlobalRef {
 public:
  explicit GlobalRef(jobject object) {
    obj_ = EnvRef::GetEnv()->NewGlobalRef(object);
  }
  GlobalRef(GlobalRef&) = delete;
  GlobalRef(GlobalRef&&) = delete;
  ~GlobalRef() { EnvRef::GetEnv()->DeleteGlobalRef(obj_); }
  jobject operator*() { return obj_; }

 private:
  jobject obj_;
};

class WeakGlobalRef {
 public:
  explicit WeakGlobalRef(jobject object) {
    obj_ = EnvRef::GetEnv()->NewWeakGlobalRef(object);
  }
  WeakGlobalRef(WeakGlobalRef&) = delete;
  WeakGlobalRef(WeakGlobalRef&&) = delete;
  ~WeakGlobalRef() { EnvRef::GetEnv()->DeleteWeakGlobalRef(obj_); }
  jobject operator*() { return obj_; }

 private:
  jobject obj_;
};

}  // namespace guideline::jni

#endif  // PROJECT_GUIDELINE_ANDROID_JNI_JNI_HELPERS_H_
