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

#include <android/native_window_jni.h>
#include <jni.h>

#include <memory>
#include <string>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "project_guideline/android/jni/guideline_engine_wrapper.h"
#include "project_guideline/android/jni/jni_helpers.h"
#include "project_guideline/proto/control_signal.pb.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"

#define JNI_METHOD_NAME(method_name) \
  Java_com_google_research_guideline_engine_GuidelineNativeEngine_##method_name
#define JNI_METHOD(return_type, method_name) \
  JNIEXPORT return_type JNICALL JNI_METHOD_NAME(method_name)

namespace {

using guideline::ControlSignalCallbackWrapper;
using guideline::ControlSignalLite;
using guideline::GuidelineEngineConfig;
using guideline::GuidelineEngineWrapper;
using guideline::jni::EnvRef;
using guideline::jni::JvmRef;
using guideline::jni::ThreadGuard;

static std::unique_ptr<JvmRef> jvm = nullptr;

inline jlong jptr(void* handle) { return reinterpret_cast<jlong>(handle); }

inline GuidelineEngineWrapper* native(jlong ptr) {
  return reinterpret_cast<GuidelineEngineWrapper*>(ptr);
}

template <typename ProtoType>
auto FromByteArray(jbyteArray bytes) -> ProtoType {
  JNIEnv* env = EnvRef::GetEnv();
  jint length = env->GetArrayLength(bytes);
  jbyte* array = env->GetByteArrayElements(bytes, nullptr);
  ProtoType result;
  CHECK(result.ParseFromArray(array, length)) << "Failed to parse proto";
  env->ReleaseByteArrayElements(bytes, array, JNI_ABORT);
  return result;
}

inline std::string ToString(jstring str) {
  std::string result;
  JNIEnv* env = EnvRef::GetEnv();
  jsize length = env->GetStringUTFLength(str);
  const char* utf = env->GetStringUTFChars(str, nullptr);
  result = std::string(utf, length);
  env->ReleaseStringUTFChars(str, utf);
  return result;
}

class ControlSignalCallbackImpl : public ControlSignalCallbackWrapper {
 public:
  ControlSignalCallbackImpl(jobject callback_object) {
    JNIEnv* env = EnvRef::GetEnv();
    callback_object_ref_ = env->NewGlobalRef(callback_object);
    jclass cls = env->GetObjectClass(callback_object_ref_);
    method_id_ = env->GetMethodID(cls, "onControlSignal", "([B)V");
  }

  ~ControlSignalCallbackImpl() override {
    JNIEnv* env = EnvRef::GetEnv();
    env->DeleteGlobalRef(callback_object_ref_);
  }

  void OnControlSignal(const ControlSignalLite& signal) override {
    ThreadGuard thread_guard;
    auto* env = EnvRef::GetEnv();
    std::string serialized_signal = signal.SerializeAsString();
    jbyteArray serialized_array = env->NewByteArray(serialized_signal.size());
    env->SetByteArrayRegion(
        serialized_array, 0, serialized_signal.size(),
        reinterpret_cast<const jbyte*>(serialized_signal.c_str()));
    env->CallVoidMethod(callback_object_ref_, method_id_, serialized_array);
  }

 private:
  jobject callback_object_ref_;
  jmethodID method_id_;
};

}  // namespace

extern "C" {

jint JNI_OnLoad(JavaVM* vm, void*) {
  jvm.reset(new JvmRef(vm));
  return JNI_VERSION_1_6;
}

JNI_METHOD(jlong, nativeCreateEngine)
(JNIEnv* env, jclass, jbyteArray serialized_config, jstring build_fingerprint,
 jobject app_context, jobject class_loader,
 jint preferred_audio_frames_per_buffer, jint preferred_audio_sample_rate_hz) {
  auto config = FromByteArray<GuidelineEngineConfig>(serialized_config);

  auto engine = guideline::GuidelineEngineWrapper::Create(
      app_context, ToString(build_fingerprint), config,
      preferred_audio_frames_per_buffer, preferred_audio_sample_rate_hz);
  if (!engine.ok()) {
    LOG(ERROR) << "Failed to create engine: " << engine.status();
    return 0;
  }

  return jptr(engine->release());
}

JNI_METHOD(jlong, nativeCreateEngineForPlaybackDataset)
(JNIEnv* env, jclass, jbyteArray serialized_config, jstring build_fingerprint,
 jstring dataset_path, jobject app_context, jobject class_loader,
 jint preferred_audio_frames_per_buffer, jint preferred_audio_sample_rate_hz) {
  auto config = FromByteArray<GuidelineEngineConfig>(serialized_config);

  // TODO(b/276748005): Implement ARCore playback.

  auto engine = guideline::GuidelineEngineWrapper::Create(
      app_context, ToString(build_fingerprint), config,
      preferred_audio_frames_per_buffer, preferred_audio_sample_rate_hz);
  if (!engine.ok()) {
    LOG(ERROR) << "Failed to create engine: " << engine.status();
    return 0;
  }

  return jptr(engine->release());
}

JNI_METHOD(jboolean, nativeStartRecording)
(JNIEnv* env, jclass, jlong engine, jstring file_path) {
  auto status = native(engine)->StartRecording(ToString(file_path));
  if (!status.ok()) {
    LOG(ERROR) << "Failed to start recording: " << status;
  }
  return status.ok();
}

JNI_METHOD(jboolean, nativeStopRecording)
(JNIEnv*, jclass, jlong engine) {
  auto status = native(engine)->StopRecording();
  if (!status.ok()) {
    LOG(ERROR) << "Failed to stop recording: " << status;
  }
  return status.ok();
}

JNI_METHOD(void, nativeStart)
(JNIEnv*, jclass, jlong engine) { CHECK_OK(native(engine)->Start()); }

JNI_METHOD(void, nativeStop)
(JNIEnv*, jclass, jlong engine) {
  auto status = native(engine)->Stop();
  if (!status.ok()) {
    LOG(ERROR) << "Failed to stop: " << status;
  }
}

JNI_METHOD(void, nativeSetControlSignalCallback)
(JNIEnv* env, jclass, jlong engine, jobject callback_object_ref) {
  native(engine)->SetControlSignalCallback(
      std::make_unique<ControlSignalCallbackImpl>(callback_object_ref));
}

JNI_METHOD(void, nativeOnGlSurfaceCreated)
(JNIEnv*, jclass, jlong engine) { native(engine)->OnGlSurfaceCreated(); }

JNI_METHOD(void, nativeOnGlViewportChanged)
(JNIEnv*, jclass, jlong engine, jint width, jint height) {
  native(engine)->OnGlViewportChanged(width, height);
}

JNI_METHOD(void, nativeOnGlDrawFrame)
(JNIEnv*, jclass, jlong engine) { native(engine)->OnGlDrawFrame(); }

JNI_METHOD(void, nativeOnBatteryLevel)
(JNIEnv*, jclass, jlong engine, jint level) {
  native(engine)->OnBatteryLevel(level);
}

JNI_METHOD(void, nativeDestroy)
(JNIEnv*, jclass, jlong engine) { delete native(engine); }

}  // extern "C"
