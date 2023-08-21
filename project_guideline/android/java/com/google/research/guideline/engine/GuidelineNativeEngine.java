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

package com.google.research.guideline.engine;

import static java.util.Objects.requireNonNull;

import android.content.Context;
import android.media.AudioManager;
import android.os.Build;
import android.util.Log;
import androidx.annotation.Keep;
import com.google.research.guideline.proto.GuidelineEngineConfig;

/** Interface to the native Guideline engine. */
public final class GuidelineNativeEngine {
  private static final String TAG = "GuidelineNativeEngine";

  private static final int DEFAULT_AUDIO_SAMPLE_RATE_HZ = 48000;
  private static final int DEFAULT_AUDIO_FRAMES_PER_BUFFER = 256;

  /** Callback for receiving control signals from native engine. */
  public interface ControlSignalCallback {
    /**
     * Invoked when a new control signal is generated.
     *
     * @param rotationDegrees the angle in degrees from the users current heading to a point some
     *     distance ahead of the runner
     * @param lateralMovementMeters the tangential distance in meters from the runner to the closest
     *     point on the line
     * @param stop {@code true} if the user should stop and no further guidance given
     */
    @Keep
    public void onControlSignal(float rotationDegrees, float lateralMovementMeters, boolean stop);
  }

  private final long handle;

  private GuidelineNativeEngine(long handle) {
    if (handle == 0) {
      throw new AssertionError("Failed to create native engine");
    }
    this.handle = handle;
  }

  public static void loadNativeLibraries() {
    System.loadLibrary("opencv_java3");
    System.loadLibrary("guideline_native_jni");
  }

  public static GuidelineNativeEngine create(Context context, GuidelineEngineConfig config) {
    loadNativeLibraries();
    return new GuidelineNativeEngine(
        nativeCreateEngine(
            config.toByteArray(),
            Build.FINGERPRINT,
            context.getApplicationContext(),
            requireNonNull(GuidelineNativeEngine.class.getClassLoader()),
            getPreferredAudioFramesPerBuffer(context),
            getPreferredAudioSampleRateHz(context)));
  }

  public static GuidelineNativeEngine createForPlaybackDataset(
      Context context, GuidelineEngineConfig config, String mp4DatasetFilePath) {
    loadNativeLibraries();
    return new GuidelineNativeEngine(
        nativeCreateEngineForPlaybackDataset(
            config.toByteArray(),
            Build.FINGERPRINT,
            mp4DatasetFilePath,
            context.getApplicationContext(),
            requireNonNull(GuidelineNativeEngine.class.getClassLoader()),
            getPreferredAudioFramesPerBuffer(context),
            getPreferredAudioSampleRateHz(context)));
  }

  private static int getPreferredAudioSampleRateHz(Context context) {
    AudioManager audioManager = context.getSystemService(AudioManager.class);
    String sampleRateProperty = audioManager.getProperty(AudioManager.PROPERTY_OUTPUT_SAMPLE_RATE);
    if (sampleRateProperty != null) {
      try {
        return Integer.parseInt(sampleRateProperty);
      } catch (NumberFormatException e) {
        Log.e(TAG, "Failed to parse preferred sample rate", e);
      }
    }
    return DEFAULT_AUDIO_SAMPLE_RATE_HZ;
  }

  private static int getPreferredAudioFramesPerBuffer(Context context) {
    AudioManager audioManager = context.getSystemService(AudioManager.class);
    String framesPerBufferProperty =
        audioManager.getProperty(AudioManager.PROPERTY_OUTPUT_FRAMES_PER_BUFFER);
    if (framesPerBufferProperty != null) {
      try {
        return Integer.parseInt(framesPerBufferProperty);
      } catch (NumberFormatException e) {
        Log.e(TAG, "Failed to parse preferred frames per buffer", e);
      }
    }
    return DEFAULT_AUDIO_FRAMES_PER_BUFFER;
  }

  public void start() {
    nativeStart(handle);
  }

  public void stop() {
    nativeStop(handle);
  }

  public void setControlSignalCallback(ControlSignalCallback callback) {
    nativeSetControlSignalCallback(handle, callback);
  }

  public void onGlSurfaceCreated() {
    nativeOnGlSurfaceCreated(handle);
  }

  public void onGlViewportChanged(int width, int height) {
    nativeOnGlViewportChanged(handle, width, height);
  }

  public void onGlDrawFrame() {
    nativeOnGlDrawFrame(handle);
  }

  public boolean startRecording(String filePath) {
    return nativeStartRecording(handle, filePath);
  }

  public boolean stopRecording() {
    return nativeStopRecording(handle);
  }

  public void onBatteryLevel(int level) {
    nativeOnBatteryLevel(handle, level);
  }

  public void destroy() {
    nativeDestroy(handle);
  }

  private static native long nativeCreateEngine(
      byte[] serializedConfig,
      String buildFingerprint,
      Context appContext,
      ClassLoader classLoader,
      int preferredAudioFramesPerBuffer,
      int preferredAudioSampleRateHz);

  private static native long nativeCreateEngineForPlaybackDataset(
      byte[] serializedConfig,
      String buildFingerprint,
      String mp4DatasetFilePath,
      Context appContext,
      ClassLoader classLoader,
      int preferredAudioFramesPerBuffer,
      int preferredAudioSampleRateHz);

  private static native void nativeSetPreviewTexture(long handle, int textureId);

  private static native void nativeStart(long handle);

  private static native void nativeStop(long handle);

  private static native void nativeSetControlSignalCallback(
      long handle, ControlSignalCallback callback);

  private static native void nativeOnGlSurfaceCreated(long handle);

  private static native void nativeOnGlViewportChanged(long handle, int width, int height);

  private static native void nativeOnGlDrawFrame(long handle);

  private static native void nativeOnBatteryLevel(long handle, int level);

  private static native boolean nativeStartRecording(long handle, String filePath);

  private static native boolean nativeStopRecording(long handle);

  private static native void nativeDestroy(long handle);
}
