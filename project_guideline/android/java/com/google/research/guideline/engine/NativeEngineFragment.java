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

import static android.os.PowerManager.PARTIAL_WAKE_LOCK;
import static java.util.Objects.requireNonNull;

import android.annotation.SuppressLint;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.PowerManager;
import android.os.PowerManager.WakeLock;
import androidx.fragment.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Toast;
import com.google.ar.core.ArCoreApk;
import com.google.ar.core.exceptions.FatalException;
import com.google.ar.core.exceptions.UnavailableDeviceNotCompatibleException;
import com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException;
import com.google.protobuf.Duration;
import com.google.research.guideline.proto.AudioSystemOptions;
import com.google.research.guideline.proto.ClearanceZoneOptions;
import com.google.research.guideline.proto.FrameBasedOccupancyMapOptions;
import com.google.research.guideline.proto.FrameBasedPointCloudOptions;
import com.google.research.guideline.proto.GuidanceSystemOptions;
import com.google.research.guideline.proto.GuidelineAggregatorOptions;
import com.google.research.guideline.proto.GuidelineEngineConfig;
import com.google.research.guideline.proto.LegacySoundPackOptions;
import com.google.research.guideline.proto.LegacySoundPackOptions.LegacySoundPackType;
import com.google.research.guideline.proto.LocalTemporalRegressionBasedGuidelineAggregatorOptions;
import com.google.research.guideline.proto.OccupancyMapOptions;
import com.google.research.guideline.proto.PointCloudOptions;
import dagger.hilt.android.AndroidEntryPoint;
import java.util.concurrent.atomic.AtomicBoolean;
import javax.annotation.Nullable;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/** Fragment for the Guideline Native Engine. */
@AndroidEntryPoint(Fragment.class)
public final class NativeEngineFragment extends Hilt_NativeEngineFragment {
  private static final String TAG = "NativeEngineFragment";

  // Obstacle-only mode will only detect obstacles and not guideline navigation.
  private static final boolean ENABLE_OBSTACLE_ONLY_MODE = false;

  // Stop signal given after this time if no line detected in camera frame.
  private static final Duration EAGER_STOP_THRESHOLD = durationFromMillis(750);

  // Estimated height of camera above ground (approx. waist height).
  private static final float CAMERA_HEIGHT_METERS = 1.0f;

  private static final int CLEARANCE_ZONE_HALF_WIDTH_METERS = 2;
  private static final int CLEARANCE_ZONE_DEPTH_METERS = 4;
  // The bottom/top are expressed in meters above the ground.
  private static final float CLEARANCE_ZONE_BOTTOM_METERS = 0.8f;
  private static final float CLEARANCE_ZONE_TOP_METERS = 2.0f;

  private static final int OBSTACLE_OCCUPANCY_THRESHOLD = 10;
  private static final int POINT_CLOUD_SUBSAMPLE_STEP = 4;

  private static final float SOUND_LANE_WIDTH_METERS = 4.0f;
  private static final float SOUND_MAX_ROTATION_DEGREES = 30.0f;
  private static final float SOUND_SENSITIVITY_CURVATURE = 0.4f;

  private final AtomicBoolean started = new AtomicBoolean();
  private final GLSurfaceView.Renderer glSurfaceViewRenderer = new GLSurfaceViewRenderer();

  private GuidelineNativeEngine engine;
  private GLSurfaceView glSurfaceView;

  private WakeLock partialWakeLock;

  private boolean arcoreInstallRequested = false;

  private static Duration durationFromMillis(long milliseconds) {
    long seconds = milliseconds / 1000;
    int nanos = (int) ((milliseconds % 1000) * 1000000);
    return Duration.newBuilder().setSeconds(seconds).setNanos(nanos).build();
  }

  @Override
  public View onCreateView(
      LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
    return inflater.inflate(R.layout.native_engine_layout, container, /* attachToRoot= */ false);
  }

  @Override
  public void onViewCreated(View view, @Nullable Bundle savedInstanceState) {
    GuidelineEngineConfig.Builder configBuilder = GuidelineEngineConfig.newBuilder();

    configBuilder.setGuidanceSystemOptions(
        GuidanceSystemOptions.newBuilder()
            .setEagerStopThreshold(EAGER_STOP_THRESHOLD)
            .setEnableObstacleDetection(true)
            .setObstacleOnlyMode(ENABLE_OBSTACLE_ONLY_MODE)
            .setCameraHeightMeters(CAMERA_HEIGHT_METERS));

    configBuilder.setOccupancyMapOptions(
        OccupancyMapOptions.newBuilder()
            .setFrameBasedOccupancyMapOptions(
                FrameBasedOccupancyMapOptions.newBuilder()
                    .setOccupancyThreshold(OBSTACLE_OCCUPANCY_THRESHOLD)
                    .setClearanceZoneOptions(
                        ClearanceZoneOptions.newBuilder()
                            .setWidth(2 * CLEARANCE_ZONE_HALF_WIDTH_METERS)
                            .setDepth(CLEARANCE_ZONE_DEPTH_METERS)
                            // The bottom/top parameters need to be relative to camera height.
                            .setBottom(CLEARANCE_ZONE_BOTTOM_METERS - CAMERA_HEIGHT_METERS)
                            .setTop(CLEARANCE_ZONE_TOP_METERS - CAMERA_HEIGHT_METERS))));

    configBuilder.setGuidelineAggregatorOptions(
        GuidelineAggregatorOptions.newBuilder()
            .setLocalTemporalRegressionBasedGuidelineAggregatorOptions(
                LocalTemporalRegressionBasedGuidelineAggregatorOptions.getDefaultInstance()));

    configBuilder.setPointCloudOptions(
        PointCloudOptions.newBuilder()
            .setFrameBasedPointCloudOptions(
                FrameBasedPointCloudOptions.newBuilder()
                    .setSubsampleStep(POINT_CLOUD_SUBSAMPLE_STEP)));

    configBuilder.setAudioSystemOptions(
        AudioSystemOptions.newBuilder()
            .setLegacySoundPackOptions(
                LegacySoundPackOptions.newBuilder()
                    .setType(LegacySoundPackType.V4_2)
                    .setWarningThresholdMeters(SOUND_LANE_WIDTH_METERS)
                    .setMaxRotationDegrees(SOUND_MAX_ROTATION_DEGREES)
                    .setSensitivityCurvature(SOUND_SENSITIVITY_CURVATURE)));

    engine = GuidelineNativeEngine.create(getContext(), configBuilder.build());

    PowerManager powerManager = getContext().getSystemService(PowerManager.class);
    partialWakeLock = powerManager.newWakeLock(PARTIAL_WAKE_LOCK, "guideline:partial");

    glSurfaceView = view.findViewById(R.id.gl_surface_view);
    glSurfaceView.setPreserveEGLContextOnPause(true);
    glSurfaceView.setEGLContextClientVersion(2);
    glSurfaceView.setEGLConfigChooser(true);
    glSurfaceView.setRenderer(glSurfaceViewRenderer);
    glSurfaceView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
    glSurfaceView.setWillNotDraw(false);
  }

  @SuppressLint("WakelockTimeout")
  @Override
  public void onStart() {
    super.onStart();
    partialWakeLock.acquire();
  }

  @Override
  public void onStop() {
    super.onStop();
    partialWakeLock.release();
  }

  @Override
  public void onResume() {
    super.onResume();

    try {
      ArCoreApk.InstallStatus installStatus =
          ArCoreApk.getInstance()
              .requestInstall(requireNonNull(getActivity()), !arcoreInstallRequested);
      switch (installStatus) {
        case INSTALLED:
          break;
        case INSTALL_REQUESTED:
          arcoreInstallRequested = true;
          return;
      }
    } catch (UnavailableDeviceNotCompatibleException e) {
      Log.e(TAG, "Device incompatible with ARCore", e);
      Toast.makeText(getContext(), R.string.arcore_incompatible_error, Toast.LENGTH_LONG).show();
      return;
    } catch (UnavailableUserDeclinedInstallationException e) {
      Log.e(TAG, "ARCore installation declined", e);
      Toast.makeText(getContext(), R.string.arcore_declined_error, Toast.LENGTH_LONG).show();
      return;
    } catch (FatalException e) {
      Log.e(TAG, "Failed to install ARCore", e);
      Toast.makeText(getContext(), R.string.arcore_fatal_error, Toast.LENGTH_LONG).show();
      return;
    }

    engine.start();
    glSurfaceView.onResume();

    started.set(true);
  }

  @Override
  public void onPause() {
    super.onPause();

    started.set(false);

    engine.stop();
    glSurfaceView.onPause();
  }

  @Override
  public void onDestroy() {
    super.onDestroy();
    if (engine != null) {
      engine.destroy();
      engine = null;
    }
  }

  private final class GLSurfaceViewRenderer implements GLSurfaceView.Renderer {
    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
      GLES20.glClearColor(/* red= */ 0.1f, /* green= */ 0.1f, /* blue= */ 0.1f, /* alpha= */ 1.0f);
      engine.onGlSurfaceCreated();
    }

    @Override
    public void onSurfaceChanged(GL10 gl, int width, int height) {
      GLES20.glViewport(/* x= */ 0, /* y= */ 0, width, height);
      engine.onGlViewportChanged(width, height);
    }

    @Override
    public void onDrawFrame(GL10 gl) {
      // Clears screen to notify driver it should not load any pixels from previous frame.
      GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);

      if (!started.get()) {
        return;
      }

      engine.onGlDrawFrame();
    }
  }
}
