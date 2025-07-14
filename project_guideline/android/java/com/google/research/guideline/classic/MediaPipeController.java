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

package com.google.research.guideline.classic;

import static com.google.common.base.Preconditions.checkNotNull;
import static java.util.Objects.requireNonNull;

import android.graphics.SurfaceTexture;
import androidx.fragment.app.Fragment;
import android.util.Log;
import android.util.Size;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import androidx.annotation.Nullable;
import androidx.camera.core.Camera;
import androidx.lifecycle.DefaultLifecycleObserver;
import androidx.lifecycle.LifecycleOwner;
import com.google.auto.factory.AutoFactory;
import com.google.auto.factory.Provided;
import com.google.auto.value.AutoValue;
import com.google.common.util.concurrent.FutureCallback;
import com.google.common.util.concurrent.Futures;
import com.google.common.util.concurrent.MoreExecutors;
import com.google.mediapipe.components.CameraHelper;
import com.google.mediapipe.components.CameraXPreviewHelper;
import com.google.mediapipe.components.ExternalTextureConverter;
import com.google.mediapipe.components.FrameProcessor;
import com.google.mediapipe.framework.AndroidAssetUtil;
import com.google.mediapipe.glutil.EglManager;
import dagger.hilt.android.scopes.FragmentScoped;

/** Controller for the MediaPipe graph and camera functionality. */
@FragmentScoped
final class MediaPipeController {
  private static final String TAG = "MediaPipeController";
  private static final int NUM_TEXTURE_CONVERTER_BUFFERS = 2;

  /** Callback that allows the FrameProcessor to be configured. */
  public interface FrameProcessorConfigurer {
    /** Invoked when a new FrameProcessor is created and should be configured. */
    void configureFrameProcessor(FrameProcessor frameProcessor);
  }

  private final Fragment fragment;
  private final GraphConfig graphConfig;
  private final SurfaceView previewSurfaceView;
  private final EglManager eglManager;
  private final FrameProcessorConfigurer configurer;

  private CameraXPreviewHelper cameraHelper;
  private SurfaceTexture previewFrameTexture;

  @Nullable private FrameProcessor frameProcessor;
  @Nullable private ExternalTextureConverter textureConverter;

  @AutoFactory
  MediaPipeController(
      @Provided Fragment fragment,
      GraphConfig graphConfig,
      SurfaceView previewSurfaceView,
      FrameProcessorConfigurer configurer) {
    this.fragment = fragment;
    this.graphConfig = graphConfig;
    this.previewSurfaceView = previewSurfaceView;
    this.configurer = configurer;
    this.eglManager = new EglManager(/* parentContext= */ null);
  }

  public void initialize() {
    setupPreviewSurfaceView();
    AndroidAssetUtil.initializeNativeAssetManager(fragment.requireContext());
    fragment.getLifecycle().addObserver(new LifecycleObserver());
  }

  private void createFrameProcessor() {
    FrameProcessor processor =
        new FrameProcessor(
            fragment.requireContext(),
            eglManager.getNativeContext(),
            graphConfig.graphName(),
            graphConfig.inputVideoStreamName(),
            graphConfig.outputVideoStreamName());
    processor.setDisableMaxFramesInFlight();
    processor.getVideoSurfaceOutput().setFlipY(graphConfig.flipFramesVertically());
    processor.setAsynchronousErrorListener(this::onMediaPipeError);

    configurer.configureFrameProcessor(processor);
    this.frameProcessor = processor;
  }

  private void destroyFrameProcessor() {
    if (frameProcessor != null) {
      frameProcessor.close();
      frameProcessor = null;
    }
  }

  private void setupPreviewSurfaceView() {
    previewSurfaceView.setVisibility(View.GONE);

    previewSurfaceView
        .getHolder()
        .addCallback(
            new SurfaceHolder.Callback() {
              @Override
              public void surfaceCreated(SurfaceHolder holder) {
                requireNonNull(frameProcessor)
                    .getVideoSurfaceOutput()
                    .setSurface(holder.getSurface());
              }

              @Override
              public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
                onPreviewDisplaySurfaceChanged(width, height);
              }

              @Override
              public void surfaceDestroyed(SurfaceHolder holder) {
                if (frameProcessor != null) {
                  frameProcessor.getVideoSurfaceOutput().setSurface(null);
                }
              }
            });
  }

  private void startCamera() {
    ExternalTextureConverter converter =
        new ExternalTextureConverter(eglManager.getContext(), NUM_TEXTURE_CONVERTER_BUFFERS);
    converter.setFlipY(graphConfig.flipFramesVertically());
    converter.setConsumer(requireNonNull(frameProcessor));
    converter.setRotation(
        fragment
            .requireContext()
            .getSystemService(WindowManager.class)
            .getDefaultDisplay()
            .getRotation());

    cameraHelper = new CameraXPreviewHelper();
    previewFrameTexture = converter.getSurfaceTexture();
    cameraHelper.setOnCameraStartedListener(this::onCameraStarted);
    cameraHelper.setOnCameraBoundListener(this::onCameraBound);
    cameraHelper.setLandscapeOrientation(graphConfig.landscapeOrientation());

    this.textureConverter = converter;

    cameraHelper.startCamera(
        fragment.requireContext(),
        fragment.getViewLifecycleOwner(),
        graphConfig.cameraFacing(),
        previewFrameTexture,
        graphConfig.targetResolution());
  }

  private void stopCamera() {
    requireNonNull(textureConverter).close();
    previewSurfaceView.setVisibility(View.GONE);
  }

  private void onCameraStarted(@Nullable SurfaceTexture surfaceTexture) {
    previewFrameTexture = checkNotNull(surfaceTexture);
    previewSurfaceView.setVisibility(View.VISIBLE);
  }

  private void onCameraBound(Camera camera) {
    if (graphConfig.useWideAngleLens()) {
      Futures.addCallback(
          camera
              .getCameraControl()
              .setZoomRatio(
                  requireNonNull(camera.getCameraInfo().getZoomState().getValue())
                      .getMinZoomRatio()),
          new FutureCallback<Void>() {
            @Override
            public void onSuccess(Void unused) {
              Log.d(TAG, "Switched to wide angle lens");
            }

            @Override
            public void onFailure(Throwable t) {
              Log.e(TAG, "Failed to switch to wide angle lens", t);
            }
          },
          MoreExecutors.directExecutor());
    }
  }

  private void onPreviewDisplaySurfaceChanged(int width, int height) {
    Size viewSize = new Size(width, height);
    Size displaySize = cameraHelper.computeDisplaySizeFromViewSize(viewSize);
    boolean isCameraRotated = cameraHelper.isCameraRotated();

    // Configure the output width and height as the computed display size.
    requireNonNull(textureConverter)
        .setDestinationSize(
            isCameraRotated ? displaySize.getHeight() : displaySize.getWidth(),
            isCameraRotated ? displaySize.getWidth() : displaySize.getHeight());
  }

  private void onMediaPipeError(RuntimeException e) {
    Log.e(TAG, "onMediaPipeError", e);

    // Throw the exception to crash the app. By default FrameProcess will just log the exceptions
    // and continue to run the graph which typically continues to fail if there is a problem in the
    // graph.
    throw e;
  }

  /** Observes lifecycle events of the host fragment. */
  private final class LifecycleObserver implements DefaultLifecycleObserver {
    @Override
    public void onResume(LifecycleOwner lifecycleOwner) {
      createFrameProcessor();
      startCamera();
    }

    @Override
    public void onPause(LifecycleOwner lifecycleOwner) {
      stopCamera();
      destroyFrameProcessor();
    }
  }

  /** Configuration for the MediaPipe graph. */
  @AutoValue
  public abstract static class GraphConfig {
    public abstract String graphName();

    public abstract String inputVideoStreamName();

    public abstract String outputVideoStreamName();

    public abstract boolean flipFramesVertically();

    public abstract CameraHelper.CameraFacing cameraFacing();

    public abstract Size targetResolution();

    public abstract boolean landscapeOrientation();

    public abstract boolean useWideAngleLens();

    public static Builder builder() {
      return new AutoValue_MediaPipeController_GraphConfig.Builder()
          .flipFramesVertically(false)
          .cameraFacing(CameraHelper.CameraFacing.BACK)
          .targetResolution(new Size(800, 600))
          .landscapeOrientation(true)
          .useWideAngleLens(false);
    }

    /** Builder for {@link GraphConfig}. */
    @AutoValue.Builder
    abstract static class Builder {
      abstract Builder graphName(String value);

      abstract Builder inputVideoStreamName(String value);

      abstract Builder outputVideoStreamName(String value);

      abstract Builder cameraFacing(CameraHelper.CameraFacing value);

      abstract Builder flipFramesVertically(boolean value);

      abstract Builder targetResolution(Size value);

      abstract Builder landscapeOrientation(boolean landscapeOrientation);

      abstract Builder useWideAngleLens(boolean useWideAngleLens);

      abstract GraphConfig build();
    }
  }
}
