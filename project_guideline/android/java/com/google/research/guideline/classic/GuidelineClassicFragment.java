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

import static android.os.PowerManager.PARTIAL_WAKE_LOCK;
import static java.util.Objects.requireNonNull;

import android.annotation.SuppressLint;
import android.os.Bundle;
import android.os.PowerManager;
import android.os.PowerManager.WakeLock;
import androidx.fragment.app.Fragment;
import android.util.Log;
import android.util.Size;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import com.google.mediapipe.components.FrameProcessor;
import com.google.mediapipe.framework.MediaPipeException;
import com.google.mediapipe.framework.Packet;
import com.google.mediapipe.framework.PacketGetter;
import com.google.research.guideline.proto.ClassicGuidance;
import com.google.research.guideline.util.ui.ForceAspectRatioLayout;
import dagger.hilt.android.AndroidEntryPoint;
import java.util.Objects;
import javax.annotation.Nullable;
import javax.inject.Inject;

/** Fragment for Guideline Classic mode. */
@AndroidEntryPoint(Fragment.class)
public final class GuidelineClassicFragment extends Hilt_GuidelineClassicFragment {
  private static final String TAG = "GuidelineClassicFragment";

  static {
    System.loadLibrary("mediapipe_jni");
  }

  private static final Size TARGET_RESOLUTION = new Size(800, 600);
  private static final boolean USE_WIDE_ANGLE_LENS = false;

  @Inject MediaPipeControllerFactory mediaPipeControllerFactory;

  private WakeLock partialWakeLock;
  private MediaPipeController mediaPipeController;
  private boolean enableDebugOverlay = true;

  @Override
  public View onCreateView(
      LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
    return inflater.inflate(
        R.layout.guideline_classic_layout, container, /* attachToRoot= */ false);
  }

  @Override
  public void onViewCreated(View view, @Nullable Bundle savedInstanceState) {
    PowerManager powerManager = getContext().getSystemService(PowerManager.class);
    partialWakeLock = powerManager.newWakeLock(PARTIAL_WAKE_LOCK, "guideline:partial");

    ForceAspectRatioLayout previewContainer =
        Objects.requireNonNull(view.findViewById(R.id.preview_container));
    previewContainer.forceAspectRatio(
        (float) TARGET_RESOLUTION.getWidth() / TARGET_RESOLUTION.getHeight());

    MediaPipeController.GraphConfig graphConfig =
        MediaPipeController.GraphConfig.builder()
            .graphName("guideline_classic_graph.binarypb")
            .inputVideoStreamName("input_video")
            .outputVideoStreamName("output_video")
            .flipFramesVertically(true)
            .targetResolution(TARGET_RESOLUTION)
            .useWideAngleLens(USE_WIDE_ANGLE_LENS)
            .build();

    mediaPipeController =
        mediaPipeControllerFactory.create(
            graphConfig,
            requireNonNull(view.findViewById(R.id.preview_surface_view)),
            this::configureFrameProcessor);
    mediaPipeController.initialize();
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
  }

  @Override
  public void onPause() {
    super.onPause();
  }

  @Override
  public void onDestroy() {
    super.onDestroy();
  }

  private void configureFrameProcessor(FrameProcessor frameProcessor) {
    // Can set input side packets here if the graph needs data (settings, etc).
    frameProcessor.setOnWillAddFrameListener(
        (long timestamp) -> onWillAddFrame(timestamp, frameProcessor));
    frameProcessor.addPacketCallback("guidance", this::onGuidance);
  }

  private void onWillAddFrame(long timestamp, FrameProcessor frameProcessor) {
    Packet enableDebugOverlayPacket = null;
    try {
      enableDebugOverlayPacket = frameProcessor.getPacketCreator().createBool(enableDebugOverlay);
      frameProcessor
          .getGraph()
          .addConsumablePacketToInputStream(
              "enable_debug_overlay", enableDebugOverlayPacket, timestamp);
    } catch (MediaPipeException e) {
      Log.e(TAG, "Add packet to input stream failed", e);
    } finally {
      if (enableDebugOverlayPacket != null) {
        enableDebugOverlayPacket.release();
      }
    }
  }

  private void onGuidance(Packet packet) {
    if (packet.isEmpty()) {
      return;
    }
    ClassicGuidance guidance = PacketGetter.getProto(packet, ClassicGuidance.parser());
    if (guidance.hasLineDetection()) {
      // TODO(dhawkey): Add sound implementation.
    }
  }
}
