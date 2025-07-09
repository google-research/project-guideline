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

package com.google.research.guideline.classic.sound;

import android.content.Context;
import android.media.AudioAttributes;
import android.media.SoundPool;
import android.support.annotation.VisibleForTesting;
import android.util.Log;
import androidx.annotation.Nullable;
import androidx.annotation.RawRes;
import com.google.auto.value.AutoValue;
import com.google.common.base.Function;
import com.google.research.guideline.classic.sound.PanningStrategy.StereoVolume;
import com.google.research.guideline.util.math.Interpolator;
import com.google.research.guideline.util.math.LerpUtils;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * A basic audio implementation of a control system. Positional information causes stereo "steering"
 * audio to be heard, which is panned left or right to steer the runner back right or left,
 * respectively. The sound also goes up in pitch and gets louder. Think of the runner as being in a
 * "tunnel", with the walls of the tunnel emitting sound that directs the runner down the middle. If
 * the runner strays too far, an additional, very aggressive warning sound is played to prevent
 * accident.
 *
 * <p>Additionally, a separate, intermittent sound is played to indicate if the track is currently
 * curving left or right. To remain consistent with the "tunnel" analogy above, the turning is also
 * heard in the opposite ear of the direction of the curve.
 */
public final class SoundPoolControlSystem {
  private static final String TAG = "SoundPoolControlSystem";

  /**
   * Indicates whether the steering audio is paused, likely because we have issued a stop command to
   * the runner or have not received the first line position yet.
   */
  private boolean steeringPaused = true;

  private final float maxLanePosition;
  private final float sensitivityCurvature;
  private final float maxTurningVolume;
  private final float minTurnAngleDegrees;
  private final float maxTurnAngleDegrees;

  private final SoundPool steeringSoundPool;
  private final SoundPool turningSoundPool;
  private final SoundPool alertsSoundPool;
  private final int steeringId;
  private final int warningId;
  private final int turnId;
  private final int stopId;
  private final int batteryWarningId;
  private final int alertNotificationId;
  private final PanningStrategy steeringPanner;
  private final PanningStrategy warningPanner;
  private final Function<Float, Float> steeringRateStrategy;
  private final Function<Float, Float> warningRateStrategy;

  private float lastSteeringRate;
  private float lastWarningRate;
  @Nullable private StereoVolume lastSteeringVolume;
  @Nullable private StereoVolume lastWarningVolume;
  private final AtomicInteger soundsLoadedCount = new AtomicInteger(0);

  /** Configurable parameters for the {@link SoundPoolControlSystem}. */
  @AutoValue
  public abstract static class Config {
    @RawRes
    public abstract int stopResource();

    @RawRes
    public abstract int lowBatteryResource();

    @RawRes
    public abstract int alertNotificationResource();

    @RawRes
    public abstract int turnResource();

    @RawRes
    public abstract int steeringSoundResource();

    @RawRes
    public abstract int warningSoundResource();

    public abstract PanningStrategy steeringPanner();

    public abstract PanningStrategy warningPanner();

    public abstract Function<Float, Float> steeringRateStrategy();

    public abstract Function<Float, Float> warningRateStrategy();

    public abstract float steeringSensitivity();

    public abstract float steeringSensitivityCurvature();

    public abstract float maxTurningVolume();

    public abstract float minTurnAngleDegrees();

    public abstract float maxTurnAngleDegrees();

    public abstract boolean fastTurns();

    public static Config.Builder builder() {
      return new AutoValue_SoundPoolControlSystem_Config.Builder()
          .setStopResource(R.raw.v4_2_stop)
          .setLowBatteryResource(R.raw.low_battery)
          .setAlertNotificationResource(R.raw.alert_notification)
          .setTurnResource(R.raw.turn)
          .setSteeringSoundResource(R.raw.v4_2_steering)
          .setWarningSoundResource(R.raw.v4_2_warning)
          .setSteeringPanner(new PanningStrategy.LegacyHardPan(/* reversed= */ true))
          .setWarningPanner(new PanningStrategy.LinearThresholdPan(0.4f))
          .setSteeringRateStrategy((position) -> 1f)
          .setWarningRateStrategy(
              (position) ->
                  LerpUtils.clampedLerp(
                      Math.abs(position), 0.4f, 1, 1.0f, 2.0f, Interpolator.SQUARE))
          .setSteeringSensitivity(0.35f)
          .setSteeringSensitivityCurvature(0.4f)
          .setMaxTurningVolume(0.1875f)
          .setMinTurnAngleDegrees(4)
          .setMaxTurnAngleDegrees(8)
          .setFastTurns(false);
    }

    /** Builder for {@link Config}. */
    @AutoValue.Builder
    public abstract static class Builder {
      public abstract Builder setStopResource(@RawRes int stopResource);

      public abstract Builder setLowBatteryResource(@RawRes int lowBatteryResource);

      public abstract Builder setAlertNotificationResource(@RawRes int alertNotificationResource);

      public abstract Builder setTurnResource(@RawRes int turnResource);

      public abstract Builder setSteeringSoundResource(@RawRes int steeringSoundResource);

      public abstract Builder setWarningSoundResource(@RawRes int warningSoundResource);

      public abstract Builder setSteeringPanner(PanningStrategy steeringPanner);

      public abstract Builder setWarningPanner(PanningStrategy warningPanner);

      public abstract Builder setSteeringRateStrategy(Function<Float, Float> steeringRateStrategy);

      public abstract Builder setWarningRateStrategy(Function<Float, Float> warningRateStrategy);

      public abstract Builder setSteeringSensitivity(float sensitivity);

      public abstract Builder setSteeringSensitivityCurvature(float sensitivityCurvature);

      public abstract Builder setMaxTurningVolume(float maxTurningVolume);

      public abstract Builder setMinTurnAngleDegrees(float minTurnAngleDegrees);

      public abstract Builder setMaxTurnAngleDegrees(float maxTurnAngleDegrees);

      public abstract Builder setFastTurns(boolean fastTurns);

      public abstract Config build();
    }
  }

  public static SoundPoolControlSystem createDefault(Context context) {
    return new SoundPoolControlSystem(context, Config.builder().build());
  }

  public SoundPoolControlSystem(Context context, Config config) {
    this.steeringRateStrategy = config.steeringRateStrategy();
    this.warningRateStrategy = config.warningRateStrategy();
    this.warningPanner = config.warningPanner();
    this.steeringPanner = config.steeringPanner();

    maxLanePosition = 1f - (config.steeringSensitivity() * 0.5f);
    sensitivityCurvature = config.steeringSensitivityCurvature();
    maxTurningVolume = config.maxTurningVolume();
    minTurnAngleDegrees = config.minTurnAngleDegrees();
    maxTurnAngleDegrees = config.maxTurnAngleDegrees();

    AudioAttributes audioAttrib =
        new AudioAttributes.Builder()
            .setUsage(AudioAttributes.USAGE_ASSISTANCE_NAVIGATION_GUIDANCE)
            .setContentType(AudioAttributes.CONTENT_TYPE_SONIFICATION)
            .build();
    steeringSoundPool =
        new SoundPool.Builder().setAudioAttributes(audioAttrib).setMaxStreams(3).build();
    turningSoundPool =
        new SoundPool.Builder().setAudioAttributes(audioAttrib).setMaxStreams(1).build();
    alertsSoundPool =
        new SoundPool.Builder().setAudioAttributes(audioAttrib).setMaxStreams(1).build();

    int totalSteeringTurningSoundsToLoad = 3;
    SoundPool.OnLoadCompleteListener loadCompleteListener =
        (soundPool, sampleId, status) -> {
          if (status == 0) {
            if (soundsLoadedCount.incrementAndGet() == totalSteeringTurningSoundsToLoad) {
              start();
              silence();
            }
          } else {
            Log.e(TAG, "Unable to load sample " + sampleId + ", error code: " + status);
          }
        };
    steeringSoundPool.setOnLoadCompleteListener(loadCompleteListener);
    turningSoundPool.setOnLoadCompleteListener(loadCompleteListener);
    steeringId = steeringSoundPool.load(context, config.steeringSoundResource(), 1);
    warningId = steeringSoundPool.load(context, config.warningSoundResource(), 1);
    turnId = turningSoundPool.load(context, config.turnResource(), 1);
    stopId = alertsSoundPool.load(context, config.stopResource(), 1);
    batteryWarningId = alertsSoundPool.load(context, config.lowBatteryResource(), 0);
    alertNotificationId = alertsSoundPool.load(context, config.alertNotificationResource(), 0);
  }

  private void start() {
    // Set rate to 2.0f initially because for some reason setRate will not work later on some audio
    // files if set to 1.0f here.
    steeringSoundPool.play(steeringId, 0, 0, 1, -1, 2.0f);
    steeringSoundPool.play(warningId, 0, 0, 1, -1, 2.0f);
    turningSoundPool.play(turnId, 0, 0, 1, -1, 1.0f);
  }

  public void setPosition(float position) {
    if (steeringPaused) {
      steeringPaused = false;
    }
    float rawPosition = position;

    // Remap the raw position value (-1 is left of screen, 1 is right of screen) using the
    // maxLanePosition for cases where the lane is narrower than the full screen width and we want
    // to trigger audio corrections earlier.
    position = LerpUtils.clampedLerp(position, -maxLanePosition, maxLanePosition, -1f, 1f);

    // Apply curve shaping so there is a slow increase for small deviations from line, and a harder
    // increase when approaching the edge.
    position = applyShaping(position);

    if (Log.isLoggable(TAG, Log.VERBOSE)) {
      Log.v(TAG, "#setPosition(" + rawPosition + ") adjusted: " + position);
    }

    lastSteeringVolume = steeringPanner.getStereoVolume(position);
    steeringSoundPool.setVolume(
        steeringId, lastSteeringVolume.leftVolume(), lastSteeringVolume.rightVolume());

    lastSteeringRate = steeringRateStrategy.apply(position);
    steeringSoundPool.setRate(steeringId, lastSteeringRate);

    // Warning sound
    lastWarningVolume = warningPanner.getStereoVolume(position);
    steeringSoundPool.setVolume(
        warningId, lastWarningVolume.leftVolume(), lastWarningVolume.rightVolume());

    lastWarningRate = warningRateStrategy.apply(position);
    steeringSoundPool.setRate(warningId, lastWarningRate);
  }

  public void setTurning(float turning) {
    if (turning < 0) {
      float leftTurnLevel =
          LerpUtils.clampedLerp(
              turning, -maxTurnAngleDegrees, -minTurnAngleDegrees, maxTurningVolume, 0);
      turningSoundPool.setVolume(turnId, leftTurnLevel, 0);
    } else {
      float rightTurnLevel =
          LerpUtils.clampedLerp(
              turning, minTurnAngleDegrees, maxTurnAngleDegrees, 0, maxTurningVolume);
      turningSoundPool.setVolume(turnId, 0, rightTurnLevel);
    }
  }

  public void setNoLineFound() {
    if (!steeringPaused) {
      alertsSoundPool.play(stopId, 1, 1, 1, 0, 1.0f);
      steeringPaused = true;
      silence();
    }
  }

  public void stop() {
    steeringSoundPool.stop(steeringId);
    steeringSoundPool.stop(warningId);
    turningSoundPool.stop(turnId);
    steeringSoundPool.release();
    turningSoundPool.release();
  }

  public void pause() {
    // Keep audio running for fast resuming, but silence all streams
    silence();
  }

  public void resume() {
    // No-op
  }

  public void warnLowBattery() {
    alertsSoundPool.play(batteryWarningId, 1, 1, 0, 0, 1.0f);
  }

  public void alertNotification() {
    alertsSoundPool.play(alertNotificationId, 1, 1, 0, 0, 1f);
  }

  private void silence() {
    steeringSoundPool.setVolume(steeringId, 0, 0);
    steeringSoundPool.setVolume(warningId, 0, 0);
    turningSoundPool.setVolume(turnId, 0, 0);
  }

  /**
   * Applies some simple shaping to the value of x. If the curvature is 0, then the output is simply
   * x. As the curvature increases, the output develops an "elbow", growing slowly for |x| < 0.5 and
   * then quickly for |x| > 0.5.
   *
   * @param x the input value
   * @return the shaped output
   */
  private float applyShaping(float x) {
    if (sensitivityCurvature == 0) {
      return x;
    }
    float midY = LerpUtils.lerp(sensitivityCurvature, 0, 1, 0.5f, 0.01f);
    float y;
    if (x < -0.5) {
      y = LerpUtils.lerp(x, -1, -0.5f, -1, -midY);
    } else if (x < 0) {
      y = LerpUtils.lerp(x, -0.5f, 0, -midY, 0);
    } else if (x < 0.5) {
      y = LerpUtils.lerp(x, 0, 0.5f, 0, midY);
    } else {
      y = LerpUtils.lerp(x, 0.5f, 1, midY, 1);
    }
    return y;
  }
  
  @VisibleForTesting
  SoundPool getSteeringSoundPool() {
    return steeringSoundPool;
  }

  @VisibleForTesting
  SoundPool getTurningSoundPool() {
    return turningSoundPool;
  }

  @VisibleForTesting
  SoundPool getAlertsSoundPool() {
    return alertsSoundPool;
  }

  @VisibleForTesting
  float getLastSteeringRate() {
    return lastSteeringRate;
  }

  @VisibleForTesting
  float getLastWarningRate() {
    return lastWarningRate;
  }

  @VisibleForTesting
  @Nullable StereoVolume getLastSteeringVolume() {
    return lastSteeringVolume;
  }

  @VisibleForTesting
  @Nullable StereoVolume getLastWarningVolume() {
    return lastWarningVolume;
  }
}
