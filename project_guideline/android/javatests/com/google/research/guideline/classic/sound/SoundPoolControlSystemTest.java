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

import static com.google.common.truth.Truth.assertThat;
import static org.robolectric.Shadows.shadowOf;

import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.shadows.ShadowSoundPool;

@RunWith(AndroidJUnit4.class)
public final class SoundPoolControlSystemTest {
  private static final float FLOAT_TOLERANCE = 0.01f;

  ShadowSoundPool steeringSoundPool;
  ShadowSoundPool turningSoundPool;
  ShadowSoundPool alertsSoundPool;

  @Test
  public void testFullLaneWidth() {
    SoundPoolControlSystem.Config config =
        SoundPoolControlSystem.Config.builder()
            .setSteeringSensitivity(1.0f)
            .setSteeringSensitivityCurvature(0)
            .setSteeringSoundResource(R.raw.v4_2_steering)
            .setWarningSoundResource(R.raw.v4_2_warning)
            .setTurnResource(R.raw.turn)
            .build();

    SoundPoolControlSystem soundPoolControlSystem = createAudioControlSystem(config);
    assertThat(steeringSoundPool.wasResourcePlayed(R.raw.v4_2_steering)).isTrue();

    soundPoolControlSystem.setPosition(0f);

    assertThat(soundPoolControlSystem.getLastSteeringRate()).isWithin(FLOAT_TOLERANCE).of(1f);
    assertSteeringVolumes(
        soundPoolControlSystem,
        /* steeringLeft= */ 0.09090912f,
        /* steeringRight= */ 0.09090912f,
        /* warningLeft= */ 0f,
        /* warningRight= */ 0f);

    soundPoolControlSystem.setPosition(1f);

    assertThat(soundPoolControlSystem.getLastSteeringRate()).isWithin(FLOAT_TOLERANCE).of(1f);
    assertSteeringVolumes(
        soundPoolControlSystem,
        /* steeringLeft= */ 0f,
        /* steeringRight= */ 1.0f,
        /* warningLeft= */ 1.0f,
        /* warningRight= */ 0f);

    soundPoolControlSystem.setPosition(-1f);

    assertThat(soundPoolControlSystem.getLastSteeringRate()).isWithin(FLOAT_TOLERANCE).of(1f);
    assertSteeringVolumes(
        soundPoolControlSystem,
        /* steeringLeft= */ 1.0f,
        /* steeringRight= */ 0f,
        /* warningLeft= */ 0f,
        /* warningRight= */ 1.0f);

    soundPoolControlSystem.setPosition(0.25f);

    assertThat(soundPoolControlSystem.getLastSteeringRate()).isWithin(FLOAT_TOLERANCE).of(1f);
    assertSteeringVolumes(
        soundPoolControlSystem,
        /* steeringLeft= */ 0f,
        /* steeringRight= */ 0.54545456f,
        /* warningLeft= */ 0.16666666f,
        /* warningRight= */ 0f);
  }

  @Test
  public void testHalfLaneWidth() {
    SoundPoolControlSystem.Config config =
        SoundPoolControlSystem.Config.builder()
            .setSteeringSensitivity(0.5f)
            .setSteeringSensitivityCurvature(0)
            .setSteeringSoundResource(R.raw.v4_2_steering)
            .setWarningSoundResource(R.raw.v4_2_warning)
            .setTurnResource(R.raw.turn)
            .build();

    SoundPoolControlSystem soundPoolControlSystem = createAudioControlSystem(config);
    assertThat(steeringSoundPool.wasResourcePlayed(R.raw.v4_2_steering)).isTrue();

    soundPoolControlSystem.setPosition(0f);

    assertThat(soundPoolControlSystem.getLastSteeringRate()).isWithin(FLOAT_TOLERANCE).of(1f);
    assertSteeringVolumes(
        soundPoolControlSystem,
        /* steeringLeft= */ 0.09090912f,
        /* steeringRight= */ 0.09090912f,
        /* warningLeft= */ 0f,
        /* warningRight= */ 0f);

    soundPoolControlSystem.setPosition(1f);

    assertThat(soundPoolControlSystem.getLastSteeringRate()).isWithin(FLOAT_TOLERANCE).of(1f);
    assertSteeringVolumes(
        soundPoolControlSystem,
        /* steeringLeft= */ 0f,
        /* steeringRight= */ 1.0f,
        /* warningLeft= */ 1.0f,
        /* warningRight= */ 0f);

    soundPoolControlSystem.setPosition(0.25f);

    assertThat(soundPoolControlSystem.getLastSteeringRate()).isWithin(FLOAT_TOLERANCE).of(1f);
    assertSteeringVolumes(
        soundPoolControlSystem,
        /* steeringLeft= */ 0f,
        /* steeringRight= */ 0.3939394f,
        /* warningLeft= */ 0f,
        /* warningRight= */ 0f);
  }

  private void assertSteeringVolumes(
      SoundPoolControlSystem soundPoolControlSystem,
      float steeringLeft,
      float steeringRight,
      float warningLeft,
      float warningRight) {
    assertThat(soundPoolControlSystem.getLastSteeringVolume().leftVolume())
        .isWithin(FLOAT_TOLERANCE)
        .of(steeringLeft);
    assertThat(soundPoolControlSystem.getLastSteeringVolume().rightVolume())
        .isWithin(FLOAT_TOLERANCE)
        .of(steeringRight);

    assertThat(soundPoolControlSystem.getLastWarningVolume().leftVolume())
        .isWithin(FLOAT_TOLERANCE)
        .of(warningLeft);
    assertThat(soundPoolControlSystem.getLastWarningVolume().rightVolume())
        .isWithin(FLOAT_TOLERANCE)
        .of(warningRight);
  }

  private SoundPoolControlSystem createAudioControlSystem(SoundPoolControlSystem.Config config) {
    // TODO: This is short term work-around. Need to find better way of end to end testing.
    SoundPoolControlSystem soundPoolControlSystem =
        new SoundPoolControlSystem(ApplicationProvider.getApplicationContext(), config);

    steeringSoundPool = shadowOf(soundPoolControlSystem.getSteeringSoundPool());
    turningSoundPool = shadowOf(soundPoolControlSystem.getTurningSoundPool());
    alertsSoundPool = shadowOf(soundPoolControlSystem.getAlertsSoundPool());

    steeringSoundPool.notifyResourceLoaded(config.steeringSoundResource(), true);
    steeringSoundPool.notifyResourceLoaded(config.warningSoundResource(), true);
    turningSoundPool.notifyResourceLoaded(config.turnResource(), true);
    alertsSoundPool.notifyResourceLoaded(config.stopResource(), true);
    alertsSoundPool.notifyResourceLoaded(config.lowBatteryResource(), true);
    alertsSoundPool.notifyResourceLoaded(config.alertNotificationResource(), true);

    return soundPoolControlSystem;
  }
}
