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

import static java.lang.Math.abs;

import com.google.auto.value.AutoValue;
import com.google.research.guideline.util.math.LerpUtils;

/** Determines the left and right volumes based on input. */
public interface PanningStrategy {

  StereoVolume getStereoVolume(float input);

  /** Value container for left and right volumes. */
  @AutoValue
  abstract class StereoVolume {
    public static StereoVolume create(float left, float right) {
      return new AutoValue_PanningStrategy_StereoVolume(left, right);
    }

    /** The right volume from 0 to 1.0. */
    public abstract float leftVolume();

    /** The right volume from 0 to 1.0. */
    public abstract float rightVolume();
  }

  /**
   * Pans linearly from left when input >= 1 to right when input <= -1. Total volume (left + right)
   * is always equal to {@code volume}.
   */
  final class SpatialLinearStrategy implements PanningStrategy {
    private final float volume;

    public SpatialLinearStrategy(float volume) {
      this.volume = volume;
    }

    @Override
    public StereoVolume getStereoVolume(float input) {
      float left = LerpUtils.clampedLerp(input, -1, 1, 1, 0) * volume;
      float right = LerpUtils.clampedLerp(input, -1, 1, 0, 1) * volume;
      return StereoVolume.create(left, right);
    }
  }

  /**
   * Legacy pan strategy that hard pans to one side when abs(input) > 0.1 and soft pans when input
   * is between -0.1 and 0.1.
   *
   * <ul>
   *   <li>scales left volume linearly from 0 when input is <= -0.1 to 1 when input is 1
   *   <li>scales right volume from linearly 1 when input is -1 to 0 when input is >= 0.1
   * </ul>
   */
  final class LegacyHardPan implements PanningStrategy {
    private final boolean reversed;

    public LegacyHardPan(boolean reversed) {
      this.reversed = reversed;
    }

    @Override
    public StereoVolume getStereoVolume(float input) {
      float left = LerpUtils.clampedLerp(input, -0.1f, 1f, 0, 1);
      float right = LerpUtils.clampedLerp(input, -1, 0.1f, 1, 0);

      if (reversed) {
        return StereoVolume.create(right, left);
      } else {
        return StereoVolume.create(left, right);
      }
    }
  }

  /**
   * Strategy with zero volume when abs(input) < specified threshold, and pans linearly to the
   * opposite side otherwise.
   */
  final class LinearThresholdPan implements PanningStrategy {
    private final float threshold;

    public LinearThresholdPan(float threshold) {
      this.threshold = threshold;
    }

    @Override
    public StereoVolume getStereoVolume(float input) {
      float warningVolume = LerpUtils.clampedLerp(abs(input), threshold, 1, 0, 1);
      return StereoVolume.create(
          /* left= */ input > 0f ? warningVolume : 0, /* right= */ input < 0f ? warningVolume : 0);
    }
  }
}
