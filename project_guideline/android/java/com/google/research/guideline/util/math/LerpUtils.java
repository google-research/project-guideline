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

package com.google.research.guideline.util.math;

/** Linear interpolation (lerp) utilities. */
public class LerpUtils {
  private LerpUtils() {}

  /**
   * Lerps the range [a,b] to [u,v], with clamping.
   *
   * @param x the input value
   * @param a the input minimum
   * @param b the input maximum
   * @param u the output minimum
   * @param v the output maximum
   * @return the output, in the range [u,v]
   */
  public static float lerp(float x, float a, float b, float u, float v) {
    float p = (x - a) / (b - a);
    return u + p * (v - u);
  }

  /**
   * Lerps the range [a,b] to [u,v] but with an interpolator applied.
   *
   * @param x the input value
   * @param a the input minimum
   * @param b the input maximum
   * @param u the output minimum
   * @param v the output maximum
   * @param interpolator the interpolator to apply to the output
   * @return the output
   */
  public static float lerp(float x, float a, float b, float u, float v, Interpolator interpolator) {
    float p = interpolator.interpolate((x - a) / (b - a));
    return u + p * (v - u);
  }

  /**
   * Lerps the range [a,b] to [u,v], with clamping.
   *
   * @param x the input value
   * @param a the input minimum
   * @param b the input maximum
   * @param u the output minimum
   * @param v the output maximum
   * @return the output, in the range [u,v]
   */
  public static float clampedLerp(float x, float a, float b, float u, float v) {
    if (x <= a) {
      return u;
    } else if (x >= b) {
      return v;
    }
    float p = (x - a) / (b - a);
    return u + p * (v - u);
  }

  /**
   * Lerps the range [a,b] to [u,v], with clamping, but with an interpolator applied.
   *
   * @param x the input value
   * @param a the input minimum
   * @param b the input maximum
   * @param u the output minimum
   * @param v the output maximum
   * @param interpolator the interpolator to apply to the output
   * @return the output
   */
  public static float clampedLerp(
      float x, float a, float b, float u, float v, Interpolator interpolator) {
    if (x <= a) {
      return u;
    } else if (x >= b) {
      return v;
    }
    float p = interpolator.interpolate((x - a) / (b - a));
    return u + p * (v - u);
  }
}
