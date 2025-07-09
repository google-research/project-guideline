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

/** Defines an interpolator curve over the interval [0,1]. Includes some standard interpolators. */
public interface Interpolator {
  Interpolator IDENTITY =
      new Interpolator() {
        @Override
        public float interpolate(float input) {
          return input;
        }
      };

  Interpolator SQUARE =
      new Interpolator() {
        @Override
        public float interpolate(float input) {
          return input * input;
        }
      };

  Interpolator SQUARE_ROOT =
      new Interpolator() {
        @Override
        public float interpolate(float input) {
          return (float) Math.sqrt(input);
        }
      };

  Interpolator CUBIC =
      new Interpolator() {
        @Override
        public float interpolate(float t) {
          t *= 2;
          if (t < 1) {
            return t * t * t / 2.0f;
          } else {
            return ((t -= 2) * t * t + 2) / 2;
          }
        }
      };

  float interpolate(float input);
}
