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

package com.google.research.guideline.util.ui;

import android.app.Activity;
import androidx.core.view.WindowCompat;
import androidx.core.view.WindowInsetsCompat;
import androidx.core.view.WindowInsetsControllerCompat;
import dagger.hilt.android.scopes.ActivityScoped;
import javax.inject.Inject;

/**
 * Utility to enable immersive mode for an activity (hides status and navigation bars).
 *
 * <p>This is needed for Android 15 (API 35) to replace windowFullscreen attribute. It is also used
 * for prior versions to maintain consistent behavior.
 */
@ActivityScoped
public final class ImmersiveModeController {
  private final WindowInsetsControllerCompat windowInsetsController;

  @Inject
  ImmersiveModeController(Activity activity) {
    windowInsetsController =
        WindowCompat.getInsetsController(activity.getWindow(), activity.getWindow().getDecorView());
  }

  /* Enables immersive mode for the activity. */
  public void enableImmersiveMode() {
    windowInsetsController.setSystemBarsBehavior(
        WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE);
    windowInsetsController.hide(WindowInsetsCompat.Type.systemBars());
  }

  /* Disables immersive mode for the activity. */
  public void disableImmersiveMode() {
    windowInsetsController.show(WindowInsetsCompat.Type.systemBars());
  }
}
