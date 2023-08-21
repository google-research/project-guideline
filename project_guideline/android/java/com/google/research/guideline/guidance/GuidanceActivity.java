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

package com.google.research.guideline.guidance;

import android.Manifest;
import android.os.Bundle;
import androidx.appcompat.app.AppCompatActivity;
import com.google.research.guideline.engine.NativeEngineFragment;
import com.google.research.guideline.util.permissions.RequiredPermissionsHelper;
import dagger.hilt.android.AndroidEntryPoint;
import javax.annotation.Nullable;
import javax.inject.Inject;

/** Main Guideline activity. */
@AndroidEntryPoint(AppCompatActivity.class)
public final class GuidanceActivity extends Hilt_GuidanceActivity {
  private static final String[] REQUIRED_PERMISSIONS = {Manifest.permission.CAMERA};

  @Inject RequiredPermissionsHelper permissionsHelper;

  @Override
  public void onCreate(@Nullable Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    permissionsHelper.checkRequiredPermissions(this, REQUIRED_PERMISSIONS, this::createFragment);
  }

  private void createFragment() {
    getSupportFragmentManager()
        .beginTransaction()
        .replace(android.R.id.content, new NativeEngineFragment())
        .commitNow();
  }

  @Override
  protected void onStop() {
    super.onStop();
    // Finish the activity so it will be re-created from scratch the next time.
    finish();
  }
}
