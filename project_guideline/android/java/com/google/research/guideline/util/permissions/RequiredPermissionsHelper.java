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

package com.google.research.guideline.util.permissions;

import static android.content.pm.PackageManager.PERMISSION_GRANTED;
import static android.provider.Settings.ACTION_APPLICATION_DETAILS_SETTINGS;

import android.app.Activity;
import android.content.Intent;
import android.net.Uri;
import androidx.appcompat.app.AlertDialog;
import android.util.Log;
import androidx.activity.result.ActivityResultCaller;
import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts.RequestMultiplePermissions;
import androidx.core.content.ContextCompat;
import dagger.hilt.android.scopes.ActivityScoped;
import java.util.Map;
import javax.inject.Inject;

/** Helper for checking that required runtime permissions have been granted. */
@ActivityScoped
public final class RequiredPermissionsHelper {
  private static final String TAG = "PermissionsHelper";
  private final Activity activity;

  @Inject
  RequiredPermissionsHelper(Activity activity) {
    this.activity = activity;
  }

  /**
   * Checks that the given runtime permissions have been granted, and if not will request that the
   * user grant the permissions.
   *
   * <p>If the user has explicitly denied the permissions then a dialog will be shown that directs
   * the user to the application settings and the current activity will be finished.
   *
   * <p>This must be called in or before {@code onCreate()} of the calling activity/fragment.
   *
   * @param caller the activity or fragment making this call
   * @param requiredPermissions the runtime permissions that are required
   * @param successRunnable will be run immediately if the permissions are already granted, or run
   *     after the user grants the required permissions
   */
  public void checkRequiredPermissions(
      ActivityResultCaller caller, String[] requiredPermissions, Runnable successRunnable) {
    if (hasRequiredPermissions(requiredPermissions)) {
      successRunnable.run();
      return;
    }

    ActivityResultLauncher<String[]> permissionsLauncher =
        caller.registerForActivityResult(
            new RequestMultiplePermissions(),
            (grantResults) -> onPermissionGrantResults(grantResults, successRunnable));

    permissionsLauncher.launch(requiredPermissions);
  }

  private boolean hasRequiredPermissions(String[] requiredPermissions) {
    for (String permission : requiredPermissions) {
      if (ContextCompat.checkSelfPermission(activity, permission) != PERMISSION_GRANTED) {
        Log.w(TAG, "Permission is denied: " + permission);
        return false;
      }
    }
    return true;
  }

  private void onPermissionGrantResults(
      Map<String, Boolean> grantResults, Runnable successRunnable) {
    if (!grantResults.containsValue(false)) {
      successRunnable.run();
    } else {
      new AlertDialog.Builder(activity)
          .setTitle(activity.getString(R.string.permissions_required_dialog_title))
          .setMessage(activity.getString(R.string.permissions_required_dialog_message))
          .setPositiveButton(
              activity.getString(R.string.dialog_settings),
              (dialog, which) -> {
                launchAppSettings();
                dialog.dismiss();
              })
          .setNegativeButton(
              activity.getString(R.string.dialog_cancel), (dialog, which) -> dialog.dismiss())
          .setOnDismissListener((dialog) -> activity.finish())
          .show();
    }
  }

  private void launchAppSettings() {
    Intent intent =
        new Intent(ACTION_APPLICATION_DETAILS_SETTINGS)
            .setData(Uri.fromParts("package", activity.getPackageName(), /* fragment= */ null))
            .addFlags(Intent.FLAG_ACTIVITY_NEW_TASK | Intent.FLAG_ACTIVITY_CLEAR_TASK);
    activity.startActivity(intent);
  }
}
