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

package com.google.research.guideline.testing;

import static androidx.test.espresso.intent.Intents.intending;
import static androidx.test.espresso.intent.matcher.IntentMatchers.hasAction;
import static java.util.Objects.requireNonNull;
import static org.robolectric.Shadows.shadowOf;

import android.app.Activity;
import android.app.Application;
import android.app.Instrumentation.ActivityResult;
import android.content.Intent;
import android.content.pm.PackageManager;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.hamcrest.Description;
import org.hamcrest.TypeSafeMatcher;

/** Testing utilities for runtime permissions. */
public final class RuntimePermissions {

  public static void intendingGrantedPermissions(Application application, String... permissions) {
    intendingPermissions(
        application,
        Arrays.asList(permissions),
        Collections.<String>emptyList(),
        Activity.RESULT_OK);
  }

  public static void intendingRevokedPermissions(Application application, String... permissions) {
    intendingPermissions(
        application,
        Collections.<String>emptyList(),
        Arrays.asList(permissions),
        Activity.RESULT_OK);
  }

  public static void intendingPermissions(
      Application application,
      List<String> grantedPermissions,
      List<String> revokedPermissions,
      int resultCode) {
    intending(
            new TypeSafeMatcher<Intent>() {
              @Override
              public void describeTo(Description description) {
                description.appendText("requestedPermission");
              }

              @Override
              public boolean matchesSafely(Intent intent) {
                return hasAction("android.content.pm.action.REQUEST_PERMISSIONS").matches(intent);
              }
            })
        .respondWithFunction(
            intent -> {
              List<String> requested =
                  Arrays.asList(
                      requireNonNull(
                          intent.getStringArrayExtra(
                              "android.content.pm.extra.REQUEST_PERMISSIONS_NAMES")));

              String[] requestedGranted =
                  requested.stream()
                      .distinct()
                      .filter(p -> grantedPermissions.contains(p))
                      .toArray(String[]::new);

              String[] requestedRevoked =
                  requested.stream()
                      .distinct()
                      .filter(p -> revokedPermissions.contains(p))
                      .toArray(String[]::new);

              shadowOf(application).grantPermissions(requestedGranted);

              Intent resultData = new Intent();
              int totalResults = requestedGranted.length + requestedRevoked.length;
              int[] grantResults = new int[totalResults];
              String[] permissionNames = new String[totalResults];

              int i = 0;
              for (String granted : requestedGranted) {
                grantResults[i] = PackageManager.PERMISSION_GRANTED;
                permissionNames[i] = granted;
                i++;
              }

              for (String revoked : requestedRevoked) {
                grantResults[i] = PackageManager.PERMISSION_DENIED;
                permissionNames[i] = revoked;
                i++;
              }

              resultData.putExtra(
                  "android.content.pm.extra.REQUEST_PERMISSIONS_RESULTS", grantResults);
              resultData.putExtra(
                  "android.content.pm.extra.REQUEST_PERMISSIONS_NAMES", permissionNames);

              return new ActivityResult(resultCode, resultData);
            });
  }

  private RuntimePermissions() {}
}
