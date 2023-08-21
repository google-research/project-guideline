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

import static android.provider.Settings.ACTION_APPLICATION_DETAILS_SETTINGS;
import static androidx.test.core.app.ApplicationProvider.getApplicationContext;
import static androidx.test.espresso.Espresso.onView;
import static androidx.test.espresso.action.ViewActions.click;
import static androidx.test.espresso.assertion.ViewAssertions.matches;
import static androidx.test.espresso.intent.Intents.intended;
import static androidx.test.espresso.intent.matcher.IntentMatchers.hasAction;
import static androidx.test.espresso.intent.matcher.IntentMatchers.hasDataString;
import static androidx.test.espresso.matcher.RootMatchers.isDialog;
import static androidx.test.espresso.matcher.ViewMatchers.isDisplayed;
import static androidx.test.espresso.matcher.ViewMatchers.withText;
import static com.google.common.truth.Truth.assertThat;
import static com.google.research.guideline.testing.RuntimePermissions.intendingGrantedPermissions;
import static com.google.research.guideline.testing.RuntimePermissions.intendingRevokedPermissions;
import static org.hamcrest.Matchers.allOf;
import static org.hamcrest.Matchers.equalTo;
import static org.robolectric.Shadows.shadowOf;
import static org.robolectric.shadows.ShadowLooper.idleMainLooper;

import android.Manifest;
import android.app.Application;
import android.os.Bundle;
import androidx.fragment.app.Fragment;
import androidx.annotation.Nullable;
import androidx.test.core.app.ActivityScenario;
import androidx.test.espresso.intent.Intents;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import com.google.research.guideline.testing.ShellActivity;
import dagger.hilt.android.AndroidEntryPoint;
import dagger.hilt.android.testing.HiltAndroidRule;
import dagger.hilt.android.testing.HiltAndroidTest;
import dagger.hilt.android.testing.HiltTestApplication;
import java.util.concurrent.atomic.AtomicBoolean;
import javax.inject.Inject;
import org.junit.After;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

/** Tests for RequiredPermissionsHelper. */
@HiltAndroidTest
@RunWith(AndroidJUnit4.class)
@Config(application = HiltTestApplication.class)
public final class RequiredPermissionsHelperTest {
  @Rule public final HiltAndroidRule rules = new HiltAndroidRule(this);

  private static final String[] REQUIRED_PERMISSIONS =
      new String[] {
        Manifest.permission.CAMERA,
        Manifest.permission.READ_EXTERNAL_STORAGE,
        Manifest.permission.WRITE_EXTERNAL_STORAGE
      };

  private ActivityScenario<ShellActivity> activityScenario;
  private TestFragment fragment;

  @Before
  public void setUp() {
    rules.inject();

    Intents.init();
  }

  @After
  public void tearDown() {
    Intents.release();
    if (activityScenario != null) {
      activityScenario.close();
    }
  }

  @Test
  public void allPermissionsAlreadyGranted() throws Exception {
    for (String permission : REQUIRED_PERMISSIONS) {
      shadowOf((Application) getApplicationContext()).grantPermissions(permission);
    }

    launchActivity();

    assertThat(fragment.successCallbackInvoked.get()).isTrue();
  }

  @Test
  public void allPermissionsAccepted() throws Exception {
    intendingGrantedPermissions(getApplicationContext(), REQUIRED_PERMISSIONS);

    launchActivity();
    idleMainLooper();

    assertThat(fragment.successCallbackInvoked.get()).isTrue();
  }

  @Test
  public void allPermissionsRevokedShowsDialog() throws Exception {
    intendingRevokedPermissions(getApplicationContext(), REQUIRED_PERMISSIONS);

    launchActivity();

    onView(withText(R.string.permissions_required_dialog_message))
        .inRoot(isDialog())
        .check(matches(isDisplayed()));

    onView(withText(R.string.dialog_cancel)).inRoot(isDialog()).perform(click());

    assertThat(fragment.requireActivity().isFinishing()).isTrue();
    assertThat(fragment.successCallbackInvoked.get()).isFalse();
  }

  @Test
  public void dismissDialogFinishesActivity() throws Exception {
    intendingRevokedPermissions(getApplicationContext(), REQUIRED_PERMISSIONS);

    launchActivity();

    onView(withText(R.string.dialog_cancel)).inRoot(isDialog()).perform(click());

    assertThat(fragment.requireActivity().isFinishing()).isTrue();
    assertThat(fragment.successCallbackInvoked.get()).isFalse();
  }

  @Test
  public void somePermissionsRevokedShowsDialog() throws Exception {
    intendingRevokedPermissions(getApplicationContext(), REQUIRED_PERMISSIONS[0]);

    launchActivity();

    onView(withText(R.string.permissions_required_dialog_message))
        .inRoot(isDialog())
        .check(matches(isDisplayed()));
  }

  @Test
  public void dialogLaunchesAppSettings() throws Exception {
    intendingRevokedPermissions(getApplicationContext(), REQUIRED_PERMISSIONS);

    launchActivity();

    onView(withText(R.string.dialog_settings)).inRoot(isDialog()).perform(click());

    intended(
        allOf(
            hasAction(ACTION_APPLICATION_DETAILS_SETTINGS),
            hasDataString(equalTo("package:" + getApplicationContext().getPackageName()))));
    assertThat(fragment.successCallbackInvoked.get()).isFalse();
  }

  private void launchActivity() {
    fragment = new TestFragment();
    activityScenario = ActivityScenario.launch(ShellActivity.class);
    activityScenario.onActivity(
        activity -> {
          activity
              .getSupportFragmentManager()
              .beginTransaction()
              .add(android.R.id.content, fragment)
              .commitNow();
        });
  }

  @AndroidEntryPoint(Fragment.class)
  public static final class TestFragment extends Hilt_RequiredPermissionsHelperTest_TestFragment {
    @Inject RequiredPermissionsHelper permissionsHelper;

    private final AtomicBoolean successCallbackInvoked = new AtomicBoolean(false);

    @Override
    public void onCreate(@Nullable Bundle savedInstanceState) {
      super.onCreate(savedInstanceState);
      permissionsHelper.checkRequiredPermissions(
          this, REQUIRED_PERMISSIONS, () -> successCallbackInvoked.set(true));
    }
  }
}
