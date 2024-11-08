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
import android.view.View;
import android.view.ViewGroup.MarginLayoutParams;
import androidx.annotation.IntDef;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowCompat;
import androidx.core.view.WindowInsetsCompat;
import androidx.preference.PreferenceFragmentCompat;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

/**
 * Utilities for edge-to-edge mode which is enforced on Android 15 (API 35).
 *
 * <p>See https://developer.android.com/develop/ui/views/layout/edge-to-edge
 */
public final class EdgeToEdgeUtils {
  private EdgeToEdgeUtils() {}

  /** Edge options for applying window insets. */
  @IntDef({InsetEdge.TOP, InsetEdge.BOTTOM, InsetEdge.LEFT, InsetEdge.RIGHT, InsetEdge.ALL})
  @Retention(RetentionPolicy.SOURCE)
  public @interface InsetEdge {
    int TOP = 1 << 1;
    int BOTTOM = 1 << 2;
    int LEFT = 1 << 3;
    int RIGHT = 1 << 4;
    int VERTICAL = TOP | BOTTOM;
    int HORIZONTAL = LEFT | RIGHT;
    int ALL = TOP | BOTTOM | LEFT | RIGHT;
  }

  /**
   * Enables edge-to-edge mode for the activity.
   *
   * <p>Note that edge-to-edge mode is enabled by default on Android 15 (API 35). This method can be
   * called to enable edge-to-edge mode on earlier versions of Android for consistency.
   *
   * <p>Should be called from onCreate().
   */
  public static void enableEdgeToEdge(Activity activity) {
    WindowCompat.setDecorFitsSystemWindows(activity.getWindow(), false);
  }

  /**
   * Handles the edge-to-edge window insets for a preference fragment.
   *
   * <p>Should be called from onCreateView/onViewCreated.
   */
  public static void applyInsetsToPreferences(PreferenceFragmentCompat prefFragment, View view) {
    ViewCompat.setOnApplyWindowInsetsListener(
        view,
        (v, windowInsets) -> {
          Insets contentInsets = getContentInsets(windowInsets);
          prefFragment
              .getListView()
              .setPadding(
                  contentInsets.left, contentInsets.top, contentInsets.right, contentInsets.bottom);

          return WindowInsetsCompat.CONSUMED;
        });
  }

  /**
   * Applies the window insets as padding to the view, in addition to any existing padding.
   *
   * <p>Should be called from onCreateView/onViewCreated.
   *
   * @param view The target view.
   * @param edges The inset edges to apply.
   */
  public static void applyInsetsAsPadding(View view, @InsetEdge int edges) {
    ViewCompat.setOnApplyWindowInsetsListener(
        view,
        (v, windowInsets) -> {
          Insets contentInsets = getContentInsets(windowInsets);
          Insets insets = applyEdges(contentInsets, edges);

          Insets originalPadding = (Insets) view.getTag(R.id.edge_to_edge_original_padding_tag);
          if (originalPadding == null) {
            originalPadding =
                Insets.of(
                    view.getPaddingLeft(),
                    view.getPaddingTop(),
                    view.getPaddingRight(),
                    view.getPaddingBottom());
            view.setTag(R.id.edge_to_edge_original_padding_tag, originalPadding);
          }
          insets = Insets.add(insets, originalPadding);
          view.setPadding(insets.left, insets.top, insets.right, insets.bottom);
          return windowInsets;
        });
  }

  /**
   * Applies the window insets as margins to the view, in addition to any existing margins.
   *
   * <p>Should be called from onCreateView/onViewCreated.
   *
   * @param view The target view.
   * @param edges The inset edges to apply.
   */
  public static void applyInsetsAsMargin(View view, @InsetEdge int edges) {
    ViewCompat.setOnApplyWindowInsetsListener(
        view,
        (v, windowInsets) -> {
          Insets contentInsets = getContentInsets(windowInsets);
          Insets insets = applyEdges(contentInsets, edges);
          MarginLayoutParams params = (MarginLayoutParams) view.getLayoutParams();

          Insets originalMargins = (Insets) view.getTag(R.id.edge_to_edge_original_margin_tag);
          if (originalMargins == null) {
            originalMargins =
                Insets.of(
                    params.leftMargin, params.topMargin, params.rightMargin, params.bottomMargin);
            view.setTag(R.id.edge_to_edge_original_margin_tag, originalMargins);
          }
          insets = Insets.add(insets, originalMargins);
          params.setMargins(insets.left, insets.top, insets.right, insets.bottom);
          view.setLayoutParams(params);
          return windowInsets;
        });
  }

  private static Insets getContentInsets(WindowInsetsCompat windowInsets) {
    return windowInsets.getInsets(
        WindowInsetsCompat.Type.systemBars()
            | WindowInsetsCompat.Type.navigationBars()
            | WindowInsetsCompat.Type.displayCutout());
  }

  private static Insets applyEdges(Insets insets, @InsetEdge int edges) {
    if (edges == InsetEdge.ALL) {
      return insets;
    }
    int top = 0;
    int bottom = 0;
    int left = 0;
    int right = 0;
    if ((edges & InsetEdge.TOP) != 0) {
      top = insets.top;
    }
    if ((edges & InsetEdge.BOTTOM) != 0) {
      bottom = insets.bottom;
    }
    if ((edges & InsetEdge.LEFT) != 0) {
      left = insets.left;
    }
    if ((edges & InsetEdge.RIGHT) != 0) {
      right = insets.right;
    }
    return Insets.of(left, top, right, bottom);
  }
}
