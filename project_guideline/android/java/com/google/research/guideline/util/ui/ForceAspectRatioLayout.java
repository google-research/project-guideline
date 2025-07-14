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

import android.content.Context;
import android.util.AttributeSet;
import android.widget.FrameLayout;
import androidx.annotation.Nullable;

/**
 * Layout that can force a specific aspect ratio.
 *
 * <p>By default this layout is a no-op. {@link #forceAspectRatio} must be invoked to force the
 * desired aspect ratio.
 */
public final class ForceAspectRatioLayout extends FrameLayout {
  private boolean forceAspectRatio = false;
  private float aspectRatio;

  public ForceAspectRatioLayout(Context context) {
    super(context);
  }

  public ForceAspectRatioLayout(Context context, @Nullable AttributeSet attrs) {
    super(context, attrs);
  }

  public ForceAspectRatioLayout(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
    super(context, attrs, defStyleAttr);
  }

  public ForceAspectRatioLayout(
      Context context, @Nullable AttributeSet attrs, int defStyleAttr, int defStyleRes) {
    super(context, attrs, defStyleAttr, defStyleRes);
  }

  /** Forces this layout to have the given aspect ratio. */
  public void forceAspectRatio(float aspectRatio) {
    this.forceAspectRatio = true;
    this.aspectRatio = aspectRatio;
    requestLayout();
  }

  @Override
  protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
    if (!forceAspectRatio) {
      super.onMeasure(widthMeasureSpec, heightMeasureSpec);
      return;
    }

    int height = MeasureSpec.getSize(heightMeasureSpec);
    int width = MeasureSpec.getSize(widthMeasureSpec);

    float availableAspectRatio = (float) width / height;

    int desiredWidth;
    int desiredHeight;
    if (aspectRatio > availableAspectRatio) {
      desiredWidth = width;
      desiredHeight = (int) (width / aspectRatio);
    } else {
      desiredHeight = height;
      desiredWidth = (int) (height * aspectRatio);
    }

    super.onMeasure(
        MeasureSpec.makeMeasureSpec(desiredWidth, MeasureSpec.EXACTLY),
        MeasureSpec.makeMeasureSpec(desiredHeight, MeasureSpec.EXACTLY));
  }
}
