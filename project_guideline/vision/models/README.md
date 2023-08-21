# Guideline Models and Model Cards

## Segmentation

The Guideline segmentation model is used to detect a purple line ground marking
(approximate RGB [144, 99, 205]).

* [TFLite model](https://github.com/google-research/project-guideline/blob/main/project_guideline/vision/models/guideline.tflite)
* [Model card](https://github.com/google-research/project-guideline/blob/main/project_guideline/vision/models/docs/guideline_segmentation_model_card.pdf)

## Depth

The depth model estimates depth from monocular images. It is used to detect
potential obstacles along the user's path.

* [TFLite model](https://github.com/google-research/project-guideline/blob/main/project_guideline/vision/models/guideline.tflite)
* [Model card](https://github.com/google-research/project-guideline/blob/main/project_guideline/vision/models/docs/guideline_depth_model_card.pdf)
