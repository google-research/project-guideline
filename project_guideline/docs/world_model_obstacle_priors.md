# World Model Obstacle Priors

Project Guideline currently builds obstacle warnings from a frame-local depth
point cloud and a clearance-zone occupancy map. That approach is fast and
auditable, but it treats all high-confidence depth points the same. A modern
vision or world model can add complementary evidence: object permanence,
semantic hazard type, motion prediction, and confidence estimates for regions
where monocular depth is noisy.

This document proposes a conservative integration path: keep the existing
geometry-first obstacle map as the safety-critical path, and let model-specific
adapters provide optional occupancy priors in world coordinates.

## Why This Is a Good Fit

Project Guideline already maintains the pieces needed by modern spatial models:

* timestamped camera poses from ARCore;
* monocular or ARCore depth;
* a top-down clearance zone;
* a control system that consumes obstacle positions rather than raw pixels.

That makes the occupancy map a natural boundary. It can fuse learned priors
with depth evidence while preserving the existing planner, audio feedback, and
logging surfaces.

## Candidate Model Families

### Practical On-Device Candidate: Depth Anything V2 Small

[Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2) is a
modern monocular depth foundation model. The Small checkpoint is published
under Apache-2.0 and is substantially more practical for mobile prototyping than
larger non-commercial checkpoints. It should be evaluated as a drop-in
replacement or auxiliary depth source behind the existing MediaPipe/TFLite depth
path.

Expected contribution path:

1. Convert the Small checkpoint to a Pixel-friendly runtime format.
2. Benchmark latency and thermals against the current `depth.tflite`.
3. Feed its point cloud through the existing depth alignment and occupancy map.
4. Optionally emit `WorldModelOccupancyPrior` records when the model is paired
   with semantic segmentation or temporal filtering.

### Research World Model Candidate: V-JEPA 2 / V-JEPA 2.1

[V-JEPA 2](https://github.com/facebookresearch/vjepa2) is an open-source
video-trained world model family aimed at physical understanding, prediction,
and planning. It is valuable for Project Guideline research because the app's
camera stream is egocentric and action-conditioned by the runner's motion.

V-JEPA-style models should initially live outside the real-time Android loop:

* simulator evaluation;
* logged-run analysis;
* temporal hazard prediction;
* candidate path or action scoring;
* distillation into a smaller on-device prior head.

The direct model is too large for the current Pixel real-time safety loop, so it
should not replace geometric obstacle detection. The safer path is to use it to
train or validate compact priors that can be fused by the occupancy map.

## Integration Contract

`WorldModelOccupancyPrior` is a small model-agnostic data contract:

* `position`: the estimated hazardous location in world `x/y` coordinates;
* `confidence`: model confidence that the location is relevant;
* `occupancy_weight`: how strongly the prior should contribute to a grid cell.

`OccupancyMap::UpdateOccupancyMap` now has an overload that accepts these
priors. The map still filters them through the runner-relative clearance zone
and a configurable confidence threshold, then converts them into the same
occupied-grid representation already used by the control system.

## Safety Notes

Learned priors must be treated as additional evidence, not as permission to
ignore depth or tracking failures. A model adapter should fail closed by
emitting no priors when frames are stale, confidence is low, or camera pose is
unavailable. The existing STOP behavior remains owned by the control system.

## Next Steps

1. Add a MediaPipe calculator that maps semantic model output into
   `WorldModelOccupancyPrior` records.
2. Add logging for priors so simulator and field runs can compare depth-only
   and world-model-assisted warnings.
3. Add an offline evaluator that replays logs through candidate model adapters.
4. Benchmark Depth Anything V2 Small and a distilled V-JEPA prior head on Pixel
   hardware before enabling any real-time path by default.
