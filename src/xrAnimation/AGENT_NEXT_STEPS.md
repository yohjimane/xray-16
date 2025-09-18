# AGENT_NEXT_STEPS

## Align OMF → ozz Animation Conversion with Blender-XRay Workflow
- Blender-XRay bakes animations in local joint space using parent-inverted matrices plus a +90° X rotation (MATRIX_BONE) before writing keys. Our converter currently does `inv_bind * parent_bind`, which leaves animations in mixed spaces.
- In `ConvertSingleMotionWithBindMatrices`, switch to computing local transforms exactly like Blender-XRay: `local = parent_bind.inverted() * child_bind` (root uses the same +90° basis), then decompose.
- Apply the consistent axis remap `(X, Y, Z) → (X, -Z, Y)` and matching quaternion reorder/sign adjustments when writing translation and rotation keys (use `TransformConverter` helpers).
- Re-run the bind pose + animation frame comparisons (with `OZZ_SAMPLE_RATIO` for frame alignment) to confirm positional/rotational deltas drop near zero.
