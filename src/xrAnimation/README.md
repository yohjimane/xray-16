# X-Ray Animation / ozz-animation Integration

This module hosts the ongoing effort to move the OpenXRay animation toolchain and runtime over to [ozz-animation](https://github.com/guillaumeblanc/ozz-animation). It currently ships production-ready converters and validation tooling alongside a work-in-progress runtime façade (`OzzKinematics`) that mirrors the legacy `IKinematics` interface.

## Current State

- **Asset conversion**: The `xray_to_ozz_converter` CLI and helper scripts convert `.ogf/.omf` pairs into `.ozz` skeletons/animations plus `.ozzx` bundles. Outputs are regression-tested against Blender exports.
- **Visualization**: `ozz_animation_viewer` loads converted bundles, dumps bind-pose/animation data, and provides profiling overlays to compare with upstream ozz samples.
- **Runtime façade**: `OzzKinematics` evaluates bind pose and sampled animation, handles visibility masks, `CBoneInstance` callbacks, and additional bone transforms while conforming to `IKinematics` expectations.
- **Parity tests**: GoogleTest fixtures under `src/xrAnimation/tests` diff world-space transforms between legacy `CKinematics` and `OzzKinematics` for bind pose and sampled clips to ensure behaviour remains aligned.
- **Still TODO**: Bone picking, vertex enumeration, renderer wiring, and an engine-ready `.ozzx` visual remain in progress; the façade is not yet a drop-in replacement for gameplay actors.

## Key Components

- `OzzConversion.*`: Matrix/transform helpers shared by the converter, tests, and runtime façade.
- `OzzBundle.*`: Minimal reader/writer for `.ozzx` bundles (skeleton + mesh payload).
- `OzzKinematics.*`: `IKinematics` implementation backed by ozz runtime jobs (`LocalToModelJob`, sampling contexts, visibility masks).
- `tests/`: Converter smoke tests, `ozz_kinematics_tests`, and parity fixtures that compare against legacy assets.
- `tools/`: Viewer integration, converter CLI, and support scripts for running conversions on sample data.

## Building & Running Tools

```bash
cmake -S xray-16 -B xray-16/ozz_utils -DCMAKE_BUILD_TYPE=Debug
cmake --build xray-16/ozz_utils -j$(nproc)
```

- **Converters**: `xray-16/ozz_utils/bin/<cfg>/xray_to_ozz_converter`
- **Viewer**: `xray-16/ozz_utils/bin/<cfg>/ozz_animation_viewer --bundle=<path>.ozzx`

Helper scripts in the repository root (e.g. `run_stalker_hero_conversion.sh`) regenerate sample assets under `src/xrAnimation/tests/testdata/` and launch the viewer in verification modes.

## Runtime Usage Snapshot

```cpp
XRay::Animation::OzzKinematics kinematics;
if (kinematics.InitializeFromOzz("path/to/skeleton.ozz"))
{
    // Evaluate rest pose or sampled animation (locals come from an ozz SamplingJob)
    kinematics.CalculateBones(TRUE); // forces recompute this frame
    const Fmatrix& world = kinematics.LL_GetTransform(bone_id);
    // Feed matrices into renderer/physics as needed.
}
```

`OzzKinematics` owns ozz sampling context/cache buffers; multi-threaded sampling is on the roadmap, so avoid sharing an instance across threads without external synchronization.

## Tests

- Build tests with the same CMake cache as the tools.
- Run `ctest --output-on-failure` from the build directory or invoke binaries directly:
  - `ozz_kinematics_tests`
  - `xrAnimation_converter_tests`
- Parity fixtures require the testdata produced by the converter scripts; regenerate them when converter logic changes.

## Roadmap / Known Gaps

- Implement renderer-facing visual that consumes `.ozzx` bundles and feeds bone palettes to the existing model pool.
- Cover `.ozzx` runtime loading with dedicated tests (bundle hydration, material metadata, failure modes).
- Add pilot actor/HUD wiring to exercise callbacks, physics hooks, and animation events through the façade.
- Expand optimization tests to assert size wins for `--optimize` and detect pose drift on representative clips.
- Harden `OzzKinematics` for multi-threaded sampling (dedicated scratch buffers, deterministic update cadence).

## Resources

- `AGENT_DOCS.md`: Session context, workflows, and tooling quick reference.
- `AGENT_COMMANDS.md`: Ready-to-run snippets for gathering bind-pose and sampled animation data.
- `AGENT_NEXT_STEPS.md`: Rolling plan that tracks façade work, visual integration, and testing priorities.

## License

This module is part of the OpenXRay project and follows the same licensing terms.
