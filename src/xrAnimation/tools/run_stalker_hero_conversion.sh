#!/usr/bin/env bash
set -euo pipefail

# Convert the stalker hero bind skeleton to ozz format
# Paths relative to repository root
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/ozz_utils/bin/Debug"
TESTDATA_DIR="${ROOT_DIR}/src/xrAnimation/tests/testdata"

mkdir -p "${TESTDATA_DIR}"

"${BUILD_DIR}/xray_to_ozz_converter" \
  skeleton \
  "${ROOT_DIR}/res/testdata/stalker_hero_1.ogf" \
  "${TESTDATA_DIR}/stalker_hero_1.ozz"

# Convert the critical_hit_grup_1 animation using the same skeleton
"${BUILD_DIR}/xray_to_ozz_converter" \
  animation \
  "${ROOT_DIR}/res/testdata/critical_hit_grup_1.omf" \
  "${TESTDATA_DIR}/critical_hit_grup_1.ozz" \
  "${ROOT_DIR}/res/testdata/stalker_hero_1.ogf"

# Launch the viewer with the converted assets
"${BUILD_DIR}/ozz_animation_viewer" \
  --skeleton="${TESTDATA_DIR}/stalker_hero_1.ozz" \
  --animation="${TESTDATA_DIR}/critical_hit_grup_1.ozz" \
  --mesh="${TESTDATA_DIR}/stalker_hero_mesh.ozz"
