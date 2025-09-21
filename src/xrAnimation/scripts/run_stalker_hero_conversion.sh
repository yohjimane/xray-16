#!/usr/bin/env bash
set -euo pipefail

# Convert the stalker hero bind skeleton to ozz format
# Paths relative to repository root
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/ozz_utils/bin/Debug"
TESTDATA_DIR="${ROOT_DIR}/src/xrAnimation/tests/testdata"
TEXTURE_DIR="/mnt/f/modding/Vanilla_Guns_noedits/unpacked_patches/basedata/textures"

mkdir -p "${TESTDATA_DIR}"

"${BUILD_DIR}/xray_to_ozz_converter" \
  skeleton \
  "${ROOT_DIR}/res/testdata/npc/stalker_hero_1.ogf" \
  "${TESTDATA_DIR}/stalker_hero_1.ozz"

# Convert the critical_hit_grup_1 animation using the same skeleton
"${BUILD_DIR}/xray_to_ozz_converter" \
  animation \
  "${ROOT_DIR}/res/testdata/npc/critical_hit_grup_1.omf" \
  "${TESTDATA_DIR}/critical_hit_grup_1.ozz" \
  "${ROOT_DIR}/res/testdata/npc/stalker_hero_1.ogf"

# Convert the stalker hero mesh for skinning validation
"${BUILD_DIR}/xray_to_ozz_converter" \
  mesh \
  "${ROOT_DIR}/res/testdata/npc/stalker_hero_1.ogf" \
  "${TESTDATA_DIR}/stalker_hero_mesh.ozz"

# Launch the viewer with the converted assets
"${BUILD_DIR}/ozz_animation_viewer" \
  --skeleton="${TESTDATA_DIR}/stalker_hero_1.ozz" \
  --mesh="${TESTDATA_DIR}/stalker_hero_mesh.ozz" \
  --animation="${TESTDATA_DIR}/critical_hit_grup_1.ozz" \
  --texture_root="${TEXTURE_DIR}" \
  #--dump_skinning_json="${TESTDATA_DIR}/stalker_hero_mesh_skinning.json" \
  #--render=false \
  #--max_idle_loops=2
