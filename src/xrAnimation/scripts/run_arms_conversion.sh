#!/usr/bin/env bash
set -euo pipefail

CATEGORY="arms"
SKELETON_BASENAME="wpn_hand_01"
ANIMATION_BASENAMES=(
  "wpn_hand_ak74u_hud_animation"
)

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/ozz_utils/bin/Debug"
TESTDATA_ROOT="${ROOT_DIR}/src/xrAnimation/tests/testdata"
OUTPUT_DIR="${TESTDATA_ROOT}/${CATEGORY}"
TEXTURE_DIR="/mnt/f/modding/Vanilla_Guns_noedits/unpacked_patches/basedata/textures"

mkdir -p "${OUTPUT_DIR}"

SKELETON_PATH="${ROOT_DIR}/res/testdata/${CATEGORY}/${SKELETON_BASENAME}.ogf"
SKELETON_OUTPUT="${OUTPUT_DIR}/${SKELETON_BASENAME}.ozz"

"${BUILD_DIR}/xray_to_ozz_converter" \
  skeleton \
  "${SKELETON_PATH}" \
  "${SKELETON_OUTPUT}"

for animation_base in "${ANIMATION_BASENAMES[@]}"; do
  ANIMATION_PATH="${ROOT_DIR}/res/testdata/${CATEGORY}/${animation_base}.omf"
  if [[ -f "${ANIMATION_PATH}" ]]; then
    "${BUILD_DIR}/xray_to_ozz_converter" \
      animation \
      "${ANIMATION_PATH}" \
      "${OUTPUT_DIR}/${animation_base}.ozz" \
      "${SKELETON_PATH}" \
      --optimize
  else
    echo "Warning: animation file not found: ${ANIMATION_PATH}" >&2
  fi
done

MESH_OUTPUT="${OUTPUT_DIR}/${SKELETON_BASENAME}_mesh.ozz"
"${BUILD_DIR}/xray_to_ozz_converter" \
  mesh \
  "${SKELETON_PATH}" \
  "${MESH_OUTPUT}"

BUNDLE_OUTPUT="${OUTPUT_DIR}/${SKELETON_BASENAME}.ozzx"
"${BUILD_DIR}/xray_to_ozz_converter" \
  bundle \
  "${SKELETON_PATH}" \
  "${BUNDLE_OUTPUT}"

viewer_args=(
  "--bundle=${BUNDLE_OUTPUT}"
  "--texture_root=${TEXTURE_DIR}"
)

if ((${#ANIMATION_BASENAMES[@]} > 0)); then
  FIRST_ANIMATION_OUTPUT="${OUTPUT_DIR}/${ANIMATION_BASENAMES[0]}.ozz"
  if [[ -f "${FIRST_ANIMATION_OUTPUT}" ]]; then
    viewer_args+=("--animation=${FIRST_ANIMATION_OUTPUT}")
  fi
fi

"${BUILD_DIR}/ozz_animation_viewer" "${viewer_args[@]}"
