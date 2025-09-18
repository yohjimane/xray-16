#!/bin/bash

# Script to convert X-Ray assets to ozz format based on user's examples

# Set paths relative to the repo layout
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(realpath "${SCRIPT_DIR}/../../..")"
WORKSPACE_ROOT="$(realpath "${PROJECT_ROOT}/..")"

CONVERTER="${PROJECT_ROOT}/ozz_utils/bin/Debug/xray_to_ozz_converter"
GAMEDATA="${WORKSPACE_ROOT}/gamedata"
OUTPUT_DIR="${WORKSPACE_ROOT}/asset_tests"
LOGGING_DIR="${WORKSPACE_ROOT}/logs"

# Ensure output and logging directories exist
mkdir -p "$OUTPUT_DIR"
mkdir -p "$LOGGING_DIR"
cd "$OUTPUT_DIR"

# Convert skeleton
echo "Converting skeleton..."
"$CONVERTER" skeleton "$GAMEDATA/stalker_hero/stalker_hero_1.ogf" . > "$LOGGING_DIR/skeleton_conversion.log" 2>&1

# Convert animation  
echo "Converting animation..."
"$CONVERTER" animation "$GAMEDATA/critical_hit_grup_1.omf" . "$GAMEDATA/stalker_hero/stalker_hero_1.ogf" > "$LOGGING_DIR/animation_conversion.log" 2>&1

echo "Conversion complete. Files saved to $OUTPUT_DIR"
echo "Logs saved to $LOGGING_DIR"
