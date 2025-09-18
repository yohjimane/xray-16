#!/bin/bash

# Script to convert X-Ray assets to ozz format based on user's examples

# Set paths
CONVERTER="/mnt/f/modding/claude_sessions/build-debug/bin/Debug/xray_to_ozz_converter"
GAMEDATA="/mnt/f/modding/claude_sessions/gamedata"
OUTPUT_DIR="/mnt/f/modding/claude_sessions/asset_tests"
LOGGING_DIR="/mnt/f/modding/claude_sessions/logs"

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