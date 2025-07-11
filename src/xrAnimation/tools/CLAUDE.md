# X-Ray to ozz-animation Converter Tool

## Overview
Command-line tool for converting X-Ray Engine animation assets (OGF skeletons, OMF animations) to ozz-animation format.

## Usage

### Converting a Skeleton
```bash
xray_to_ozz_converter skeleton <input.ogf> <output.ozz>
```

### Converting an Animation
```bash
xray_to_ozz_converter animation <input.omf> <output.ozz> <skeleton.ozz> [-optimize]
```

### Batch Conversion
```bash
xray_to_ozz_converter batch <input_dir> <output_dir> <skeleton.ozz> [-optimize]
```

## Features

### Skeleton Conversion (OGF → ozz)
- Parses OGF_S_BONE_NAMES chunk for bone hierarchy
- Correctly reads parent-child relationships  
- Supports IK data and bind poses from OGF_S_IKDATA chunk
- Preserves bone OBBs (oriented bounding boxes)
- Outputs metadata file with X-Ray specific data

### Animation Conversion (OMF → ozz)
- Reads compressed motion data from OMF files
- Supports both 8-bit and 16-bit compression
- Handles TCB (Tension-Continuity-Bias) interpolation
- Preserves motion parameters (speed, power, accrue, falloff)
- Supports animation optimization via ozz optimizer

### Batch Processing
- Automatically finds all OMF files in directory
- Also converts any OGF skeleton files found
- Progress reporting for large batches

## File Format Support

### OGF (Object Geometry Format)
- Chunk-based format
- Supported chunks:
  - OGF_S_BONE_NAMES (0x0D): Bone hierarchy
  - OGF_S_IKDATA (0x10): IK constraints and bind poses
  - OGF_S_USERDATA (0x11): User metadata
  - OGF_S_MOTIONS (0x0E): Motion references
  - OGF_S_SMPARAMS (0x0F): Motion parameters

### OMF (Object Motion Format)
- Compressed animation format
- Version 4 supported
- Per-bone motion channels
- Event markers and motion flags

## Implementation Details

### Bone Hierarchy
- Parent relationships stored as string names in OGF
- Converted to parent indices for ozz
- Root bones have empty parent name
- Multiple root bones generate warning

### Compression
- X-Ray uses quantized quaternions (16-bit per component)
- Translation can be 8-bit or 16-bit compressed
- ozz optimizer can further reduce data size

### Metadata Preservation
- Motion parameters saved to .meta files
- IK constraints preserved for physics
- Event markers maintained for gameplay

## Error Handling
- Validates file formats before conversion
- Reports unsupported versions
- Handles missing parent bones gracefully
- Provides detailed error messages

## Build Requirements
- ozz-animation library
- X-Ray Engine xrCore
- C++17 compiler

## Known Limitations
- SDK format converters (.skl, .skls) not yet implemented
- Some advanced IK features may need manual adjustment
- Large skeletons (>64 bones) require special handling