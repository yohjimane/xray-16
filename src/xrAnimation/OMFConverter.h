#pragma once

#include "AnimationConverter.h"
#include "xrCore/stream_reader.h"
#include "xrCore/FS.h"
#include "xrCore/_matrix.h"
#include "xrCore/_quaternion.h"
#include "xrCore/_vector3d.h"
#include "xrCommon/xr_vector.h"
#include "xrCommon/xr_map.h"
#include "xrCommon/xr_smart_pointers.h"

namespace XRay {
namespace Animation {

// OMF (Object Motion File) format constants
const u32 OMF_CHUNK_VERSION = 0x0900;
const u32 OMF_CHUNK_PARAMS = 0x0901;
const u32 OMF_CHUNK_MOTIONS = 0x0902;
const u32 OMF_VERSION = 4;

// Motion flags from X-Ray
enum EMotionFlags {
    esmFX = (1 << 0),
    esmStopAtEnd = (1 << 1),
    esmNoMix = (1 << 2),
    esmSyncPart = (1 << 3),
    esmUseFootSteps = (1 << 4),
    esmRootMover = (1 << 5),
    esmAPlatform = (1 << 6),
    esmUseWeaponBone = (1 << 7),
    esmFXBitMask = (esmFX | esmSyncPart)
};

// Key flags for compressed animation data
enum EKeyFlags {
    flTKeyPresent = (1 << 0),
    flRKeyAbsent = (1 << 1),
    flTKey16IsBit = (1 << 2)
};

struct OMFMotionDef {
    shared_str name;
    u16 bone_or_part;
    u16 motion;
    float speed;
    float power;
    float accrue;
    float falloff;
    u32 flags;
    u32 marks_count;
    xr_vector<float> marks;  // Event markers
};

struct OMFBoneMotion {
    shared_str name;
    u32 flags;
    
    // Compressed motion data
    xr_vector<u16> keys_translation_frames;  // Frame indices for translation keys
    xr_vector<Fvector> keys_translation;      // Translation values
    
    xr_vector<u16> keys_rotation_frames;     // Frame indices for rotation keys
    xr_vector<Fquaternion> keys_rotation;     // Rotation values (as quaternions)
    
    // Envelope data for curves
    struct Envelope {
        u8 behavior[2];  // Pre/post behavior
        xr_vector<float> keys;
    };
    
    Envelope envelopes[6];  // For 6 channels (PosX, PosY, PosZ, RotH, RotP, RotB)
};

class OMFReader {
public:
    struct OMFHeader {
        u32 version;
        u16 motion_count;
    };
    
    bool LoadFromFile(const shared_str& file_path);
    bool LoadFromMemory(const void* data, size_t size);
    
    const OMFHeader& GetHeader() const { return header_; }
    
    xr_vector<OMFMotionDef> ReadMotionDefs();
    xr_vector<OMFBoneMotion> ReadBoneMotions();
    
private:
    xr_unique_ptr<IReader> reader_;
    OMFHeader header_;
    
    void ReadHeader();
    void ReadMotionParams(IReader& reader, OMFMotionDef& motion_def);
    void ReadCompressedMotion(IReader& reader, OMFBoneMotion& bone_motion);
    
    // Decompression helpers
    Fquaternion DecompressQuaternion(u64 packed) const;
    Fvector DecompressTranslation(u32 packed, const Fvector& init, const Fvector& size) const;
    void DecompressMotionKeys(IReader& reader, u32 flags, OMFBoneMotion& motion);
};

class OMFToOzzAnimationConverter {
public:
    struct ConversionParams {
        float fps;  // Default FPS for X-Ray animations
        bool optimize;
        float position_threshold;
        float rotation_threshold;
        float scale_threshold;
        
        ConversionParams() 
            : fps(30.0f)
            , optimize(true)
            , position_threshold(0.001f)
            , rotation_threshold(0.001f)
            , scale_threshold(0.001f)
        {}
    };
    
    xr_vector<ozz::animation::offline::RawAnimation> ConvertAnimations(
        const xr_vector<OMFBoneMotion>& omf_motions,
        const xr_vector<shared_str>& bone_names,
        const ConversionParams& params = ConversionParams()
    );
    
    ozz::animation::offline::RawAnimation ConvertSingleMotion(
        const OMFBoneMotion& omf_motion,
        const xr_vector<shared_str>& bone_names,
        const ConversionParams& params = ConversionParams()
    );
    
private:
    struct KeyframeData {
        xr_vector<float> times;
        xr_vector<ozz::math::Float3> translations;
        xr_vector<ozz::math::Quaternion> rotations;
        xr_vector<ozz::math::Float3> scales;
    };
    
    KeyframeData ExtractKeyframes(
        const OMFBoneMotion& omf_motion,
        float fps
    );
    
    void BuildAnimationTrack(
        const KeyframeData& keyframes,
        ozz::animation::offline::RawAnimation::JointTrack& track
    );
    
    float CalculateAnimationDuration(
        const OMFBoneMotion& omf_motion,
        float fps
    );
    
    // Interpolation helpers
    ozz::math::Float3 InterpolateTranslation(
        const xr_vector<Fvector>& keys,
        const xr_vector<u16>& frames,
        u16 current_frame
    );
    
    ozz::math::Quaternion InterpolateRotation(
        const xr_vector<Fquaternion>& keys,
        const xr_vector<u16>& frames,
        u16 current_frame
    );
};

class OMFConverter : public IFormatConverter {
public:
    ConversionResult Convert(const shared_str& input_path) override;
    
    bool CanHandle(const shared_str& file_extension) override {
        return file_extension == ".omf";
    }
    
    // Additional method to convert with skeleton reference
    ConversionResult ConvertWithSkeleton(
        const shared_str& input_path,
        const ozz::animation::offline::RawSkeleton& skeleton
    );
    
private:
    OMFReader reader_;
    OMFToOzzAnimationConverter converter_;
    
    XRayMetadata ExtractMetadata(
        const xr_vector<OMFMotionDef>& motion_defs
    );
    
    xr_vector<shared_str> ExtractBoneNamesFromSkeleton(
        const ozz::animation::offline::RawSkeleton& skeleton
    );
};

} // namespace Animation
} // namespace XRay