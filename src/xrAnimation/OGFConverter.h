#pragma once

#include "AnimationConverter.h"
#include "xrCore/stream_reader.h"
#include "xrCore/FS.h"
#include "xrCore/_fbox.h"
#include "xrCore/_sphere.h"
#include "xrCore/_matrix.h"
#include "xrCore/_quaternion.h"
#include "xrCore/_vector2.h"
#include "xrCore/xr_ini.h"
#include "xrCore/FMesh.hpp"
#include "xrCommon/xr_vector.h"
#include "xrCommon/xr_map.h"
#include "xrCommon/xr_smart_pointers.h"
#include "xrCore/Animation/Bone.hpp"  // For SBoneShape

namespace XRay {
namespace Animation {

// OGF chunk types (from FMesh.hpp)
const u32 OGF_HEADER = 1;
const u32 OGF_S_BONE_NAMES = 13;
const u32 OGF_S_MOTIONS = 14;
const u32 OGF_S_SMPARAMS = 15;
const u32 OGF_S_IKDATA = 16;
const u32 OGF_S_USERDATA = 17;
const u32 OGF_S_DESC = 18;

// XRay format specifications
struct XRayFormatSpec {
    struct SkeletalVertex {
        Fvector position, normal, tangent, binormal;
        Fvector2 uv;
        struct BoneWeight {
            u16 bone_index;
            float weight;
        };
        BoneWeight bones[4];
    };

    struct MotionKeyframe {
        float value;
        float time;
        u8 shape;
        float tension, continuity, bias;
        float param[4];
    };

    struct MotionChannel {
        enum Type { PosX=0, PosY=1, PosZ=2, RotH=3, RotP=4, RotB=5, ctMaxChannel=6 };
        xr_vector<MotionKeyframe> keyframes;
        int behavior[2];
    };

    struct BoneMotion {
        shared_str name;
        MotionChannel channels[6];
        u8 flags;
    };

    struct MotionParams {
        float speed = 1.0f;
        float power = 1.0f;
        float accrue = 0.2f;
        float falloff = 0.2f;
        u16 bone_or_part = 0;
        u8 flags = 0;
        xr_vector<float> event_markers;
    };
};

class OGFReader {
public:
    struct OGFHeader {
        u8 format_version;
        u8 type;
        u16 shader_id;
        Fbox bb;
        Fsphere bs;
    };

    struct ChunkHeader {
        u32 type;
        u32 size;
    };

    bool LoadFromFile(const shared_str& file_path);
    bool LoadFromMemory(const void* data, size_t size);

    bool FindChunk(u32 chunk_type);
    xr_unique_ptr<IReader> ReadChunk(u32 chunk_type);
    void DebugListChunks();

    struct BoneData {
        shared_str name;
        shared_str parent_name;
        Fobb obb;
        Fmatrix bind_transform;
        float mass;
        Fvector center_of_mass;
    };

    xr_vector<BoneData> ReadBoneData();
    xr_vector<XRayFormatSpec::BoneMotion> ReadMotionData();
    XRayFormatSpec::MotionParams ReadMotionParams();
    xr_vector<XRayMetadata::IKConstraints> ReadIKData();
    xr_map<shared_str, shared_str> ReadUserData();

    const OGFHeader& GetHeader() const { return header_; }

private:
    xr_unique_ptr<IReader> reader_;
    OGFHeader header_;
    xr_vector<u8> file_data_;  // Store file data for memory reader

    void ReadHeader();
    void ParseChunkData(u32 chunk_type, IReader& chunk_reader);
};

class XRayCompressionDecoder {
public:
    struct CompressedQuaternion {
        s16 x, y, z, w;

        Fquaternion Decompress() const {
            const float scale = 1.0f / 32767.0f;
            return Fquaternion().set(x * scale, y * scale, z * scale, w * scale).normalize();
        }
    };

    struct CompressedTranslation {
        union {
            struct { s8 x, y, z; } t8;
            struct { s16 x, y, z; } t16;
        };

        Fvector Decompress(const Fvector& init, const Fvector& size, bool is16bit) const {
            if (is16bit) {
                const float scale = 1.0f / 32767.0f;
                Fvector result;
                return result.set(init.x + t16.x * scale * size.x,
                                 init.y + t16.y * scale * size.y,
                                 init.z + t16.z * scale * size.z);
            } else {
                const float scale = 1.0f / 127.0f;
                Fvector result;
                return result.set(init.x + t8.x * scale * size.x,
                                 init.y + t8.y * scale * size.y,
                                 init.z + t8.z * scale * size.z);
            }
        }
    };

    enum MotionFlags {
        flTKeyPresent = (1 << 0),
        flRKeyAbsent = (1 << 1),
        flTKey16IsBit = (1 << 2)
    };

    xr_vector<XRayFormatSpec::MotionKeyframe> DecompressMotionKeys(
        IReader& reader, const MotionFlags& flags);
};

class OGFSkeletonParser {
public:
    struct ParseResult {
        xr_vector<shared_str> bone_names;
        xr_vector<s16> parent_indices;
        xr_vector<Fmatrix> bind_poses;  // World space bind poses
        xr_vector<Fmatrix> local_transforms;  // Local space transforms from IK data
        xr_vector<XRayFormatSpec::BoneMotion> motions;
        XRayFormatSpec::MotionParams motion_params;
        xr_vector<XRayMetadata::IKConstraints> ik_data;
        xr_map<shared_str, shared_str> user_data;
        u16 root_bone_index;
    };

    ParseResult Parse(OGFReader& reader);

private:
    void ParseS_BONE_NAMES(IReader& chunk_reader, ParseResult& result);
    void ParseS_MOTIONS(IReader& chunk_reader, ParseResult& result);
    void ParseS_SMPARAMS(IReader& chunk_reader, ParseResult& result);
    void ParseS_IKDATA(IReader& chunk_reader, ParseResult& result);
    void ParseS_USERDATA(IReader& chunk_reader, ParseResult& result);
    void ParseS_DESC(IReader& chunk_reader, ParseResult& result);

    XRayCompressionDecoder compression_decoder_;
};

class OGFToOzzSkeletonConverter {
public:
    ozz::animation::offline::RawSkeleton ConvertSkeleton(
        const OGFSkeletonParser::ParseResult& ogf_data
    );

private:
    struct BoneInfo {
        shared_str name;
        s16 parent_index;
        Fmatrix bind_pose;
        XRayMetadata::IKConstraints ik_constraints;
        XRayMetadata::PhysicsShape physics_shape;
    };

    void BuildBoneHierarchy(
        const xr_vector<shared_str>& bone_names,
        const xr_vector<s16>& parent_indices,
        const xr_vector<Fmatrix>& bind_poses,
        const xr_vector<Fmatrix>& local_transforms,
        ozz::animation::offline::RawSkeleton& skeleton
    );

    ozz::math::Transform ConvertBindPose(const Fmatrix& xray_matrix);
    bool ValidateBoneHierarchy(const xr_vector<BoneInfo>& bones);
    xr_vector<size_t> ComputeDepthFirstOrder(const xr_vector<BoneInfo>& bones);
};

class OGFToOzzAnimationConverter {
public:
    xr_vector<ozz::animation::offline::RawAnimation> ConvertAnimations(
        const xr_vector<XRayFormatSpec::BoneMotion>& xray_motions,
        const xr_vector<shared_str>& bone_names,
        const XRayFormatSpec::MotionParams& motion_params
    );

private:
    struct ConvertedKeyframes {
        xr_vector<float> times;
        xr_vector<ozz::math::Float3> translations;
        xr_vector<ozz::math::Quaternion> rotations;
        xr_vector<ozz::math::Float3> scales;
    };

    ConvertedKeyframes ConvertBoneMotion(
        const XRayFormatSpec::BoneMotion& bone_motion,
        float motion_length
    );

    void ConvertInterpolation(
        const xr_vector<XRayFormatSpec::MotionKeyframe>& xray_keys,
        xr_vector<float>& times,
        xr_vector<float>& values
    );

    float EvaluateTCBInterpolation(
        const XRayFormatSpec::MotionKeyframe& key0,
        const XRayFormatSpec::MotionKeyframe& key1,
        float t
    );

    float EvaluateHermiteInterpolation(
        const XRayFormatSpec::MotionKeyframe& key0,
        const XRayFormatSpec::MotionKeyframe& key1,
        float t
    );

    void BuildTransformTracks(
        const XRayFormatSpec::MotionChannel channels[6],
        ConvertedKeyframes& result
    );

    ozz::math::Quaternion ConvertHPBToQuaternion(float h, float p, float b);
};

class OGFConverter : public IFormatConverter {
public:
    ConversionResult Convert(const shared_str& input_path) override;
    bool CanHandle(const shared_str& file_extension) override {
        return file_extension == ".ogf";
    }

private:
    OGFReader reader_;
    OGFSkeletonParser parser_;
    OGFToOzzSkeletonConverter skeleton_converter_;
    OGFToOzzAnimationConverter animation_converter_;

    XRayMetadata ExtractMetadata(const OGFSkeletonParser::ParseResult& parse_result);
};

} // namespace Animation
} // namespace XRay
