#include "stdafx.h"
#include "OGFConverter.h"
#include "xrCore/file_stream_reader.h"
#include "xrCore/FS.h"

namespace XRay {
namespace Animation {

// X-Ray OGF constants
const u16 xrOGF_SMParamsVersion = 4;
const u8 MT_SKELETON_ANIM = 3;
const u8 MT_SKELETON_RIGID = 4;

bool OGFReader::LoadFromFile(const shared_str& file_path) {
    if (!FS.exist(file_path.c_str())) {
        return false;
    }

    reader_.reset(FS.r_open(file_path.c_str()));
    if (!reader_) {
        return false;
    }

    ReadHeader();
    return true;
}

bool OGFReader::LoadFromMemory(const void* data, size_t size) {
    if (!data || size == 0) {
        return false;
    }

    reader_.reset(xr_new<IReader>(const_cast<void*>(data), size));
    if (!reader_) {
        return false;
    }

    ReadHeader();
    return true;
}

void OGFReader::ReadHeader() {
    if (!reader_) {
        return;
    }

    header_.format_version = reader_->r_u8();
    header_.type = reader_->r_u8();
    header_.shader_id = reader_->r_u16();

    // Read bounding box
    reader_->r_fvector3(header_.bb.vMin);
    reader_->r_fvector3(header_.bb.vMax);

    // Read bounding sphere
    reader_->r_fvector3(header_.bs.P);
    header_.bs.R = reader_->r_float();
}

bool OGFReader::FindChunk(u32 chunk_type) {
    if (!reader_) {
        return false;
    }

    reader_->seek(sizeof(OGFHeader));

    while (!reader_->eof()) {
        ChunkHeader chunk_header;
        chunk_header.type = reader_->r_u32();
        chunk_header.size = reader_->r_u32();

        if (chunk_header.type == chunk_type) {
            return true;
        }

        reader_->advance(chunk_header.size);
    }

    return false;
}

xr_unique_ptr<IReader> OGFReader::ReadChunk(u32 chunk_type) {
    if (!FindChunk(chunk_type)) {
        return nullptr;
    }

    /*u32 chunk_size = */reader_->r_u32();
    reader_->seek(reader_->tell() - sizeof(u32));

    auto result = reader_->open_chunk(chunk_type);
    return xr_unique_ptr<IReader>(result);
}

xr_vector<shared_str> OGFReader::ReadBoneNames() {
    xr_vector<shared_str> bone_names;

    auto chunk_reader = ReadChunk(OGF_S_BONE_NAMES);
    if (!chunk_reader) {
        return bone_names;
    }

    u32 bone_count = chunk_reader->r_u32();
    bone_names.reserve(bone_count);

    for (u32 i = 0; i < bone_count; ++i) {
        shared_str bone_name;
        chunk_reader->r_stringZ(bone_name);
        bone_names.push_back(bone_name);
    }

    return bone_names;
}

xr_vector<XRayFormatSpec::BoneMotion> OGFReader::ReadMotionData() {
    xr_vector<XRayFormatSpec::BoneMotion> motions;

    auto chunk_reader = ReadChunk(OGF_S_MOTIONS);
    if (!chunk_reader) {
        return motions;
    }

    u32 motion_count = chunk_reader->r_u32();
    motions.reserve(motion_count);

    for (u32 i = 0; i < motion_count; ++i) {
        XRayFormatSpec::BoneMotion motion;
        chunk_reader->r_stringZ(motion.name);
        motion.flags = chunk_reader->r_u8();

        // Read 6 channels (PosX, PosY, PosZ, RotH, RotP, RotB)
        for (int channel = 0; channel < 6; ++channel) {
            auto& ch = motion.channels[channel];

            // Read behavior
            ch.behavior[0] = chunk_reader->r_u32();
            ch.behavior[1] = chunk_reader->r_u32();

            // Read keyframes
            u32 keyframe_count = chunk_reader->r_u32();
            ch.keyframes.reserve(keyframe_count);

            for (u32 k = 0; k < keyframe_count; ++k) {
                XRayFormatSpec::MotionKeyframe keyframe;
                keyframe.value = chunk_reader->r_float();
                keyframe.time = chunk_reader->r_float();
                keyframe.shape = chunk_reader->r_u8();
                keyframe.tension = chunk_reader->r_float();
                keyframe.continuity = chunk_reader->r_float();
                keyframe.bias = chunk_reader->r_float();

                for (int p = 0; p < 4; ++p) {
                    keyframe.param[p] = chunk_reader->r_float();
                }

                ch.keyframes.push_back(keyframe);
            }
        }

        motions.push_back(motion);
    }

    return motions;
}

XRayFormatSpec::MotionParams OGFReader::ReadMotionParams() {
    XRayFormatSpec::MotionParams params;

    auto chunk_reader = ReadChunk(OGF_S_SMPARAMS);
    if (!chunk_reader) {
        return params;
    }

    u16 version = chunk_reader->r_u16();
    if (version != xrOGF_SMParamsVersion) {
        Msg("! Unsupported motion params version: %d", version);
        return params;
    }

    params.bone_or_part = chunk_reader->r_u16();
    params.speed = chunk_reader->r_float();
    params.power = chunk_reader->r_float();
    params.accrue = chunk_reader->r_float();
    params.falloff = chunk_reader->r_float();
    params.flags = chunk_reader->r_u8();

    // Read event markers
    u32 marker_count = chunk_reader->r_u32();
    params.event_markers.reserve(marker_count);

    for (u32 i = 0; i < marker_count; ++i) {
        params.event_markers.push_back(chunk_reader->r_float());
    }

    return params;
}

xr_vector<XRayMetadata::IKConstraints> OGFReader::ReadIKData() {
    xr_vector<XRayMetadata::IKConstraints> ik_data;

    auto chunk_reader = ReadChunk(OGF_S_IKDATA);
    if (!chunk_reader) {
        return ik_data;
    }

    u32 ik_count = chunk_reader->r_u32();
    ik_data.reserve(ik_count);

    for (u32 i = 0; i < ik_count; ++i) {
        XRayMetadata::IKConstraints ik;

        ik.type = static_cast<XRayMetadata::IKConstraints::JointType>(chunk_reader->r_u32());

        for (int j = 0; j < 3; ++j) {
            chunk_reader->r_fvector2(ik.limits[j].limit);
            ik.limits[j].spring_factor = chunk_reader->r_float();
            ik.limits[j].damping_factor = chunk_reader->r_float();
        }

        ik.break_force = chunk_reader->r_float();
        ik.break_torque = chunk_reader->r_float();
        ik.friction = chunk_reader->r_float();
        ik.flags = chunk_reader->r_u32();

        ik_data.push_back(ik);
    }

    return ik_data;
}

xr_map<shared_str, shared_str> OGFReader::ReadUserData() {
    xr_map<shared_str, shared_str> user_data;

    auto chunk_reader = ReadChunk(OGF_S_USERDATA);
    if (!chunk_reader) {
        return user_data;
    }

    // Read INI data from chunk
    u32 data_size = chunk_reader->length();
    if (data_size > 0) {
        xr_vector<char> ini_data(data_size + 1);
        chunk_reader->r(ini_data.data(), data_size);
        ini_data[data_size] = 0;

        // Parse INI data manually for simplicity
        shared_str ini_string(ini_data.data());
        // TODO: Add proper INI parsing here
        user_data[shared_str("raw_ini")] = ini_string;
    }

    return user_data;
}

OGFSkeletonParser::ParseResult OGFSkeletonParser::Parse(OGFReader& reader) {
    ParseResult result;

    // Parse bone names
    result.bone_names = reader.ReadBoneNames();

    // Parse motion data
    result.motions = reader.ReadMotionData();

    // Parse motion parameters
    result.motion_params = reader.ReadMotionParams();

    // Parse IK data
    result.ik_data = reader.ReadIKData();

    // Parse user data
    result.user_data = reader.ReadUserData();

    // Build parent indices (simplified - would need proper hierarchy parsing)
    result.parent_indices.resize(result.bone_names.size());
    result.bind_poses.resize(result.bone_names.size());

    for (size_t i = 0; i < result.bone_names.size(); ++i) {
        result.parent_indices[i] = (i == 0) ? -1 : static_cast<s16>(i - 1);
        result.bind_poses[i].identity();
    }

    return result;
}

ozz::animation::offline::RawSkeleton OGFToOzzSkeletonConverter::ConvertSkeleton(
    const OGFSkeletonParser::ParseResult& ogf_data
) {
    ozz::animation::offline::RawSkeleton skeleton;

    if (ogf_data.bone_names.empty()) {
        return skeleton;
    }

    // Build bone hierarchy
    BuildBoneHierarchy(
        ogf_data.bone_names,
        ogf_data.parent_indices,
        ogf_data.bind_poses,
        skeleton
    );

    return skeleton;
}

void OGFToOzzSkeletonConverter::BuildBoneHierarchy(
    const xr_vector<shared_str>& bone_names,
    const xr_vector<s16>& parent_indices,
    const xr_vector<Fmatrix>& bind_poses,
    ozz::animation::offline::RawSkeleton& skeleton
) {
    // Create joint map for hierarchy building
    xr_vector<ozz::animation::offline::RawSkeleton::Joint*> joints(bone_names.size());

    // Create all joints first
    for (size_t i = 0; i < bone_names.size(); ++i) {
        auto* joint = new ozz::animation::offline::RawSkeleton::Joint();
        joint->name = bone_names[i].c_str();
        joint->transform = TransformConverter::XRayToOzz(bind_poses[i]);
        joints[i] = joint;
    }

    // Build hierarchy
    for (size_t i = 0; i < bone_names.size(); ++i) {
        s16 parent_index = parent_indices[i];

        if (parent_index == -1) {
            // Root joint
            skeleton.roots.push_back(*joints[i]);
        } else if (parent_index < static_cast<s16>(joints.size())) {
            // Child joint
            joints[parent_index]->children.push_back(*joints[i]);
        }
    }

    // Clean up temporary joint pointers
    for (auto* joint : joints) {
        delete joint;
    }
}

ozz::math::Transform OGFToOzzSkeletonConverter::ConvertBindPose(const Fmatrix& xray_matrix) {
    return TransformConverter::XRayToOzz(xray_matrix);
}

xr_vector<ozz::animation::offline::RawAnimation> OGFToOzzAnimationConverter::ConvertAnimations(
    const xr_vector<XRayFormatSpec::BoneMotion>& xray_motions,
    const xr_vector<shared_str>& bone_names,
    const XRayFormatSpec::MotionParams& motion_params
) {
    xr_vector<ozz::animation::offline::RawAnimation> animations;

    if (xray_motions.empty()) {
        return animations;
    }

    // For simplicity, create one animation from all motions
    ozz::animation::offline::RawAnimation animation;
    animation.name = "converted_animation";
    animation.duration = 1.0f; // Default duration

    // Create tracks for each bone
    animation.tracks.resize(bone_names.size());

    // Convert each bone motion
    for (size_t bone_idx = 0; bone_idx < bone_names.size() && bone_idx < xray_motions.size(); ++bone_idx) {
        const auto& bone_motion = xray_motions[bone_idx];
        auto& track = animation.tracks[bone_idx];

        // Convert translation channels (PosX, PosY, PosZ)
        for (int i = 0; i < 3; ++i) {
            const auto& channel = bone_motion.channels[i];
            for (const auto& keyframe : channel.keyframes) {
                ozz::animation::offline::RawAnimation::TranslationKey key;
                key.time = keyframe.time;

                // Set appropriate component
                ozz::math::Float3 translation = ozz::math::Float3::zero();
                if (i == 0) translation.x = keyframe.value;
                else if (i == 1) translation.y = keyframe.value;
                else if (i == 2) translation.z = keyframe.value;
                key.value = translation;

                track.translations.push_back(key);
            }
        }

        // Convert rotation channels (RotH, RotP, RotB)
        if (!bone_motion.channels[3].keyframes.empty()) {
            for (size_t k = 0; k < bone_motion.channels[3].keyframes.size(); ++k) {
                ozz::animation::offline::RawAnimation::RotationKey key;
                key.time = bone_motion.channels[3].keyframes[k].time;

                // Get HPB values
                float h = (k < bone_motion.channels[3].keyframes.size()) ? bone_motion.channels[3].keyframes[k].value : 0.0f;
                float p = (k < bone_motion.channels[4].keyframes.size()) ? bone_motion.channels[4].keyframes[k].value : 0.0f;
                float b = (k < bone_motion.channels[5].keyframes.size()) ? bone_motion.channels[5].keyframes[k].value : 0.0f;

                key.value = ConvertHPBToQuaternion(h, p, b);
                track.rotations.push_back(key);
            }
        }

        // Add default scale keys
        if (track.scales.empty()) {
            ozz::animation::offline::RawAnimation::ScaleKey key;
            key.time = 0.0f;
            key.value = ozz::math::Float3::one();
            track.scales.push_back(key);
        }
    }

    animations.push_back(animation);
    return animations;
}

ozz::math::Quaternion OGFToOzzAnimationConverter::ConvertHPBToQuaternion(float h, float p, float b) {
    // Convert HPB (Heading, Pitch, Bank) to quaternion
    Fvector hpb_vec;
    hpb_vec.set(h, p, b);

    Fquaternion xray_quat;
    xray_quat.rotationYawPitchRoll(hpb_vec);

    return TransformConverter::XRayQuatToOzz(xray_quat);
}

IFormatConverter::ConversionResult OGFConverter::Convert(const shared_str& input_path) {
    ConversionResult result;

    try {
        // Load OGF file
        if (!reader_.LoadFromFile(input_path)) {
            string256 error_buf;
            xr_sprintf(error_buf, "Failed to load OGF file: %s", input_path.c_str());
            result.error_message = error_buf;
            return result;
        }

        // Check if it's a skeleton type
        if (reader_.GetHeader().type != MT_SKELETON_ANIM && reader_.GetHeader().type != MT_SKELETON_RIGID) {
            result.error_message = "OGF file is not a skeleton type";
            return result;
        }

        // Parse OGF data
        auto parse_result = parser_.Parse(reader_);
        if (parse_result.bone_names.empty()) {
            result.error_message = "No bones found in OGF file";
            return result;
        }

        // Convert skeleton
        result.skeleton = skeleton_converter_.ConvertSkeleton(parse_result);
        if (!result.skeleton.Validate()) {
            result.error_message = "Failed to create valid skeleton";
            return result;
        }

        // Convert animations
        auto xr_animations = animation_converter_.ConvertAnimations(
            parse_result.motions,
            parse_result.bone_names,
            parse_result.motion_params
        );
        result.animations.assign(xr_animations.begin(), xr_animations.end());

        // Extract metadata
        result.metadata = ExtractMetadata(parse_result);

        result.success = true;

    } catch (...) {
        string512 error_buf;
        xr_sprintf(error_buf, "Exception during OGF conversion");
        result.error_message = error_buf;
    }

    return result;
}

XRayMetadata OGFConverter::ExtractMetadata(const OGFSkeletonParser::ParseResult& parse_result) {
    XRayMetadata metadata;

    // Extract motion parameters
    XRayMetadata::MotionParams motion_params;
    motion_params.speed = parse_result.motion_params.speed;
    motion_params.power = parse_result.motion_params.power;
    motion_params.accrue = parse_result.motion_params.accrue;
    motion_params.falloff = parse_result.motion_params.falloff;
    motion_params.bone_or_part = parse_result.motion_params.bone_or_part;
    motion_params.flags = parse_result.motion_params.flags;
    motion_params.event_markers = parse_result.motion_params.event_markers;

    metadata.motion_params["default"] = motion_params;

    // Extract IK constraints
    for (size_t i = 0; i < parse_result.ik_data.size() && i < parse_result.bone_names.size(); ++i) {
        metadata.ik_constraints[parse_result.bone_names[i].c_str()] = parse_result.ik_data[i];
    }

    return metadata;
}

} // namespace Animation
} // namespace XRay
