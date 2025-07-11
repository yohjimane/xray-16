#include "stdafx.h"
#include "OMFConverter.h"
#include "xrCore/file_stream_reader.h"
#include "xrCore/FS.h"

namespace XRay {
namespace Animation {

bool OMFReader::LoadFromFile(const shared_str& file_path) {
    if (!FS.exist(file_path.c_str())) {
        Msg("! OMF file not found: %s", file_path.c_str());
        return false;
    }
    
    reader_.reset(FS.r_open(file_path.c_str()));
    if (!reader_) {
        Msg("! Failed to open OMF file: %s", file_path.c_str());
        return false;
    }
    
    ReadHeader();
    return true;
}

bool OMFReader::LoadFromMemory(const void* data, size_t size) {
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

void OMFReader::ReadHeader() {
    if (!reader_) {
        return;
    }
    
    // Find version chunk
    auto version_chunk = reader_->open_chunk(OMF_CHUNK_VERSION);
    if (!version_chunk) {
        Msg("! OMF: Version chunk not found");
        return;
    }
    
    header_.version = version_chunk->r_u32();
    if (header_.version != OMF_VERSION) {
        Msg("! OMF: Unsupported version %d (expected %d)", header_.version, OMF_VERSION);
    }
    
    version_chunk->close();
    
    // Count motions
    header_.motion_count = 0;
    auto motions_chunk = reader_->open_chunk(OMF_CHUNK_MOTIONS);
    if (motions_chunk) {
        while (motions_chunk->find_chunk(header_.motion_count)) {
            motions_chunk->close();
            header_.motion_count++;
        }
    }
    motions_chunk->close();
    
    Msg("* OMF: Version %d, %d motions", header_.version, header_.motion_count);
}

xr_vector<OMFMotionDef> OMFReader::ReadMotionDefs() {
    xr_vector<OMFMotionDef> motion_defs;
    
    if (!reader_) {
        return motion_defs;
    }
    
    auto params_chunk = reader_->open_chunk(OMF_CHUNK_PARAMS);
    if (!params_chunk) {
        Msg("! OMF: Params chunk not found");
        return motion_defs;
    }
    
    u16 motion_count = params_chunk->r_u16();
    motion_defs.reserve(motion_count);
    
    for (u16 i = 0; i < motion_count; ++i) {
        OMFMotionDef def;
        ReadMotionParams(*params_chunk, def);
        motion_defs.push_back(def);
    }
    
    params_chunk->close();
    return motion_defs;
}

void OMFReader::ReadMotionParams(IReader& reader, OMFMotionDef& motion_def) {
    // Read motion name
    shared_str temp_name;
    reader.r_stringZ(temp_name);
    motion_def.name = temp_name;
    
    // Read motion parameters
    motion_def.flags = reader.r_u32();
    motion_def.bone_or_part = reader.r_u16();
    motion_def.motion = reader.r_u16();
    motion_def.speed = reader.r_float();
    motion_def.power = reader.r_float();
    motion_def.accrue = reader.r_float();
    motion_def.falloff = reader.r_float();
    
    // Read motion marks (event markers)
    motion_def.marks_count = reader.r_u32();
    motion_def.marks.resize(motion_def.marks_count);
    
    for (u32 j = 0; j < motion_def.marks_count; ++j) {
        shared_str mark_name;
        reader.r_stringZ(mark_name);
        motion_def.marks[j] = reader.r_float();
    }
}

xr_vector<OMFBoneMotion> OMFReader::ReadBoneMotions() {
    xr_vector<OMFBoneMotion> bone_motions;
    
    if (!reader_) {
        return bone_motions;
    }
    
    auto motions_chunk = reader_->open_chunk(OMF_CHUNK_MOTIONS);
    if (!motions_chunk) {
        Msg("! OMF: Motions chunk not found");
        return bone_motions;
    }
    
    u32 motion_idx = 0;
    IReader* motion_reader = nullptr;
    
    while ((motion_reader = motions_chunk->open_chunk(motion_idx)) != nullptr) {
        OMFBoneMotion bone_motion;
        
        // Read motion name
        shared_str temp_name;
        motion_reader->r_stringZ(temp_name);
        bone_motion.name = temp_name;
        
        // Read motion data length and flags
        u32 length = motion_reader->r_u32();
        bone_motion.flags = motion_reader->r_u8();
        
        // Read compressed motion data
        ReadCompressedMotion(*motion_reader, bone_motion);
        
        bone_motions.push_back(bone_motion);
        
        motion_reader->close();
        motion_idx++;
    }
    
    motions_chunk->close();
    return bone_motions;
}

void OMFReader::ReadCompressedMotion(IReader& reader, OMFBoneMotion& bone_motion) {
    // Read bone count for this motion
    u8 bone_count = reader.r_u8();
    
    // Read bone data
    for (u8 bone_idx = 0; bone_idx < bone_count; ++bone_idx) {
        // Read bone ID
        u8 bone_id = reader.r_u8();
        
        // Read flags for this bone's motion
        u8 flags = reader.r_u8();
        
        // Decompress motion keys based on flags
        DecompressMotionKeys(reader, flags, bone_motion);
    }
}

void OMFReader::DecompressMotionKeys(IReader& reader, u32 flags, OMFBoneMotion& motion) {
    // Check if translation keys are present
    if (flags & flTKeyPresent) {
        // Read compressed translation data
        u16 t_key_count = reader.r_u16();
        
        motion.keys_translation_frames.reserve(t_key_count);
        motion.keys_translation.reserve(t_key_count);
        
        // Read initial translation
        Fvector t_init;
        reader.r_fvector3(t_init);
        
        // Read translation size (for decompression)
        Fvector t_size;
        reader.r_fvector3(t_size);
        
        // Determine if using 16-bit or 8-bit compression
        bool use_16bit = (flags & flTKey16IsBit) != 0;
        
        for (u16 i = 0; i < t_key_count; ++i) {
            // Read frame index
            u16 frame = reader.r_u16();
            motion.keys_translation_frames.push_back(frame);
            
            // Read and decompress translation
            u32 packed_translation;
            if (use_16bit) {
                // 16-bit precision (3 x 16-bit values)
                packed_translation = reader.r_u32();
                u16 extra = reader.r_u16();
                packed_translation |= (u32(extra) << 16);
            } else {
                // 8-bit precision (3 x 8-bit values)
                packed_translation = reader.r_u32() & 0xFFFFFF;
            }
            
            Fvector translation = DecompressTranslation(packed_translation, t_init, t_size);
            motion.keys_translation.push_back(translation);
        }
    }
    
    // Check if rotation keys are present
    if (!(flags & flRKeyAbsent)) {
        // Read compressed rotation data
        u16 r_key_count = reader.r_u16();
        
        motion.keys_rotation_frames.reserve(r_key_count);
        motion.keys_rotation.reserve(r_key_count);
        
        for (u16 i = 0; i < r_key_count; ++i) {
            // Read frame index
            u16 frame = reader.r_u16();
            motion.keys_rotation_frames.push_back(frame);
            
            // Read compressed quaternion (64-bit packed)
            u64 packed_quat = reader.r_u64();
            Fquaternion rotation = DecompressQuaternion(packed_quat);
            motion.keys_rotation.push_back(rotation);
        }
    }
}

Fquaternion OMFReader::DecompressQuaternion(u64 packed) const {
    // X-Ray uses 16-bit per component quaternion compression
    const float scale = 1.0f / 32767.0f;
    
    s16 x = s16(packed & 0xFFFF);
    s16 y = s16((packed >> 16) & 0xFFFF);
    s16 z = s16((packed >> 32) & 0xFFFF);
    s16 w = s16((packed >> 48) & 0xFFFF);
    
    Fquaternion result;
    result.set(x * scale, y * scale, z * scale, w * scale);
    result.normalize();
    
    return result;
}

Fvector OMFReader::DecompressTranslation(u32 packed, const Fvector& init, const Fvector& size) const {
    // Extract components based on compression type
    float x, y, z;
    
    if (packed & 0xFF000000) {
        // 16-bit compression
        const float scale = 1.0f / 32767.0f;
        s16 cx = s16(packed & 0xFFFF);
        s16 cy = s16((packed >> 16) & 0xFFFF);
        s16 cz = s16((packed >> 32) & 0xFFFF);
        
        x = init.x + cx * scale * size.x;
        y = init.y + cy * scale * size.y;
        z = init.z + cz * scale * size.z;
    } else {
        // 8-bit compression
        const float scale = 1.0f / 127.0f;
        s8 cx = s8(packed & 0xFF);
        s8 cy = s8((packed >> 8) & 0xFF);
        s8 cz = s8((packed >> 16) & 0xFF);
        
        x = init.x + cx * scale * size.x;
        y = init.y + cy * scale * size.y;
        z = init.z + cz * scale * size.z;
    }
    
    Fvector result;
    result.set(x, y, z);
    return result;
}

xr_vector<ozz::animation::offline::RawAnimation> OMFToOzzAnimationConverter::ConvertAnimations(
    const xr_vector<OMFBoneMotion>& omf_motions,
    const xr_vector<shared_str>& bone_names,
    const ConversionParams& params
) {
    xr_vector<ozz::animation::offline::RawAnimation> animations;
    animations.reserve(omf_motions.size());
    
    for (const auto& omf_motion : omf_motions) {
        auto animation = ConvertSingleMotion(omf_motion, bone_names, params);
        animations.push_back(animation);
    }
    
    return animations;
}

ozz::animation::offline::RawAnimation OMFToOzzAnimationConverter::ConvertSingleMotion(
    const OMFBoneMotion& omf_motion,
    const xr_vector<shared_str>& bone_names,
    const ConversionParams& params
) {
    ozz::animation::offline::RawAnimation animation;
    animation.name = omf_motion.name.c_str();
    animation.duration = CalculateAnimationDuration(omf_motion, params.fps);
    
    // Create tracks for each bone
    animation.tracks.resize(bone_names.size());
    
    // Extract keyframes from compressed data
    KeyframeData keyframes = ExtractKeyframes(omf_motion, params.fps);
    
    // For now, apply the same animation to the first bone (simplified)
    // In a real implementation, we'd match bone names and apply per-bone data
    if (!animation.tracks.empty() && !keyframes.times.empty()) {
        BuildAnimationTrack(keyframes, animation.tracks[0]);
    }
    
    // Ensure all tracks have at least one key
    for (auto& track : animation.tracks) {
        if (track.translations.empty()) {
            ozz::animation::offline::RawAnimation::TranslationKey key;
            key.time = 0.0f;
            key.value = ozz::math::Float3::zero();
            track.translations.push_back(key);
        }
        
        if (track.rotations.empty()) {
            ozz::animation::offline::RawAnimation::RotationKey key;
            key.time = 0.0f;
            key.value = ozz::math::Quaternion::identity();
            track.rotations.push_back(key);
        }
        
        if (track.scales.empty()) {
            ozz::animation::offline::RawAnimation::ScaleKey key;
            key.time = 0.0f;
            key.value = ozz::math::Float3::one();
            track.scales.push_back(key);
        }
    }
    
    return animation;
}

OMFToOzzAnimationConverter::KeyframeData OMFToOzzAnimationConverter::ExtractKeyframes(
    const OMFBoneMotion& omf_motion,
    float fps
) {
    KeyframeData keyframes;
    
    // Convert translation keyframes
    for (size_t i = 0; i < omf_motion.keys_translation_frames.size(); ++i) {
        float time = omf_motion.keys_translation_frames[i] / fps;
        keyframes.times.push_back(time);
        
        const Fvector& t = omf_motion.keys_translation[i];
        keyframes.translations.push_back(ozz::math::Float3(t.x, t.y, t.z));
    }
    
    // Convert rotation keyframes
    for (size_t i = 0; i < omf_motion.keys_rotation_frames.size(); ++i) {
        float time = omf_motion.keys_rotation_frames[i] / fps;
        
        // Only add time if not already present
        auto it = std::find(keyframes.times.begin(), keyframes.times.end(), time);
        if (it == keyframes.times.end()) {
            keyframes.times.push_back(time);
        }
        
        const Fquaternion& q = omf_motion.keys_rotation[i];
        keyframes.rotations.push_back(ozz::math::Quaternion(q.x, q.y, q.z, q.w));
    }
    
    // Add default scale keyframes
    keyframes.scales.push_back(ozz::math::Float3::one());
    
    return keyframes;
}

void OMFToOzzAnimationConverter::BuildAnimationTrack(
    const KeyframeData& keyframes,
    ozz::animation::offline::RawAnimation::JointTrack& track
) {
    // Build translation keys
    for (size_t i = 0; i < keyframes.times.size() && i < keyframes.translations.size(); ++i) {
        ozz::animation::offline::RawAnimation::TranslationKey key;
        key.time = keyframes.times[i];
        key.value = keyframes.translations[i];
        track.translations.push_back(key);
    }
    
    // Build rotation keys
    for (size_t i = 0; i < keyframes.times.size() && i < keyframes.rotations.size(); ++i) {
        ozz::animation::offline::RawAnimation::RotationKey key;
        key.time = keyframes.times[i];
        key.value = keyframes.rotations[i];
        track.rotations.push_back(key);
    }
    
    // Build scale keys (usually just one default key)
    ozz::animation::offline::RawAnimation::ScaleKey scale_key;
    scale_key.time = 0.0f;
    scale_key.value = keyframes.scales.empty() ? ozz::math::Float3::one() : keyframes.scales[0];
    track.scales.push_back(scale_key);
}

float OMFToOzzAnimationConverter::CalculateAnimationDuration(
    const OMFBoneMotion& omf_motion,
    float fps
) {
    float max_time = 0.0f;
    
    // Find maximum frame index from translation keys
    for (u16 frame : omf_motion.keys_translation_frames) {
        float time = frame / fps;
        max_time = _max(max_time, time);
    }
    
    // Find maximum frame index from rotation keys
    for (u16 frame : omf_motion.keys_rotation_frames) {
        float time = frame / fps;
        max_time = _max(max_time, time);
    }
    
    // Default to 1 second if no keys found
    return max_time > 0.0f ? max_time : 1.0f;
}

IFormatConverter::ConversionResult OMFConverter::Convert(const shared_str& input_path) {
    ConversionResult result;
    
    try {
        // Load OMF file
        if (!reader_.LoadFromFile(input_path)) {
            string256 error_buf;
            xr_sprintf(error_buf, "Failed to load OMF file: %s", input_path.c_str());
            result.error_message = error_buf;
            return result;
        }
        
        // Read motion definitions
        auto motion_defs = reader_.ReadMotionDefs();
        
        // Read bone motions
        auto bone_motions = reader_.ReadBoneMotions();
        
        if (bone_motions.empty()) {
            result.error_message = "No motions found in OMF file";
            return result;
        }
        
        // We need bone names from a skeleton - for now use placeholder
        xr_vector<shared_str> bone_names;
        bone_names.push_back(shared_str("root"));
        
        // Convert animations
        auto xr_animations = converter_.ConvertAnimations(bone_motions, bone_names);
        result.animations.assign(xr_animations.begin(), xr_animations.end());
        
        // Extract metadata
        result.metadata = ExtractMetadata(motion_defs);
        
        result.success = true;
        
    } catch (...) {
        string512 error_buf;
        xr_sprintf(error_buf, "Exception during OMF conversion");
        result.error_message = error_buf;
    }
    
    return result;
}

IFormatConverter::ConversionResult OMFConverter::ConvertWithSkeleton(
    const shared_str& input_path,
    const ozz::animation::offline::RawSkeleton& skeleton
) {
    ConversionResult result;
    
    try {
        // Load OMF file
        if (!reader_.LoadFromFile(input_path)) {
            string256 error_buf;
            xr_sprintf(error_buf, "Failed to load OMF file: %s", input_path.c_str());
            result.error_message = error_buf;
            return result;
        }
        
        // Extract bone names from skeleton
        auto bone_names = ExtractBoneNamesFromSkeleton(skeleton);
        
        // Read motion definitions
        auto motion_defs = reader_.ReadMotionDefs();
        
        // Read bone motions
        auto bone_motions = reader_.ReadBoneMotions();
        
        if (bone_motions.empty()) {
            result.error_message = "No motions found in OMF file";
            return result;
        }
        
        // Convert animations with proper bone names
        auto xr_animations = converter_.ConvertAnimations(bone_motions, bone_names);
        result.animations.assign(xr_animations.begin(), xr_animations.end());
        
        // Copy skeleton
        result.skeleton = skeleton;
        
        // Extract metadata
        result.metadata = ExtractMetadata(motion_defs);
        
        result.success = true;
        
    } catch (...) {
        string512 error_buf;
        xr_sprintf(error_buf, "Exception during OMF conversion");
        result.error_message = error_buf;
    }
    
    return result;
}

XRayMetadata OMFConverter::ExtractMetadata(const xr_vector<OMFMotionDef>& motion_defs) {
    XRayMetadata metadata;
    
    // Convert motion definitions to metadata
    for (const auto& def : motion_defs) {
        XRayMetadata::MotionParams params;
        params.speed = def.speed;
        params.power = def.power;
        params.accrue = def.accrue;
        params.falloff = def.falloff;
        params.bone_or_part = def.bone_or_part;
        params.flags = def.flags;
        params.event_markers = def.marks;
        
        metadata.motion_params[def.name.c_str()] = params;
    }
    
    return metadata;
}

xr_vector<shared_str> OMFConverter::ExtractBoneNamesFromSkeleton(
    const ozz::animation::offline::RawSkeleton& skeleton
) {
    xr_vector<shared_str> bone_names;
    
    // Use iterative approach instead of recursive lambda
    struct JointToProcess {
        const ozz::animation::offline::RawSkeleton::Joint* joint;
    };
    
    xr_vector<JointToProcess> joint_stack;
    
    // Add all roots to stack
    for (const auto& root : skeleton.roots) {
        joint_stack.push_back({&root});
    }
    
    // Process joints iteratively
    while (!joint_stack.empty()) {
        JointToProcess current = joint_stack.back();
        joint_stack.pop_back();
        
        bone_names.push_back(shared_str(current.joint->name.c_str()));
        
        // Add children in reverse order to maintain depth-first order
        for (auto it = current.joint->children.rbegin(); it != current.joint->children.rend(); ++it) {
            joint_stack.push_back({&(*it)});
        }
    }
    
    return bone_names;
}

} // namespace Animation
} // namespace XRay