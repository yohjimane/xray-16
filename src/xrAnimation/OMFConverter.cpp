#include "stdafx.h"
#include "OMFConverter.h"
#include <cstdio>
#include "xrCore/file_stream_reader.h"
#include "xrCore/FS.h"

namespace XRay {
namespace Animation {

bool OMFReader::LoadFromFile(const shared_str& file_path) {
    // First try to open as regular file
    FILE* file = fopen(file_path.c_str(), "rb");
    if (file) {
        // Get file size
        fseek(file, 0, SEEK_END);
        long file_size = ftell(file);
        fseek(file, 0, SEEK_SET);
        
        // Read file data - need to store it persistently
        file_data_.resize(file_size);
        size_t read_size = fread(file_data_.data(), 1, file_size, file);
        fclose(file);
        
        if (read_size == static_cast<size_t>(file_size)) {
            reader_.reset(xr_new<IReader>(file_data_.data(), file_size));
            if (reader_) {
                ReadHeader();
                return true;
            }
        }
    }
    
    // Fall back to X-Ray FS if direct file access failed
    if (FS.exist(file_path.c_str())) {
        reader_.reset(FS.r_open(file_path.c_str()));
        if (!reader_) {
            Msg("! Failed to open OMF file: %s", file_path.c_str());
            return false;
        }
        ReadHeader();
        return true;
    }
    
    Msg("! OMF file not found: %s", file_path.c_str());
    return false;
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
    
    // Debug: List all chunks
    Msg("* OMF Chunks found:");
    reader_->seek(0);
    bool has_ogf_chunks = false;
    while (!reader_->eof()) {
        u32 chunk_id = reader_->r_u32();
        u32 chunk_size = reader_->r_u32();
        u32 pos = reader_->tell() - 8;
        
        if (reader_->eof() || chunk_size == 0 || chunk_size > reader_->elapsed()) {
            break;
        }
        
        Msg("  - Chunk 0x%04X at offset %d, size %d", chunk_id, pos, chunk_size);
        
        // Check if this is an OGF motion chunk
        if (chunk_id == OGF_S_MOTIONS || chunk_id == OGF_S_SMPARAMS) {
            has_ogf_chunks = true;
        }
        
        reader_->advance(chunk_size);
    }
    reader_->seek(0);
    
    // Handle OGF motion format (game OMF files)
    if (has_ogf_chunks) {
        Msg("* OMF: Detected OGF motion format");
        is_ogf_format_ = true;
        header_.version = OMF_VERSION; // Assume version 4
        
        // Count motions from OGF_S_MOTIONS chunk
        auto motions_chunk = reader_->open_chunk(OGF_S_MOTIONS);
        if (motions_chunk) {
            // First sub-chunk contains motion count
            auto count_chunk = motions_chunk->open_chunk(0);
            if (count_chunk) {
                header_.motion_count = count_chunk->r_u32();
                count_chunk->close();
            }
            motions_chunk->close();
        }
        
        Msg("* OMF: OGF format, %d motions", header_.motion_count);
        return;
    }
    
    // Standard OMF format
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
    if (is_ogf_format_) {
        return ReadOGFMotionDefs();
    }
    
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
    Msg("  - Motion name: %s, pos: %d", temp_name.c_str(), reader.tell());
    
    // Read motion flags first (before other parameters)
    motion_def.flags = reader.r_u32();
    Msg("  - Flags: 0x%X, pos: %d", motion_def.flags, reader.tell());
    
    // Read motion ID and bone/part reference
    motion_def.bone_or_part = reader.r_u16();
    motion_def.motion = reader.r_u16();
    Msg("  - Bone/part: %d, motion: %d, pos: %d", motion_def.bone_or_part, motion_def.motion, reader.tell());
    
    // Read motion parameters
    Msg("  - About to read speed at pos: %d, remaining: %d", reader.tell(), reader.elapsed());
    motion_def.speed = reader.r_float();
    Msg("  - Speed: %f, pos: %d", motion_def.speed, reader.tell());
    motion_def.power = reader.r_float();
    Msg("  - Power: %f, pos: %d", motion_def.power, reader.tell());
    motion_def.accrue = reader.r_float();
    Msg("  - Accrue: %f, pos: %d", motion_def.accrue, reader.tell());
    motion_def.falloff = reader.r_float();
    Msg("  - Falloff: %f, pos: %d", motion_def.falloff, reader.tell());
    
    // Read motion marks (event markers) for version 4+
    motion_def.marks_count = reader.r_u32();
    Msg("  - Marks count: %d, pos: %d", motion_def.marks_count, reader.tell());
    
    if (motion_def.marks_count > 0) {
        // Motion marks use special \r\n terminated strings in version 4
        for (u32 j = 0; j < motion_def.marks_count; ++j) {
            // Read string until \n, strip trailing \r
            xr_string mark_name_str;
            char ch;
            while (!reader.eof()) {
                ch = reader.r_u8();
                if (ch == '\n') break;
                mark_name_str += ch;
            }
            // Remove trailing \r if present
            if (!mark_name_str.empty() && mark_name_str.back() == '\r') {
                mark_name_str.pop_back();
            }
            shared_str mark_name(mark_name_str.c_str());
            
            u32 interval_count = reader.r_u32();
            Msg("    - Mark %d: %s, intervals: %d", j, mark_name.c_str(), interval_count);
            for (u32 k = 0; k < interval_count; ++k) {
                float first = reader.r_float();
                float second = reader.r_float();
                Msg("      - Interval %d: [%f, %f]", k, first, second);
            }
        }
    }
}

xr_vector<OMFBoneMotion> OMFReader::ReadBoneMotions() {
    if (is_ogf_format_) {
        // For OGF format, we need the bone count which should be provided externally
        // This function shouldn't be called for OGF format without bone count
        Msg("! Warning: ReadBoneMotions called for OGF format without bone count");
        return xr_vector<OMFBoneMotion>();
    }
    
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
        bone_motion.motion_length = motion_reader->r_u32();
        u8 motion_flags = motion_reader->r_u8();
        
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
    // Standard OMF format - reads bone count from data
    u8 bone_count = reader.r_u8();
    Msg("    - ReadCompressedMotion: motion '%s' has %d bones", bone_motion.name.c_str(), bone_count);
    bone_motion.bone_data.reserve(bone_count);
    
    // Read bone data
    for (u8 bone_idx = 0; bone_idx < bone_count; ++bone_idx) {
        OMFBoneData bone_data;
        
        // Read bone ID
        bone_data.bone_id = reader.r_u8();
        
        // Read flags for this bone's motion
        bone_data.flags = reader.r_u8();
        
        // Decompress motion keys for this specific bone
        // Standard OMF has variable number of keys per bone
        DecompressMotionKeys(reader, bone_data.flags, bone_data);
        
        bone_motion.bone_data.push_back(bone_data);
    }
}

void OMFReader::DecompressMotionKeys(IReader& reader, u32 flags, OMFBoneData& bone_data) {
    // Check if translation keys are present
    if (flags & flTKeyPresent) {
        // Read compressed translation data
        u16 t_key_count = reader.r_u16();
        
        bone_data.keys_translation_frames.reserve(t_key_count);
        bone_data.keys_translation.reserve(t_key_count);
        
        // Read initial translation
        reader.r_fvector3(bone_data.initial_translation);
        
        // Read translation size (for decompression)
        reader.r_fvector3(bone_data.translation_size);
        
        // Determine if using 16-bit or 8-bit compression
        bool use_16bit = (flags & flTKey16IsBit) != 0;
        
        for (u16 i = 0; i < t_key_count; ++i) {
            // Read frame index
            u16 frame = reader.r_u16();
            bone_data.keys_translation_frames.push_back(frame);
            
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
            
            Fvector translation = DecompressTranslation(packed_translation, bone_data.initial_translation, bone_data.translation_size);
            bone_data.keys_translation.push_back(translation);
        }
    } else {
        // No translation keys, just read initial position
        reader.r_fvector3(bone_data.initial_translation);
    }
    
    // Check if rotation keys are present
    if (!(flags & flRKeyAbsent)) {
        // Read compressed rotation data
        u16 r_key_count = reader.r_u16();
        
        bone_data.keys_rotation_frames.reserve(r_key_count);
        bone_data.keys_rotation.reserve(r_key_count);
        
        for (u16 i = 0; i < r_key_count; ++i) {
            // Read frame index
            u16 frame = reader.r_u16();
            bone_data.keys_rotation_frames.push_back(frame);
            
            // Read compressed quaternion (64-bit packed)
            u64 packed_quat = reader.r_u64();
            Fquaternion rotation = DecompressQuaternion(packed_quat);
            bone_data.keys_rotation.push_back(rotation);
        }
    }
}

void OMFReader::ReadCompressedMotion(IReader& reader, OMFBoneMotion& bone_motion, u32 num_bones, u32 motion_length) {
    // OGF format - bone count is provided externally (all skeleton bones)
    bone_motion.bone_data.reserve(num_bones);
    
    // Read data for ALL bones in skeleton
    for (u32 bone_idx = 0; bone_idx < num_bones; ++bone_idx) {
        OMFBoneData bone_data;
        bone_data.bone_id = bone_idx;  // Sequential bone IDs
        
        // Read flags for this bone
        bone_data.flags = reader.r_u8();
        
        // Decompress motion keys based on flags
        DecompressMotionKeys(reader, bone_data.flags, bone_data, motion_length);
        
        bone_motion.bone_data.push_back(bone_data);
    }
}

void OMFReader::DecompressMotionKeys(IReader& reader, u32 flags, OMFBoneData& bone_data, u32 motion_length) {
    // OGF format version - motion length is provided
    // Order is important: rotation first, then translation
    
    // Rotation
    if (flags & flRKeyAbsent) {
        // Single rotation key
        u64 packed_quat = reader.r_u64();
        Fquaternion rotation = DecompressQuaternion(packed_quat);
        bone_data.keys_rotation_frames.push_back(0);
        bone_data.keys_rotation.push_back(rotation);
    } else {
        // Multiple rotation keys
        reader.r_u32(); // Skip CRC
        for (u32 i = 0; i < motion_length; ++i) {
            u64 packed_quat = reader.r_u64();
            Fquaternion rotation = DecompressQuaternion(packed_quat);
            bone_data.keys_rotation_frames.push_back(i);
            bone_data.keys_rotation.push_back(rotation);
        }
    }
    
    // Translation
    if (flags & flTKeyPresent) {
        reader.r_u32(); // Skip CRC
        bool use_16bit = (flags & flTKey16IsBit) != 0;
        
        // First read all compressed values
        xr_vector<Fvector> compressed_translations;
        compressed_translations.reserve(motion_length);
        
        for (u32 i = 0; i < motion_length; ++i) {
            Fvector compressed;
            if (use_16bit) {
                // 16-bit precision (3 x 16-bit values)
                compressed.x = float(s16(reader.r_u16()));
                compressed.y = float(s16(reader.r_u16()));
                compressed.z = float(s16(reader.r_u16()));
            } else {
                // 8-bit precision (3 x 8-bit values)
                compressed.x = float(s8(reader.r_u8()));
                compressed.y = float(s8(reader.r_u8()));
                compressed.z = float(s8(reader.r_u8()));
            }
            compressed_translations.push_back(compressed);
        }
        
        // Then read translation size and init
        Fvector t_size, t_init;
        reader.r_fvector3(t_size);
        reader.r_fvector3(t_init);
        
        // Store initial values for decompression
        bone_data.translation_size = t_size;
        bone_data.initial_translation = t_init;
        
        // Now decompress all values
        for (u32 i = 0; i < motion_length; ++i) {
            const Fvector& compressed = compressed_translations[i];
            Fvector translation;
            
            translation.x = t_init.x + compressed.x * t_size.x;
            translation.y = t_init.y + compressed.y * t_size.y;
            translation.z = t_init.z + compressed.z * t_size.z;
            
            bone_data.keys_translation_frames.push_back(i);
            bone_data.keys_translation.push_back(translation);
        }
    } else {
        // Single translation key
        Fvector translation;
        reader.r_fvector3(translation);
        bone_data.keys_translation_frames.push_back(0);
        bone_data.keys_translation.push_back(translation);
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
    
    // Create tracks for each bone in the skeleton
    animation.tracks.resize(bone_names.size());
    
    // Calculate duration based on all bone motion data
    float max_duration = 0.0f;
    for (const auto& bone_data : omf_motion.bone_data) {
        // Find maximum frame from translation keys
        for (u16 frame : bone_data.keys_translation_frames) {
            float time = frame / params.fps;
            max_duration = _max(max_duration, time);
        }
        // Find maximum frame from rotation keys  
        for (u16 frame : bone_data.keys_rotation_frames) {
            float time = frame / params.fps;
            max_duration = _max(max_duration, time);
        }
    }
    animation.duration = max_duration > 0.0f ? max_duration : 1.0f;
    
    // Map bone motion data to skeleton tracks
    // We need to match bone IDs from OMF to bone indices in the skeleton
    Msg("* Motion '%s' has %d bone data entries", omf_motion.name.c_str(), omf_motion.bone_data.size());
    for (const auto& bone_data : omf_motion.bone_data) {
        // For now, use bone_id as the track index (this assumes they match)
        // TODO: Implement proper bone name/ID mapping
        size_t track_idx = bone_data.bone_id;
        
        Msg("  - Bone ID %d: %d translation keys, %d rotation keys", 
            bone_data.bone_id, 
            bone_data.keys_translation_frames.size(),
            bone_data.keys_rotation_frames.size());
        
        if (track_idx < animation.tracks.size()) {
            auto& track = animation.tracks[track_idx];
            
            // Convert translation keys
            for (size_t i = 0; i < bone_data.keys_translation_frames.size(); ++i) {
                ozz::animation::offline::RawAnimation::TranslationKey key;
                key.time = bone_data.keys_translation_frames[i] / params.fps;
                const Fvector& t = bone_data.keys_translation[i];
                
                // Apply coordinate system conversion (X-Ray Y-up to ozz Z-up)
                key.value = ozz::math::Float3(t.x, t.z, -t.y);
                
                // Debug first few keys
                if (i < 3 && track_idx < 5) {
                    Msg("    Track %d, key %d: time=%.3f, pos=(%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)",
                        track_idx, i, key.time, t.x, t.y, t.z, key.value.x, key.value.y, key.value.z);
                }
                
                track.translations.push_back(key);
            }
            
            // Convert rotation keys
            for (size_t i = 0; i < bone_data.keys_rotation_frames.size(); ++i) {
                ozz::animation::offline::RawAnimation::RotationKey key;
                key.time = bone_data.keys_rotation_frames[i] / params.fps;
                const Fquaternion& q = bone_data.keys_rotation[i];
                
                // Apply coordinate system conversion for quaternions
                // X-Ray Y-up to ozz Z-up: (x,y,z,w) -> (x,z,-y,w)
                key.value = ozz::math::Quaternion(q.x, q.z, -q.y, q.w);
                track.rotations.push_back(key);
            }
            
            // Add default scale key
            ozz::animation::offline::RawAnimation::ScaleKey scale_key;
            scale_key.time = 0.0f;
            scale_key.value = ozz::math::Float3::one();
            track.scales.push_back(scale_key);
        }
    }
    
    // Ensure all tracks have at least one key (fallback for bones without animation data)
    int tracks_without_data = 0;
    for (size_t i = 0; i < animation.tracks.size(); ++i) {
        auto& track = animation.tracks[i];
        if (track.translations.empty()) {
            tracks_without_data++;
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
    
    Msg("* %d tracks out of %d had no animation data and used fallback values", 
        tracks_without_data, animation.tracks.size());
    
    return animation;
}

OMFToOzzAnimationConverter::KeyframeData OMFToOzzAnimationConverter::ExtractKeyframes(
    const OMFBoneMotion& omf_motion,
    float fps
) {
    // Legacy function - no longer used with new per-bone structure
    KeyframeData keyframes;
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
    // Legacy function - no longer used with new per-bone structure
    return 1.0f;
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
        
        // Read bone motions - use bone count for OGF format
        xr_vector<OMFBoneMotion> bone_motions;
        if (reader_.IsOGFFormat()) {
            bone_motions = reader_.ReadOGFBoneMotions(static_cast<u32>(bone_names.size()));
        } else {
            bone_motions = reader_.ReadBoneMotions();
        }
        
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

xr_vector<OMFMotionDef> OMFReader::ReadOGFMotionDefs() {
    xr_vector<OMFMotionDef> motion_defs;
    
    if (!reader_) {
        return motion_defs;
    }
    
    // Read OGF_S_SMPARAMS chunk
    auto params_chunk = reader_->open_chunk(OGF_S_SMPARAMS);
    if (!params_chunk) {
        Msg("! OMF: OGF_S_SMPARAMS chunk not found");
        return motion_defs;
    }
    
    Msg("* OGF_S_SMPARAMS chunk size: %d, position: %d", params_chunk->length(), params_chunk->tell());
    
    // Read version
    u16 version = params_chunk->r_u16();
    Msg("* OGF_S_SMPARAMS version: %d", version);
    
    // Skip bone parts for now
    u16 part_count = params_chunk->r_u16();
    Msg("* Part count: %d", part_count);
    
    for (u16 i = 0; i < part_count; ++i) {
        shared_str part_name;
        params_chunk->r_stringZ(part_name);
        u16 bone_count = params_chunk->r_u16();
        Msg("  - Part %d: %s, bones: %d", i, part_name.c_str(), bone_count);
        for (u16 j = 0; j < bone_count; ++j) {
            shared_str bone_name;
            params_chunk->r_stringZ(bone_name);
            u32 bone_id = params_chunk->r_u32();
            Msg("    - Bone: %s (id: %d)", bone_name.c_str(), bone_id);
        }
    }
    
    Msg("* Position after parts: %d of %d", params_chunk->tell(), params_chunk->length());
    
    // Read motion definitions
    u16 motion_count = params_chunk->r_u16();
    motion_defs.reserve(motion_count);
    Msg("* Motion count: %d", motion_count);
    
    for (u16 i = 0; i < motion_count; ++i) {
        Msg("* Reading motion %d at position %d", i, params_chunk->tell());
        OMFMotionDef def;
        ReadMotionParams(*params_chunk, def);
        motion_defs.push_back(def);
    }
    
    params_chunk->close();
    return motion_defs;
}

xr_vector<OMFBoneMotion> OMFReader::ReadOGFBoneMotions(u32 bone_count) {
    xr_vector<OMFBoneMotion> bone_motions;
    
    if (!reader_) {
        return bone_motions;
    }
    
    auto motions_chunk = reader_->open_chunk(OGF_S_MOTIONS);
    if (!motions_chunk) {
        Msg("! OMF: OGF_S_MOTIONS chunk not found");
        return bone_motions;
    }
    
    // Get motion count from chunk 0
    u32 expected_motion_count = 0;
    auto count_chunk = motions_chunk->open_chunk(0);
    if (count_chunk) {
        expected_motion_count = count_chunk->r_u32();
        count_chunk->close();
    }
    
    // Use provided bone count
    u32 num_bones = bone_count;
    
    // Read motion data starting from chunk 1
    u32 motion_idx = 1;
    IReader* motion_reader = nullptr;
    
    while ((motion_reader = motions_chunk->open_chunk(motion_idx)) != nullptr) {
        OMFBoneMotion bone_motion;
        
        // Read motion name
        shared_str temp_name;
        motion_reader->r_stringZ(temp_name);
        bone_motion.name = temp_name;
        
        // Read motion length (number of frames)
        bone_motion.motion_length = motion_reader->r_u32();
        
        Msg("    - Motion '%s': length=%d frames, reading data for %d bones", 
            bone_motion.name.c_str(), bone_motion.motion_length, num_bones);
        
        // Read compressed motion data for ALL bones
        ReadCompressedMotion(*motion_reader, bone_motion, num_bones, bone_motion.motion_length);
        
        bone_motions.push_back(bone_motion);
        
        motion_reader->close();
        motion_idx++;
    }
    
    motions_chunk->close();
    return bone_motions;
}

} // namespace Animation
} // namespace XRay