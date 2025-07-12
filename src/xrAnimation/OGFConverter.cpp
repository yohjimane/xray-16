#include "stdafx.h"
#include "OGFConverter.h"
#include <cstdio>
#include "xrCore/file_stream_reader.h"
#include "xrCore/FS.h"

namespace XRay {
namespace Animation {

// X-Ray OGF constants
const u16 xrOGF_SMParamsVersion = 4;
const u8 MT_NORMAL = 0;
const u8 MT_HIERRARHY = 1;
const u8 MT_PROGRESSIVE = 2;
const u8 MT_SKELETON_ANIM = 3;
const u8 MT_SKELETON_GEOMDEF_PM = 4;
const u8 MT_SKELETON_GEOMDEF_ST = 5;
const u8 MT_LOD = 6;
const u8 MT_TREE_ST = 7;
const u8 MT_PARTICLE_EFFECT = 8;
const u8 MT_PARTICLE_GROUP = 9;
const u8 MT_SKELETON_RIGID = 10;
const u8 MT_TREE_PM = 11;

bool OGFReader::LoadFromFile(const shared_str& file_path) {
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
        
        if (read_size == file_size) {
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
        if (reader_) {
            ReadHeader();
            return true;
        }
    }
    
    return false;
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

    // OGF files start with the OGF_HEADER chunk
    u32 chunk_type = reader_->r_u32();
    u32 chunk_size = reader_->r_u32();
    
    if (chunk_type != OGF_HEADER) {
        Msg("! Warning: First chunk is not OGF_HEADER (found 0x%08X)", chunk_type);
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

    // Skip the header chunk
    reader_->seek(0);
    u32 first_chunk_type = reader_->r_u32();
    u32 first_chunk_size = reader_->r_u32();
    if (first_chunk_type == OGF_HEADER) {
        reader_->advance(first_chunk_size);
    } else {
        // No header chunk, go back
        reader_->seek(0);
    }

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

void OGFReader::DebugListChunks() {
    if (!reader_) {
        return;
    }

    Msg("* OGF Chunks found:");
    reader_->seek(0);  // Start from beginning

    while (!reader_->eof()) {
        ChunkHeader chunk_header;
        u32 pos = reader_->tell();
        
        if (reader_->elapsed() < sizeof(ChunkHeader)) {
            break;
        }
        
        chunk_header.type = reader_->r_u32();
        chunk_header.size = reader_->r_u32();

        const char* chunk_name = "UNKNOWN";
        switch (chunk_header.type) {
            case OGF_HEADER: chunk_name = "OGF_HEADER"; break;
            case OGF_TEXTURE: chunk_name = "OGF_TEXTURE"; break;
            case OGF_VERTICES: chunk_name = "OGF_VERTICES"; break;
            case OGF_INDICES: chunk_name = "OGF_INDICES"; break;
            case OGF_SWIDATA: chunk_name = "OGF_SWIDATA"; break;
            case OGF_CHILDREN: chunk_name = "OGF_CHILDREN"; break;
            case OGF_CHILDREN_L: chunk_name = "OGF_CHILDREN_L"; break;
            case OGF_S_BONE_NAMES: chunk_name = "OGF_S_BONE_NAMES"; break;
            case OGF_S_MOTIONS: chunk_name = "OGF_S_MOTIONS"; break;
            case OGF_S_SMPARAMS: chunk_name = "OGF_S_SMPARAMS"; break;
            case OGF_S_IKDATA: chunk_name = "OGF_S_IKDATA"; break;
            case OGF_S_USERDATA: chunk_name = "OGF_S_USERDATA"; break;
            case OGF_S_DESC: chunk_name = "OGF_S_DESC"; break;
            case OGF_S_MOTION_REFS: chunk_name = "OGF_S_MOTION_REFS"; break;
        }
        
        Msg("  - Chunk 0x%02X (%s) at offset %d, size %d", chunk_header.type, chunk_name, pos, chunk_header.size);

        // Skip this chunk
        reader_->advance(chunk_header.size);
        if (reader_->eof()) {
            break;
        }
    }
    
    // Reset to after header
    reader_->seek(0);
    u32 first_chunk_type = reader_->r_u32();
    u32 first_chunk_size = reader_->r_u32();
    if (first_chunk_type == OGF_HEADER) {
        reader_->advance(first_chunk_size);
    } else {
        reader_->seek(0);
    }
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

xr_vector<OGFReader::BoneData> OGFReader::ReadBoneData() {
    xr_vector<BoneData> bones;

    auto chunk_reader = ReadChunk(OGF_S_BONE_NAMES);
    if (!chunk_reader) {
        return bones;
    }

    u32 bone_count = chunk_reader->r_u32();
    bones.reserve(bone_count);
    
    Msg("* Reading %d bones from OGF_S_BONE_NAMES chunk", bone_count);

    // Read bone names and parent names from OGF_S_BONE_NAMES chunk
    for (u32 i = 0; i < bone_count; ++i) {
        BoneData bone;
        
        // Read bone name
        string256 buf;
        chunk_reader->r_stringZ(buf, sizeof(buf));
        xr_strlwr(buf);
        bone.name = shared_str(buf);
        
        // Read parent name
        chunk_reader->r_stringZ(buf, sizeof(buf));
        xr_strlwr(buf);
        bone.parent_name = shared_str(buf);
        
        // Read OBB (oriented bounding box)
        chunk_reader->r(&bone.obb, sizeof(Fobb));
        
        // Use OBB transform as the bind pose transform
        bone.obb.xform_get(bone.bind_transform);
        bone.mass = 1.0f;
        bone.center_of_mass.set(0, 0, 0);
        
        // DEBUG: Print OBB data
        Msg("    OBB translate: (%.3f, %.3f, %.3f)", 
            bone.obb.m_translate.x, bone.obb.m_translate.y, bone.obb.m_translate.z);
        Msg("    OBB halfsize: (%.3f, %.3f, %.3f)", 
            bone.obb.m_halfsize.x, bone.obb.m_halfsize.y, bone.obb.m_halfsize.z);
        
        Msg("  - Bone %d: '%s' (parent: '%s')", i, bone.name.c_str(), 
            bone.parent_name.size() ? bone.parent_name.c_str() : "<root>");
        
        bones.push_back(bone);
    }

    // Read additional bone data from OGF_S_IKDATA chunk if available
    auto ik_reader = ReadChunk(OGF_S_IKDATA);
    if (ik_reader) {
        Msg("* Reading IK data for %d bones using xrSDK format with shape validity check", bones.size());
        
        for (u32 i = 0; i < bones.size() && !ik_reader->eof(); ++i) {
            BoneData& bone = bones[i];
            
            u32 start_pos = ik_reader->tell();
            
            // Read version (uint32) - OGF_IKDATA_VERSION = 0x0001
            u32 version = ik_reader->r_u32();
            
            // Read game material string
            shared_str game_mtl;
            ik_reader->r_stringZ(game_mtl);
            
            // Read SBoneShape structure to check validity
            struct {
                u16 type;     // EShapeType: stNone=0, stBox=1, stSphere=2, stCylinder=3
                u16 flags;
                Fobb box;     // 60 bytes (15 floats)
                Fsphere sphere; // 16 bytes (4 floats)
                Fcylinder cylinder; // 32 bytes (8 floats)
            } shape;
            
            shape.type = ik_reader->r_u16();
            shape.flags = ik_reader->r_u16();
            ik_reader->r(&shape.box, sizeof(Fobb));
            ik_reader->r(&shape.sphere, sizeof(Fsphere));
            ik_reader->r(&shape.cylinder, sizeof(Fcylinder));
            
            // Check shape validity using same logic as xrSDK SBoneShape::Valid()
            bool shape_valid = true;
            switch (shape.type) {
                case 1: // stBox
                    shape_valid = !fis_zero(shape.box.m_halfsize.x) && 
                                 !fis_zero(shape.box.m_halfsize.y) && 
                                 !fis_zero(shape.box.m_halfsize.z);
                    break;
                case 2: // stSphere
                    shape_valid = !fis_zero(shape.sphere.R);
                    break;
                case 3: // stCylinder
                    shape_valid = !fis_zero(shape.cylinder.m_height) && 
                                 !fis_zero(shape.cylinder.m_radius) &&
                                 !fis_zero(shape.cylinder.m_direction.square_magnitude());
                    break;
                default: // stNone or other
                    shape_valid = true;
                    break;
            }
            
            Msg("  Bone[%d] '%s': version=0x%08X, material='%s', shape_type=%d, valid=%s, pos=%d", 
                i, bone.name.c_str(), version, game_mtl.c_str(), shape.type, 
                shape_valid ? "YES" : "NO", start_pos);
            
            if (!shape_valid) {
                // Bone has invalid shape, no IK data written - use OBB fallback
                bone.obb.xform_get(bone.bind_transform);
                Msg("    Shape invalid, using OBB: (%.3f, %.3f, %.3f)", 
                    bone.obb.m_translate.x, bone.obb.m_translate.y, bone.obb.m_translate.z);
                continue; // Skip to next bone - no IK data for this bone
            }
            
            // Skip SJointIKData::Export data (76 bytes total)
            // u32 type + 3×(2×float + 2×float) + 2×float + u32 + 2×float + float
            // = 4 + 48 + 8 + 4 + 8 + 4 = 76 bytes
            ik_reader->advance(76);
            
            // Read bind pose data (rest_rotate, rest_offset from CBone::ExportOGF)
            Fvector bind_rotation, bind_translation;
            ik_reader->r_fvector3(bind_rotation);
            ik_reader->r_fvector3(bind_translation);
            
            Msg("    bind_rotation: (%.6f, %.6f, %.6f)", bind_rotation.x, bind_rotation.y, bind_rotation.z);
            Msg("    bind_translation: (%.6f, %.6f, %.6f)", bind_translation.x, bind_translation.y, bind_translation.z);
            
            // Validate bind pose data
            bool data_corrupted = false;
            float max_reasonable = 1000.0f;
            
            if (!_finite(bind_translation.x) || !_finite(bind_translation.y) || !_finite(bind_translation.z) ||
                _abs(bind_translation.x) > max_reasonable || _abs(bind_translation.y) > max_reasonable || 
                _abs(bind_translation.z) > max_reasonable) {
                data_corrupted = true;
                Msg("    *** Bind pose data corrupted, using OBB transform");
            }
            
            if (data_corrupted) {
                // Use OBB transform as fallback
                bone.obb.xform_get(bone.bind_transform);
                Msg("    Using OBB: (%.3f, %.3f, %.3f)", 
                    bone.obb.m_translate.x, bone.obb.m_translate.y, bone.obb.m_translate.z);
            } else {
                // Build transform from bind pose data
                // NOTE: X-Ray uses XYZ rotation order
                bone.bind_transform.setXYZi(bind_rotation);
                bone.bind_transform.translate_over(bind_translation);
            }
            
            // Read mass data (1 float + 3 floats)
            bone.mass = ik_reader->r_float();
            ik_reader->r_fvector3(bone.center_of_mass);
            
            Msg("    mass: %.3f, center_of_mass: (%.3f, %.3f, %.3f)", 
                bone.mass, bone.center_of_mass.x, bone.center_of_mass.y, bone.center_of_mass.z);
        }
    }

    return bones;
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

    // Parse bone data (includes names, parent relationships, and bind poses)
    auto bone_data = reader.ReadBoneData();
    
    if (bone_data.empty()) {
        return result;
    }

    // Extract bone names and build parent indices
    result.bone_names.reserve(bone_data.size());
    result.parent_indices.reserve(bone_data.size());
    result.bind_poses.reserve(bone_data.size());
    result.root_bone_index = u16(-1);

    // First pass: collect bone names
    for (const auto& bone : bone_data) {
        result.bone_names.push_back(bone.name);
        result.bind_poses.push_back(bone.bind_transform);
    }

    // Second pass: build parent indices by looking up parent names
    for (size_t i = 0; i < bone_data.size(); ++i) {
        const auto& bone = bone_data[i];
        
        if (!bone.parent_name || !bone.parent_name[0]) {
            // No parent - this is a root bone
            result.parent_indices.push_back(-1);
            if (result.root_bone_index == u16(-1)) {
                result.root_bone_index = static_cast<u16>(i);
            } else {
                Msg("! Warning: Multiple root bones found in skeleton");
            }
        } else {
            // Find parent bone index by name
            s16 parent_idx = -1;
            for (size_t j = 0; j < result.bone_names.size(); ++j) {
                if (result.bone_names[j] == bone.parent_name) {
                    parent_idx = static_cast<s16>(j);
                    break;
                }
            }
            
            if (parent_idx == -1) {
                Msg("! Warning: Bone '%s' references unknown parent '%s'", 
                    bone.name.c_str(), bone.parent_name.c_str());
            }
            
            result.parent_indices.push_back(parent_idx);
        }
    }

    // Parse motion data
    result.motions = reader.ReadMotionData();

    // Parse motion parameters
    result.motion_params = reader.ReadMotionParams();

    // Parse IK data
    result.ik_data = reader.ReadIKData();

    // Parse user data
    result.user_data = reader.ReadUserData();

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
    Msg("* Building bone hierarchy for %d bones", bone_names.size());
    
    // First, find all root bones and create them
    xr_vector<int> root_indices;
    for (size_t i = 0; i < parent_indices.size(); ++i) {
        if (parent_indices[i] == -1) {
            root_indices.push_back(static_cast<int>(i));
        }
    }
    
    Msg("  - Found %d root bones", root_indices.size());
    skeleton.roots.resize(root_indices.size());
    
    // Recursive lambda to build joint hierarchy
    auto BuildJoint = [&](int bone_index, ozz::animation::offline::RawSkeleton::Joint& joint, auto& self) -> void {
            // Set joint properties
            joint.name = bone_names[bone_index].c_str();
            joint.transform = TransformConverter::XRayToOzz(bind_poses[bone_index]);
            
            // DEBUG: Print bone transform data
            const Fmatrix& orig_matrix = bind_poses[bone_index];
            Msg("    Bone[%d] '%s': Original pos(%.3f, %.3f, %.3f)", 
                bone_index, joint.name.c_str(), orig_matrix.c.x, orig_matrix.c.y, orig_matrix.c.z);
            Msg("                     Converted pos(%.3f, %.3f, %.3f)", 
                joint.transform.translation.x, joint.transform.translation.y, joint.transform.translation.z);
            Msg("                     Scale(%.3f, %.3f, %.3f)", 
                joint.transform.scale.x, joint.transform.scale.y, joint.transform.scale.z);
            
            // Find all children of this bone
            xr_vector<int> child_indices;
            for (size_t i = 0; i < parent_indices.size(); ++i) {
                if (parent_indices[i] == bone_index) {
                    child_indices.push_back(static_cast<int>(i));
                }
            }
            
            // Recursively build children
            if (!child_indices.empty()) {
                joint.children.resize(child_indices.size());
                for (size_t i = 0; i < child_indices.size(); ++i) {
                    self(child_indices[i], joint.children[i], self);
                }
            }
        };
    
    // Build each root and its hierarchy
    for (size_t i = 0; i < root_indices.size(); ++i) {
        Msg("  - Building hierarchy from root: %s", bone_names[root_indices[i]].c_str());
        BuildJoint(root_indices[i], skeleton.roots[i], BuildJoint);
    }
    
    // DEBUG: Calculate skeleton bounds
    float min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6, min_z = 1e6, max_z = -1e6;
    
    auto CalculateBounds = [&](const ozz::animation::offline::RawSkeleton::Joint& joint, auto& self) -> void {
            min_x = _min(min_x, joint.transform.translation.x);
            max_x = _max(max_x, joint.transform.translation.x);
            min_y = _min(min_y, joint.transform.translation.y);
            max_y = _max(max_y, joint.transform.translation.y);
            min_z = _min(min_z, joint.transform.translation.z);
            max_z = _max(max_z, joint.transform.translation.z);
            
            for (const auto& child : joint.children) {
                self(child, self);
            }
        };
    
    for (const auto& root : skeleton.roots) {
        CalculateBounds(root, CalculateBounds);
    }
    
    Msg("  - Skeleton bounds: X[%.3f, %.3f], Y[%.3f, %.3f], Z[%.3f, %.3f]", 
        min_x, max_x, min_y, max_y, min_z, max_z);
    float size_x = max_x - min_x;
    float size_y = max_y - min_y; 
    float size_z = max_z - min_z;
    float size = _max(size_x, _max(size_y, size_z));
    Msg("  - Skeleton size: %.3f units", size);
    
    // Count total joints
    int total_joints = 0;
    auto CountJoints = [&](const ozz::animation::offline::RawSkeleton::Joint& joint, auto& self) -> void {
            total_joints++;
            for (const auto& child : joint.children) {
                self(child, self);
            }
        };
    
    for (const auto& root : skeleton.roots) {
        CountJoints(root, CountJoints);
    }
    
    Msg("* Skeleton hierarchy built - roots: %d, total joints: %d", skeleton.roots.size(), total_joints);
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
        u8 ogf_type = reader_.GetHeader().type;
        
        // Log header info
        Msg("* OGF Header Info:");
        Msg("  - Format version: %d", reader_.GetHeader().format_version);
        Msg("  - Type: %d", ogf_type);
        Msg("  - Shader ID: %d", reader_.GetHeader().shader_id);
        
        // Check if it's a skeleton type or hierarchical mesh that might contain skeleton
        if (ogf_type != MT_SKELETON_ANIM && ogf_type != MT_SKELETON_RIGID && 
            ogf_type != MT_SKELETON_GEOMDEF_PM && ogf_type != MT_SKELETON_GEOMDEF_ST &&
            ogf_type != MT_NORMAL && ogf_type != MT_HIERRARHY) {
            string256 error_buf;
            xr_sprintf(error_buf, "OGF file is not a supported type (type=%d)", ogf_type);
            result.error_message = error_buf;
            return result;
        }
        
        // Debug: List all chunks in the file
        reader_.DebugListChunks();
        
        // For hierarchical meshes, we need to look into children chunks
        if (ogf_type == MT_HIERRARHY) {
            Msg("* This is a hierarchical mesh - checking for skeleton in children chunks...");
            // TODO: Parse OGF_CHILDREN chunk and look for skeleton data
        }

        // Parse OGF data
        auto parse_result = parser_.Parse(reader_);
        
        // Check if we actually found skeleton data
        if (parse_result.bone_names.empty()) {
            if (ogf_type == MT_NORMAL) {
                result.error_message = "This is a regular mesh file (MT_NORMAL), not a skeleton. Please use a skeleton OGF file.";
            } else {
                result.error_message = "No bones found in OGF file";
            }
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
