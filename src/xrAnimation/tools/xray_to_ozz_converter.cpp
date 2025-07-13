#include "../stdafx.h"
#include "../OGFConverter.h"
#include "../OMFConverter.h"
#include "xrCore/xrCore.h"
#include "xrCore/FS.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/animation/offline/raw_skeleton.h"
#include "ozz/animation/offline/raw_animation.h"
#include "ozz/animation/offline/skeleton_builder.h"
#include "ozz/animation/offline/animation_builder.h"

#include <sys/stat.h>
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/types.h>
#endif

#include <functional>
#include <vector>

using namespace XRay::Animation;

int CountJoints(const ozz::animation::offline::RawSkeleton& skeleton) {
    int count = 0;
    std::function<void(const ozz::animation::offline::RawSkeleton::Joint&)> CountJoint = 
        [&](const ozz::animation::offline::RawSkeleton::Joint& joint) {
            count++;
            for (const auto& child : joint.children) {
                CountJoint(child);
            }
        };
    
    for (const auto& root : skeleton.roots) {
        CountJoint(root);
    }
    
    return count;
}

void PrintUsage(const char* program_name)
{
    Msg("Usage: %s <command> <input> <output> [options]", program_name);
    Msg("\nCommands:");
    Msg("  skeleton  - Convert OGF skeleton to ozz skeleton");
    Msg("  animation - Convert OMF animation to ozz animation");
    Msg("  batch     - Convert all animations in a directory");
    Msg("  analyze   - Analyze OMF file structure (no conversion)");
    Msg("\nOptions:");
    Msg("  -optimize - Optimize animations (remove redundant keys)");
    Msg("\nOutput:");
    Msg("  <output> can be either:");
    Msg("    - A filename (e.g., output.ozz)");
    Msg("    - A directory (input filename will be used with .ozz extension)");
    Msg("  -compress - Use compression for ozz files");
    Msg("\nExamples:");
    Msg("  %s skeleton actor.ogf actor_skeleton.ozz", program_name);
    Msg("  %s animation walk.omf walk.ozz -optimize", program_name);
    Msg("  %s batch animations/ ozz_animations/ -optimize", program_name);
    Msg("  %s analyze stalker_animation.omf", program_name);
}

void AnalyzeOMF(const std::string& input_path)
{
    Msg("Analyzing OMF file: %s", input_path.c_str());
    
    FILE* file = fopen(input_path.c_str(), "rb");
    if (!file) {
        Msg("! Failed to open file");
        return;
    }
    
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    Msg("* File size: %ld bytes", file_size);
    
    while (ftell(file) < file_size - 8) {
        u32 chunk_id, chunk_size;
        if (fread(&chunk_id, 4, 1, file) != 1) break;
        if (fread(&chunk_size, 4, 1, file) != 1) break;
        
        long chunk_pos = ftell(file) - 8;
        Msg("* Chunk 0x%04X at offset %ld, size %u", chunk_id, chunk_pos, chunk_size);
        
        if (chunk_id == XRay::Animation::OGF_S_SMPARAMS) {
            u16 version;
            fread(&version, 2, 1, file);
            Msg("  - OGF_S_SMPARAMS version: %u", version);
            fseek(file, chunk_pos + 8 + chunk_size, SEEK_SET);
        } else if (chunk_id == XRay::Animation::OGF_S_MOTIONS) {
            long motion_start = ftell(file);
            u32 sub_chunk_id, sub_chunk_size;
            
            if (fread(&sub_chunk_id, 4, 1, file) == 1 &&
                fread(&sub_chunk_size, 4, 1, file) == 1) {
                if (sub_chunk_id == 0) {
                    u32 motion_count;
                    fread(&motion_count, 4, 1, file);
                    Msg("  - Motion count: %u", motion_count);
                }
            }
            fseek(file, motion_start + chunk_size, SEEK_SET);
        } else {
            fseek(file, chunk_size, SEEK_CUR);
        }
        
        if (ftell(file) >= file_size) break;
    }
    
    fclose(file);
}

bool ConvertSkeleton(const std::string& input_path, const std::string& output_path)
{
    // If output_path is a directory, use input filename with .ozz extension
    std::string final_output_path = output_path;
    struct stat path_stat;
    if (stat(output_path.c_str(), &path_stat) == 0 && S_ISDIR(path_stat.st_mode)) {
        // Extract filename from input path
        size_t last_slash = input_path.find_last_of("/\\");
        std::string filename = (last_slash != std::string::npos) ? input_path.substr(last_slash + 1) : input_path;
        
        // Replace extension with .ozz
        size_t dot_pos = filename.find_last_of('.');
        if (dot_pos != std::string::npos) {
            filename = filename.substr(0, dot_pos) + ".ozz";
        } else {
            filename += ".ozz";
        }
        
        final_output_path = output_path + "/" + filename;
    }
    
    Msg("Converting skeleton: %s -> %s", input_path.c_str(), final_output_path.c_str());
    
    // Create OGF converter
    OGFConverter converter;
    
    // Convert OGF file
    shared_str input_str(input_path.c_str());
    auto result = converter.Convert(input_str);
    
    if (!result.success) {
        Msg("! Failed to convert skeleton: %s", result.error_message.c_str());
        return false;
    }
    
    // Build runtime skeleton from raw skeleton
    ozz::animation::offline::SkeletonBuilder builder;
    auto skeleton = builder(result.skeleton);
    if (!skeleton) {
        Msg("! Failed to build runtime skeleton");
        return false;
    }
    
    // Save ozz skeleton
    ozz::io::File file(final_output_path.c_str(), "wb");
    if (!file.opened()) {
        Msg("! Failed to create output file: %s", final_output_path.c_str());
        return false;
    }
    
    ozz::io::OArchive archive(&file);
    archive << *skeleton;
    
    Msg("* Skeleton converted successfully");
    Msg("  - Raw skeleton joints before build: %d", CountJoints(result.skeleton));
    Msg("  - Runtime skeleton joints: %d", skeleton->num_joints());
    Msg("  - Root bones: %d", result.skeleton.roots.size());
    
    // Save metadata alongside skeleton (optional)
    std::string metadata_path = output_path + ".meta";
    IWriter* writer = FS.w_open(metadata_path.c_str());
    if (writer) {
        // Save X-Ray specific metadata
        writer->w_u32(result.skeleton.roots.size());
        for (const auto& motion_param : result.metadata.motion_params) {
            writer->w_stringZ(motion_param.first.c_str());
            writer->w_float(motion_param.second.speed);
            writer->w_float(motion_param.second.power);
            writer->w_float(motion_param.second.accrue);
            writer->w_float(motion_param.second.falloff);
        }
        FS.w_close(writer);
    }
    
    return true;
}

bool ConvertAnimation(const std::string& input_path, const std::string& output_path, 
                     const std::string& skeleton_path, bool optimize)
{
    // If output_path is a directory, use input filename with .ozz extension
    std::string final_output_path = output_path;
    struct stat path_stat;
    if (stat(output_path.c_str(), &path_stat) == 0 && S_ISDIR(path_stat.st_mode)) {
        // Extract filename from input path
        size_t last_slash = input_path.find_last_of("/\\");
        std::string filename = (last_slash != std::string::npos) ? input_path.substr(last_slash + 1) : input_path;
        
        // Replace extension with .ozz
        size_t dot_pos = filename.find_last_of('.');
        if (dot_pos != std::string::npos) {
            filename = filename.substr(0, dot_pos) + ".ozz";
        } else {
            filename += ".ozz";
        }
        
        final_output_path = output_path + "/" + filename;
    }
    
    Msg("Converting animation: %s -> %s", input_path.c_str(), final_output_path.c_str());
    
    // We need both the raw skeleton and bind poses
    ozz::animation::offline::RawSkeleton raw_skeleton;
    ozz::animation::Skeleton skeleton;
    xr_vector<ozz::math::Transform> bind_poses;
    
    // Check if skeleton_path is OGF or ozz
    bool is_ogf = skeleton_path.find(".ogf") != std::string::npos;
    
    if (is_ogf) {
        // Load OGF file to get skeleton and bind poses
        Msg("* Loading OGF skeleton with bind poses: %s", skeleton_path.c_str());
        
        OGFConverter ogf_converter;
        shared_str skeleton_str(skeleton_path.c_str());
        auto ogf_result = ogf_converter.Convert(skeleton_str);
        
        if (!ogf_result.success) {
            Msg("! Failed to load OGF skeleton: %s", ogf_result.error_message.c_str());
            return false;
        }
        
        raw_skeleton = ogf_result.skeleton;
        
        // Extract bind poses from OGF result
        // The OGF converter stores bind poses in the metadata or we need to extract them
        // For now, we'll build them from the raw skeleton's rest poses
        bind_poses.resize(CountJoints(raw_skeleton));
        
        // Traverse skeleton to extract bind poses
        struct ExtractBindPoses {
            static void Extract(const ozz::animation::offline::RawSkeleton::Joint& joint, 
                              xr_vector<ozz::math::Transform>& poses, int& index) {
                if (index < poses.size()) {
                    poses[index] = joint.transform;
                    index++;
                }
                for (const auto& child : joint.children) {
                    Extract(child, poses, index);
                }
            }
        };
        
        int pose_index = 0;
        for (const auto& root : raw_skeleton.roots) {
            ExtractBindPoses::Extract(root, bind_poses, pose_index);
        }
        
        // Build runtime skeleton for optimization
        ozz::animation::offline::SkeletonBuilder builder;
        auto built_skeleton = builder(raw_skeleton);
        if (!built_skeleton) {
            Msg("! Failed to build runtime skeleton from OGF");
            return false;
        }
        skeleton = std::move(*built_skeleton);
        
    } else {
        // Load pre-converted ozz skeleton
        ozz::io::File skel_file(skeleton_path.c_str(), "rb");
        if (!skel_file.opened()) {
            Msg("! Failed to open skeleton file: %s", skeleton_path.c_str());
            return false;
        }
        
        ozz::io::IArchive skel_archive(&skel_file);
        if (!skel_archive.TestTag<ozz::animation::Skeleton>()) {
            Msg("! Invalid skeleton file format");
            return false;
        }
        skel_archive >> skeleton;
        
        // Create raw skeleton from runtime skeleton (simplified)
        // WARNING: This loses bind pose information!
        Msg("! WARNING: Using .ozz skeleton file - bind poses not available!");
        Msg("! For proper delta animation conversion, use the original .ogf skeleton file");
        
        raw_skeleton.roots.resize(1);
        raw_skeleton.roots[0].name = "root";
        for (int i = 0; i < skeleton.num_joints(); ++i) {
            if (i > 0) {
                ozz::animation::offline::RawSkeleton::Joint joint;
                joint.name = skeleton.joint_names()[i];
                raw_skeleton.roots[0].children.push_back(joint);
            }
        }
        
        // Use identity transforms as fallback
        bind_poses.resize(skeleton.num_joints());
        for (auto& pose : bind_poses) {
            pose.translation = ozz::math::Float3::zero();
            pose.rotation = ozz::math::Quaternion::identity();
            pose.scale = ozz::math::Float3::one();
        }
    }
    
    // Create OMF converter
    OMFConverter converter;
    
    // Convert with skeleton and bind poses
    shared_str input_str(input_path.c_str());
    auto result = converter.ConvertWithSkeletonAndBindPoses(input_str, raw_skeleton, bind_poses);
    
    if (!result.success) {
        Msg("! Failed to convert animation: %s", result.error_message.c_str());
        return false;
    }
    
    if (result.animations.empty()) {
        Msg("! No animations found in OMF file");
        return false;
    }
    
    // Build all animations first
    std::vector<ozz::animation::Animation> animations;
    animations.reserve(result.animations.size());
    
    int anim_idx = 0;
    for (auto& raw_animation : result.animations) {
        // Validate animation
        if (!raw_animation.Validate()) {
            Msg("! Animation %d is invalid", anim_idx);
            anim_idx++;
            continue;
        }
        
        // Optimize if requested
        if (optimize) {
            Msg("* Optimizing animation %d...", anim_idx);
            ozz::animation::offline::AnimationOptimizer optimizer;
            ozz::animation::offline::RawAnimation optimized;
            
            if (optimizer(raw_animation, skeleton, &optimized)) {
                raw_animation = std::move(optimized);
                Msg("  - Optimization complete");
            } else {
                Msg("! Optimization failed for animation %d", anim_idx);
            }
        }
        
        // Build runtime animation
        ozz::animation::offline::AnimationBuilder builder;
        auto animation = builder(raw_animation);
        if (!animation) {
            Msg("! Failed to build animation %d", anim_idx);
            anim_idx++;
            continue;
        }
        
        animations.push_back(std::move(*animation));
        anim_idx++;
    }
    
    if (animations.empty()) {
        Msg("! No valid animations to save");
        return false;
    }
    
    // Ensure output path has .ozz extension
    std::string output_with_ext = final_output_path;
    if (final_output_path.find(".ozz") == std::string::npos) {
        output_with_ext = final_output_path + ".ozz";
    }
    
    // Save all animations to a single file
    ozz::io::File file(output_with_ext.c_str(), "wb");
    if (!file.opened()) {
        Msg("! Failed to create output file: %s", output_with_ext.c_str());
        return false;
    }
    
    ozz::io::OArchive archive(&file);
    
    // First, save the count of animations
    const uint32_t anim_count = static_cast<uint32_t>(animations.size());
    archive << anim_count;
    
    // Then save each animation
    for (const auto& animation : animations) {
        archive << animation;
    }
    
    Msg("* Saved %d animations to %s", anim_count, output_with_ext.c_str());
    
    // Save animation metadata
    std::string meta_path = output_with_ext.substr(0, output_with_ext.rfind('.')) + ".meta";
    ozz::io::File meta_file(meta_path.c_str(), "wb");
    if (meta_file.opened()) {
        ozz::io::OArchive meta_archive(&meta_file);
        const uint32_t anim_count = static_cast<uint32_t>(animations.size());
        meta_archive << anim_count;
        
        // Save animation names from raw animations
        for (size_t i = 0; i < result.animations.size() && i < animations.size(); ++i) {
            std::string anim_name = result.animations[i].name.c_str();
            uint32_t name_len = static_cast<uint32_t>(anim_name.size());
            meta_archive << name_len;
            if (name_len > 0) {
                meta_archive << ozz::io::MakeArray(anim_name.data(), name_len);
            }
            Msg("  - Animation %d: '%s'", i, anim_name.c_str());
        }
        
        Msg("* Saved metadata to %s", meta_path.c_str());
    }
    
    return true;
}

bool ConvertBatch(const std::string& input_dir, const std::string& output_dir, 
                 const std::string& skeleton_path, bool optimize)
{
    Msg("Batch converting animations from: %s", input_dir.c_str());
    
    // Create output directory
    // Create output directory
    // Use platform-specific directory creation
#ifdef _WIN32
    _mkdir(output_dir.c_str());
#else
    mkdir(output_dir.c_str(), 0755);
#endif
    
    // Find all OMF files
    FS_FileSet files;
    FS.file_list(files, input_dir.c_str(), FS_ListFiles, "*.omf");
    
    if (files.empty()) {
        Msg("! No OMF files found in: %s", input_dir.c_str());
        return false;
    }
    
    Msg("* Found %d animation files", files.size());
    
    int converted = 0;
    for (const auto& file : files) {
        std::string input_path = input_dir + "/" + file.name.c_str();
        std::string output_name = file.name.c_str();
        
        // Change extension to .ozz
        size_t dot_pos = output_name.find_last_of('.');
        if (dot_pos != std::string::npos) {
            output_name = output_name.substr(0, dot_pos) + ".ozz";
        }
        
        std::string output_path = output_dir + "/" + output_name;
        
        Msg("\n[%d/%d] %s", converted + 1, files.size(), file.name.c_str());
        
        if (ConvertAnimation(input_path, output_path, skeleton_path, optimize)) {
            converted++;
        }
    }
    
    // Also look for OGF skeleton files if requested
    FS_FileSet ogf_files;
    FS.file_list(ogf_files, input_dir.c_str(), FS_ListFiles, "*.ogf");
    
    if (!ogf_files.empty()) {
        Msg("\n* Found %d OGF skeleton files", ogf_files.size());
        
        for (const auto& file : ogf_files) {
            std::string input_path = input_dir + "/" + file.name.c_str();
            std::string output_name = file.name.c_str();
            
            // Change extension to .ozz
            size_t dot_pos = output_name.find_last_of('.');
            if (dot_pos != std::string::npos) {
                output_name = output_name.substr(0, dot_pos) + "_skeleton.ozz";
            }
            
            std::string output_path = output_dir + "/" + output_name;
            
            Msg("\n[Skeleton] %s", file.name.c_str());
            
            if (ConvertSkeleton(input_path, output_path)) {
                Msg("  - Skeleton converted: %s", output_name.c_str());
            }
        }
    }
    
    Msg("\n* Batch conversion complete: %d/%d animation files converted", converted, files.size());
    
    return converted > 0;
}

int main(int argc, char* argv[])
{
    // Initialize core
    xrDebug::Initialize(argv[0] ? argv[0] : "xray_to_ozz");
    Core.Initialize("xray_to_ozz", nullptr, FALSE, nullptr);
    
    Msg("==================================================");
    Msg("     X-Ray to ozz-animation Converter");
    Msg("==================================================\n");
    
    if (argc < 3) {
        PrintUsage(argv[0]);
        Core._destroy();
        return 1;
    }
    
    std::string command = argv[1];
    std::string input = argv[2];
    std::string output = argc > 3 ? argv[3] : "";
    
    bool optimize = false;
    bool compress = false;
    
    // Parse options
    for (int i = 4; i < argc; ++i) {
        if (strcmp(argv[i], "-optimize") == 0) {
            optimize = true;
        } else if (strcmp(argv[i], "-compress") == 0) {
            compress = true;
        }
    }
    
    bool success = false;
    
    if (command == "skeleton") {
        success = ConvertSkeleton(input, output);
    } else if (command == "animation") {
        if (argc < 5) {
            Msg("! Animation conversion requires skeleton path");
            Msg("  Usage: %s animation <omf> <ozz> <skeleton.ozz>", argv[0]);
        } else {
            std::string skeleton_path = argv[4];
            success = ConvertAnimation(input, output, skeleton_path, optimize);
        }
    } else if (command == "batch") {
        if (argc < 5) {
            Msg("! Batch conversion requires skeleton path");
            Msg("  Usage: %s batch <input_dir> <output_dir> <skeleton.ozz>", argv[0]);
        } else {
            std::string skeleton_path = argv[4];
            success = ConvertBatch(input, output, skeleton_path, optimize);
        }
    } else if (command == "analyze") {
        AnalyzeOMF(input);
        success = true;
    } else {
        Msg("! Unknown command: %s", command.c_str());
        PrintUsage(argv[0]);
    }
    
    // Cleanup
    Core._destroy();
    
    return success ? 0 : 1;
}