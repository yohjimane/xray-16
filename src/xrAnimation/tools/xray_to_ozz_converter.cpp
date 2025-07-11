#include "../stdafx.h"
#include "../OGFConverter.h"
#include "../AnimationConverter.h"
#include "xrCore/xrCore.h"
#include "xrCore/FS.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"

using namespace XRay::Animation;

void PrintUsage(const char* program_name)
{
    Msg("Usage: %s <command> <input> <output> [options]", program_name);
    Msg("\nCommands:");
    Msg("  skeleton  - Convert OGF skeleton to ozz skeleton");
    Msg("  animation - Convert OMF animation to ozz animation");
    Msg("  batch     - Convert all animations in a directory");
    Msg("\nOptions:");
    Msg("  -optimize - Optimize animations (remove redundant keys)");
    Msg("  -compress - Use compression for ozz files");
    Msg("\nExamples:");
    Msg("  %s skeleton actor.ogf actor_skeleton.ozz", program_name);
    Msg("  %s animation walk.omf walk.ozz -optimize", program_name);
    Msg("  %s batch animations/ ozz_animations/ -optimize", program_name);
}

bool ConvertSkeleton(const std::string& input_path, const std::string& output_path)
{
    Msg("Converting skeleton: %s -> %s", input_path.c_str(), output_path.c_str());
    
    // Create OGF converter
    OGFConverter converter;
    
    // Load OGF file
    if (!converter.LoadFromFile(input_path)) {
        Msg("! Failed to load OGF file: %s", input_path.c_str());
        return false;
    }
    
    // Convert to ozz skeleton
    auto result = converter.Convert();
    if (!result.skeleton) {
        Msg("! Failed to convert skeleton");
        return false;
    }
    
    // Save ozz skeleton
    ozz::io::File file(output_path.c_str(), "wb");
    if (!file.opened()) {
        Msg("! Failed to create output file: %s", output_path.c_str());
        return false;
    }
    
    ozz::io::OArchive archive(&file);
    archive << *result.skeleton;
    
    Msg("* Skeleton converted successfully");
    Msg("  - Joints: %d", result.skeleton->num_joints());
    
    return true;
}

bool ConvertAnimation(const std::string& input_path, const std::string& output_path, 
                     const std::string& skeleton_path, bool optimize)
{
    Msg("Converting animation: %s -> %s", input_path.c_str(), output_path.c_str());
    
    // Load skeleton first
    ozz::animation::Skeleton skeleton;
    {
        ozz::io::File skel_file(skeleton_path.c_str(), "rb");
        if (!skel_file.opened()) {
            Msg("! Failed to open skeleton file: %s", skeleton_path.c_str());
            return false;
        }
        
        ozz::io::IArchive skel_archive(&skel_file);
        skel_archive >> skeleton;
    }
    
    // Create animation converter
    AnimationConverter converter;
    
    // Read OMF file
    IReader* reader = FS.r_open(input_path.c_str());
    if (!reader) {
        Msg("! Failed to open OMF file: %s", input_path.c_str());
        return false;
    }
    
    // Parse OMF header
    u32 version = reader->r_u32();
    if (version != 0x10) {
        Msg("! Unsupported OMF version: 0x%x", version);
        FS.r_close(reader);
        return false;
    }
    
    u16 bone_count = reader->r_u16();
    u32 frame_count = reader->r_u32();
    float fps = static_cast<float>(reader->r_u32());
    
    Msg("* OMF info: bones=%d, frames=%d, fps=%.1f", bone_count, frame_count, fps);
    
    // Read motion data
    xr_vector<xr_vector<Fvector>> positions(bone_count);
    xr_vector<xr_vector<Fquaternion>> rotations(bone_count);
    
    for (u16 bone = 0; bone < bone_count; ++bone) {
        positions[bone].resize(frame_count);
        rotations[bone].resize(frame_count);
        
        for (u32 frame = 0; frame < frame_count; ++frame) {
            // Read position
            positions[bone][frame].x = reader->r_float();
            positions[bone][frame].y = reader->r_float();
            positions[bone][frame].z = reader->r_float();
            
            // Read rotation (quaternion)
            rotations[bone][frame].x = reader->r_float();
            rotations[bone][frame].y = reader->r_float();
            rotations[bone][frame].z = reader->r_float();
            rotations[bone][frame].w = reader->r_float();
        }
    }
    
    FS.r_close(reader);
    
    // Convert to ozz animation
    ozz::animation::offline::RawAnimation raw_animation;
    raw_animation.duration = frame_count / fps;
    raw_animation.tracks.resize(bone_count);
    
    for (u16 bone = 0; bone < bone_count; ++bone) {
        auto& track = raw_animation.tracks[bone];
        
        for (u32 frame = 0; frame < frame_count; ++frame) {
            float time = frame / fps;
            
            // Add translation key
            ozz::animation::offline::RawAnimation::TranslationKey trans_key;
            trans_key.time = time;
            trans_key.value = ozz::math::Float3(
                positions[bone][frame].x,
                positions[bone][frame].y,
                positions[bone][frame].z
            );
            track.translations.push_back(trans_key);
            
            // Add rotation key
            ozz::animation::offline::RawAnimation::RotationKey rot_key;
            rot_key.time = time;
            rot_key.value = ozz::math::Quaternion(
                rotations[bone][frame].x,
                rotations[bone][frame].y,
                rotations[bone][frame].z,
                rotations[bone][frame].w
            );
            track.rotations.push_back(rot_key);
        }
    }
    
    // Optimize if requested
    if (optimize) {
        Msg("* Optimizing animation...");
        ozz::animation::offline::AnimationOptimizer optimizer;
        ozz::animation::offline::RawAnimation optimized;
        
        if (optimizer(raw_animation, skeleton, &optimized)) {
            raw_animation = std::move(optimized);
            Msg("  - Optimization complete");
        } else {
            Msg("! Optimization failed");
        }
    }
    
    // Build runtime animation
    ozz::animation::offline::AnimationBuilder builder;
    auto animation = builder(raw_animation);
    if (!animation) {
        Msg("! Failed to build animation");
        return false;
    }
    
    // Save ozz animation
    ozz::io::File file(output_path.c_str(), "wb");
    if (!file.opened()) {
        Msg("! Failed to create output file: %s", output_path.c_str());
        return false;
    }
    
    ozz::io::OArchive archive(&file);
    archive << *animation;
    
    Msg("* Animation converted successfully");
    Msg("  - Duration: %.2fs", animation->duration());
    Msg("  - Tracks: %d", animation->num_tracks());
    
    return true;
}

bool ConvertBatch(const std::string& input_dir, const std::string& output_dir, 
                 const std::string& skeleton_path, bool optimize)
{
    Msg("Batch converting animations from: %s", input_dir.c_str());
    
    // Create output directory
    FS.dir_create(output_dir.c_str());
    
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
        
        if (ConvertAnimation(input_path, output_path, skeleton_path, optimize)) {
            converted++;
        }
    }
    
    Msg("\n* Batch conversion complete: %d/%d files converted", converted, files.size());
    
    return converted > 0;
}

int main(int argc, char* argv[])
{
    // Initialize core
    Debug._initialize(false);
    Core._initialize("xray_to_ozz", nullptr, TRUE, "fs.ltx");
    
    Msg("==================================================");
    Msg("     X-Ray to ozz-animation Converter");
    Msg("==================================================\n");
    
    if (argc < 4) {
        PrintUsage(argv[0]);
        Core._destroy();
        return 1;
    }
    
    std::string command = argv[1];
    std::string input = argv[2];
    std::string output = argv[3];
    
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
    } else {
        Msg("! Unknown command: %s", command.c_str());
        PrintUsage(argv[0]);
    }
    
    // Cleanup
    Core._destroy();
    
    return success ? 0 : 1;
}