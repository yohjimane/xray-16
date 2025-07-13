#include "stdafx.h"
#include "../OGFConverter.h"
#include <iostream>
#include <iomanip>
#include <map>
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/maths/transform.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/soa_transform.h"

void PrintBoneTransform(const shared_str& name, const Fmatrix& m) {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  Bone: " << name.c_str() << std::endl;
    std::cout << "    Position: (" << m.c.x << ", " << m.c.y << ", " << m.c.z << ")" << std::endl;
    std::cout << "    Matrix:" << std::endl;
    std::cout << "      [" << m.i.x << ", " << m.j.x << ", " << m.k.x << ", " << m.c.x << "]" << std::endl;
    std::cout << "      [" << m.i.y << ", " << m.j.y << ", " << m.k.y << ", " << m.c.y << "]" << std::endl;
    std::cout << "      [" << m.i.z << ", " << m.j.z << ", " << m.k.z << ", " << m.c.z << "]" << std::endl;
}

void CompareSkeletons(const std::string& ogf_path, const std::string& ozz_skeleton_path) {
    std::cout << "=== Bind Pose Comparison ===" << std::endl << std::endl;
    
    // Load original OGF skeleton using our OGFConverter
    std::cout << "Loading original OGF skeleton from: " << ogf_path << std::endl;
    XRay::Animation::OGFReader ogf_reader;
    if (!ogf_reader.LoadFromFile(shared_str(ogf_path.c_str()))) {
        std::cerr << "Failed to load OGF file: " << ogf_path << std::endl;
        return;
    }
    
    auto bones = ogf_reader.ReadBoneData();
    std::cout << "Found " << bones.size() << " bones in OGF" << std::endl << std::endl;
    
    // Load ozz skeleton
    std::cout << "Loading ozz skeleton..." << std::endl;
    ozz::animation::Skeleton skeleton;
    {
        ozz::io::File file(ozz_skeleton_path.c_str(), "rb");
        if (!file.opened()) {
            std::cerr << "Failed to open ozz skeleton file" << std::endl;
            return;
        }
        
        ozz::io::IArchive archive(&file);
        if (!archive.TestTag<ozz::animation::Skeleton>()) {
            std::cerr << "Invalid ozz skeleton file" << std::endl;
            return;
        }
        
        archive >> skeleton;
    }
    
    std::cout << "ozz skeleton has " << skeleton.num_joints() << " joints" << std::endl << std::endl;
    
    // Compare bind poses
    std::cout << "=== Original X-Ray Bind Poses (First 10) ===" << std::endl;
    for (size_t i = 0; i < std::min(size_t(10), bones.size()); ++i) {
        PrintBoneTransform(bones[i].name, bones[i].bind_transform);
    }
    
    std::cout << std::endl << "=== ozz Skeleton Bind Poses (First 10) ===" << std::endl;
    
    // Extract ozz bind poses
    const ozz::span<const ozz::math::SoaTransform>& joint_bind_poses = skeleton.joint_rest_poses();
    const ozz::span<const char* const> joint_names = skeleton.joint_names();
    
    for (int i = 0; i < std::min(10, skeleton.num_joints()); ++i) {
        int soa_index = i / 4;
        int soa_element = i % 4;
        
        const ozz::math::SoaTransform& soa_transform = joint_bind_poses[soa_index];
        
        // Extract translation
        float tx, ty, tz;
        switch (soa_element) {
            case 0:
                tx = ozz::math::GetX(soa_transform.translation.x);
                ty = ozz::math::GetX(soa_transform.translation.y);
                tz = ozz::math::GetX(soa_transform.translation.z);
                break;
            case 1:
                tx = ozz::math::GetY(soa_transform.translation.x);
                ty = ozz::math::GetY(soa_transform.translation.y);
                tz = ozz::math::GetY(soa_transform.translation.z);
                break;
            case 2:
                tx = ozz::math::GetZ(soa_transform.translation.x);
                ty = ozz::math::GetZ(soa_transform.translation.y);
                tz = ozz::math::GetZ(soa_transform.translation.z);
                break;
            case 3:
                tx = ozz::math::GetW(soa_transform.translation.x);
                ty = ozz::math::GetW(soa_transform.translation.y);
                tz = ozz::math::GetW(soa_transform.translation.z);
                break;
        }
        
        std::cout << "  Joint " << i << ": " << joint_names[i] << std::endl;
        std::cout << "    Position: (" << tx << ", " << ty << ", " << tz << ")" << std::endl;
    }
    
    // Compare specific bones
    std::cout << std::endl << "=== Position Differences (First 10 matching bones) ===" << std::endl;
    
    // Map ozz joints by name for easy lookup
    std::map<std::string, int> ozz_joint_map;
    for (int i = 0; i < skeleton.num_joints(); ++i) {
        ozz_joint_map[joint_names[i]] = i;
    }
    
    for (size_t i = 0; i < std::min(size_t(10), bones.size()); ++i) {
        const auto& ogf_bone = bones[i];
        auto it = ozz_joint_map.find(ogf_bone.name.c_str());
        
        if (it != ozz_joint_map.end()) {
            int joint_idx = it->second;
            int soa_index = joint_idx / 4;
            int soa_element = joint_idx % 4;
            
            const ozz::math::SoaTransform& soa_transform = joint_bind_poses[soa_index];
            
            float ozz_x, ozz_y, ozz_z;
            switch (soa_element) {
                case 0:
                    ozz_x = ozz::math::GetX(soa_transform.translation.x);
                    ozz_y = ozz::math::GetX(soa_transform.translation.y);
                    ozz_z = ozz::math::GetX(soa_transform.translation.z);
                    break;
                case 1:
                    ozz_x = ozz::math::GetY(soa_transform.translation.x);
                    ozz_y = ozz::math::GetY(soa_transform.translation.y);
                    ozz_z = ozz::math::GetY(soa_transform.translation.z);
                    break;
                case 2:
                    ozz_x = ozz::math::GetZ(soa_transform.translation.x);
                    ozz_y = ozz::math::GetZ(soa_transform.translation.y);
                    ozz_z = ozz::math::GetZ(soa_transform.translation.z);
                    break;
                case 3:
                    ozz_x = ozz::math::GetW(soa_transform.translation.x);
                    ozz_y = ozz::math::GetW(soa_transform.translation.y);
                    ozz_z = ozz::math::GetW(soa_transform.translation.z);
                    break;
            }
            
            float dx = ozz_x - ogf_bone.bind_transform.c.x;
            float dy = ozz_y - ogf_bone.bind_transform.c.y;
            float dz = ozz_z - ogf_bone.bind_transform.c.z;
            
            std::cout << "  " << ogf_bone.name.c_str() << ":" << std::endl;
            std::cout << "    OGF: (" << ogf_bone.bind_transform.c.x << ", " 
                      << ogf_bone.bind_transform.c.y << ", " 
                      << ogf_bone.bind_transform.c.z << ")" << std::endl;
            std::cout << "    ozz: (" << ozz_x << ", " << ozz_y << ", " << ozz_z << ")" << std::endl;
            std::cout << "    Diff: (" << dx << ", " << dy << ", " << dz << ")" << std::endl;
        }
    }
    
    // Additional analysis: Show world space transforms
    std::cout << std::endl << "=== World Space Transforms (First 5 bones) ===" << std::endl;
    
    // Build parent map for X-Ray bones
    std::map<shared_str, size_t> bone_name_to_idx;
    for (size_t i = 0; i < bones.size(); ++i) {
        bone_name_to_idx[bones[i].name] = i;
    }
    
    // Compute world transforms
    xr_vector<Fmatrix> world_transforms(bones.size());
    for (size_t i = 0; i < bones.size(); ++i) {
        const auto& bone = bones[i];
        
        if (bone.parent_name.size() == 0) {
            // Root bone
            world_transforms[i] = bone.bind_transform;
        } else {
            // Find parent
            auto parent_it = bone_name_to_idx.find(bone.parent_name);
            if (parent_it != bone_name_to_idx.end()) {
                Fmatrix parent_world = world_transforms[parent_it->second];
                world_transforms[i].mul(parent_world, bone.bind_transform);
            } else {
                // No parent found, use local as world
                world_transforms[i] = bone.bind_transform;
            }
        }
        
        if (i < 5) {
            std::cout << "  Bone " << i << " (" << bone.name.c_str() << "):" << std::endl;
            std::cout << "    Local: (" << bone.bind_transform.c.x << ", " 
                      << bone.bind_transform.c.y << ", " 
                      << bone.bind_transform.c.z << ")" << std::endl;
            std::cout << "    World: (" << world_transforms[i].c.x << ", " 
                      << world_transforms[i].c.y << ", " 
                      << world_transforms[i].c.z << ")" << std::endl;
        }
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Test bind pose comparison starting..." << std::endl;
    
    if (argc != 3) {
        std::cout << "Usage: " << argv[0] << " <ogf_file> <ozz_skeleton_file>" << std::endl;
        return 1;
    }
    
    std::cout << "Arguments:" << std::endl;
    std::cout << "  OGF file: " << argv[1] << std::endl;
    std::cout << "  ozz file: " << argv[2] << std::endl;
    
    // Initialize minimal memory system for shared_str
    Memory._initialize();
    
    try {
        CompareSkeletons(std::string(argv[1]), std::string(argv[2]));
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception occurred" << std::endl;
        return 1;
    }
    
    return 0;
}