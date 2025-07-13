#include <iostream>
#include <fstream>
#include <vector>
#include <ozz/animation/runtime/skeleton.h>
#include <ozz/animation/runtime/animation.h>
#include <ozz/animation/runtime/sampling_job.h>
#include <ozz/animation/runtime/local_to_model_job.h>
#include <ozz/base/io/stream.h>
#include <ozz/base/io/archive.h>
#include <ozz/base/maths/soa_transform.h>
#include <ozz/base/span.h>
#include <ozz/base/memory/allocator.h>
#include <ozz/base/containers/vector.h>

// Simple skeleton viewer that shows bind pose
int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <skeleton.ozz>" << std::endl;
        return 1;
    }

    // Load skeleton
    ozz::animation::Skeleton skeleton;
    {
        ozz::io::File file(argv[1], "rb");
        if (!file.opened()) {
            std::cerr << "Failed to open skeleton file: " << argv[1] << std::endl;
            return 1;
        }
        
        ozz::io::IArchive archive(&file);
        if (!archive.TestTag<ozz::animation::Skeleton>()) {
            std::cerr << "Invalid skeleton file format" << std::endl;
            return 1;
        }
        
        archive >> skeleton;
        if (!skeleton.num_joints()) {
            std::cerr << "Skeleton has no joints" << std::endl;
            return 1;
        }
    }

    std::cout << "Loaded skeleton with " << skeleton.num_joints() << " joints" << std::endl;
    std::cout << "\nBind pose (local transforms):" << std::endl;
    std::cout << "=============================" << std::endl;

    // Print joint hierarchy and bind pose transforms
    auto joint_names = skeleton.joint_names();
    auto joint_parents = skeleton.joint_parents();
    auto bind_poses = skeleton.joint_rest_poses();
    
    for (int i = 0; i < skeleton.num_joints(); ++i) {
        // Get parent name
        const char* parent_name = (joint_parents[i] >= 0) ? joint_names[joint_parents[i]] : "none";
        
        std::cout << "\nJoint " << i << ": " << joint_names[i] 
                  << " (parent: " << parent_name << ")" << std::endl;
        
        // Access bind pose for this joint
        // Bind poses are stored in SoA format, 4 joints per SoaTransform
        int soa_index = i / 4;
        int soa_element = i % 4;
        
        const ozz::math::SoaTransform& soa_transform = bind_poses[soa_index];
        
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
        
        // Extract rotation quaternion
        float rx, ry, rz, rw;
        switch (soa_element) {
            case 0:
                rx = ozz::math::GetX(soa_transform.rotation.x);
                ry = ozz::math::GetX(soa_transform.rotation.y);
                rz = ozz::math::GetX(soa_transform.rotation.z);
                rw = ozz::math::GetX(soa_transform.rotation.w);
                break;
            case 1:
                rx = ozz::math::GetY(soa_transform.rotation.x);
                ry = ozz::math::GetY(soa_transform.rotation.y);
                rz = ozz::math::GetY(soa_transform.rotation.z);
                rw = ozz::math::GetY(soa_transform.rotation.w);
                break;
            case 2:
                rx = ozz::math::GetZ(soa_transform.rotation.x);
                ry = ozz::math::GetZ(soa_transform.rotation.y);
                rz = ozz::math::GetZ(soa_transform.rotation.z);
                rw = ozz::math::GetZ(soa_transform.rotation.w);
                break;
            case 3:
                rx = ozz::math::GetW(soa_transform.rotation.x);
                ry = ozz::math::GetW(soa_transform.rotation.y);
                rz = ozz::math::GetW(soa_transform.rotation.z);
                rw = ozz::math::GetW(soa_transform.rotation.w);
                break;
        }
        
        // Extract scale
        float sx, sy, sz;
        switch (soa_element) {
            case 0:
                sx = ozz::math::GetX(soa_transform.scale.x);
                sy = ozz::math::GetX(soa_transform.scale.y);
                sz = ozz::math::GetX(soa_transform.scale.z);
                break;
            case 1:
                sx = ozz::math::GetY(soa_transform.scale.x);
                sy = ozz::math::GetY(soa_transform.scale.y);
                sz = ozz::math::GetY(soa_transform.scale.z);
                break;
            case 2:
                sx = ozz::math::GetZ(soa_transform.scale.x);
                sy = ozz::math::GetZ(soa_transform.scale.y);
                sz = ozz::math::GetZ(soa_transform.scale.z);
                break;
            case 3:
                sx = ozz::math::GetW(soa_transform.scale.x);
                sy = ozz::math::GetW(soa_transform.scale.y);
                sz = ozz::math::GetW(soa_transform.scale.z);
                break;
        }
        
        std::cout << "  Translation: (" << tx << ", " << ty << ", " << tz << ")" << std::endl;
        std::cout << "  Rotation:    (" << rx << ", " << ry << ", " << rz << ", " << rw << ")" << std::endl;
        std::cout << "  Scale:       (" << sx << ", " << sy << ", " << sz << ")" << std::endl;
    }
    
    // Now compute and display world transforms from bind pose
    std::cout << "\n\nWorld space transforms (bind pose):" << std::endl;
    std::cout << "===================================" << std::endl;
    
    // Allocate buffers for LocalToModelJob
    ozz::vector<ozz::math::SoaTransform> local_transforms;
    local_transforms.resize(skeleton.num_soa_joints());
    
    // Copy bind poses to local transforms
    for (int i = 0; i < skeleton.num_soa_joints(); ++i) {
        local_transforms[i] = bind_poses[i];
    }
    
    // Allocate output world transforms
    ozz::vector<ozz::math::Float4x4> world_transforms;
    world_transforms.resize(skeleton.num_joints());
    
    // Run LocalToModelJob to compute world transforms
    ozz::animation::LocalToModelJob ltm_job;
    ltm_job.skeleton = &skeleton;
    ltm_job.input = ozz::make_span(local_transforms);
    ltm_job.output = ozz::make_span(world_transforms);
    
    if (!ltm_job.Run()) {
        std::cerr << "Failed to compute world transforms" << std::endl;
        return 1;
    }
    
    // Print world transforms
    for (int i = 0; i < skeleton.num_joints(); ++i) {
        std::cout << "\nJoint " << i << ": " << joint_names[i] << " (world space)" << std::endl;
        
        const ozz::math::Float4x4& transform = world_transforms[i];
        
        // Extract translation from 4th column
        float tx = ozz::math::GetX(transform.cols[3]);
        float ty = ozz::math::GetY(transform.cols[3]);
        float tz = ozz::math::GetZ(transform.cols[3]);
        
        std::cout << "  World position: (" << tx << ", " << ty << ", " << tz << ")" << std::endl;
    }
    
    std::cout << "\n\nBind pose display complete!" << std::endl;
    return 0;
}