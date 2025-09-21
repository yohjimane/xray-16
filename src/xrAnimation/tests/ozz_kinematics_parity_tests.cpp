#include "Common/Platform.hpp"
#include "xrCore/xrCore.h"
#include "xrCore/Animation/Bone.hpp"

#include "gtest/gtest.h"

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "OzzKinematics.h"

#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/animation/runtime/skeleton.h"

using XRay::Animation::OzzKinematics;

namespace
{
std::filesystem::path ResolveProjectPath(const std::string& relative)
{
    const std::filesystem::path root(PROJECT_ROOT);
    return root / relative;
}
}

TEST(OzzKinematicsBootstrap, MatchesJointCountWithReferenceSkeleton)
{
    const std::filesystem::path skeleton_path =
        ResolveProjectPath("src/xrAnimation/tests/testdata/stalker_hero_1.ozz");
    ASSERT_TRUE(std::filesystem::exists(skeleton_path))
        << "Missing sample skeleton at " << skeleton_path;

    ozz::animation::Skeleton reference_skeleton;
    {
        ozz::io::File file(skeleton_path.string().c_str(), "rb");
        ASSERT_TRUE(file.opened()) << "Failed to open " << skeleton_path;
        ozz::io::IArchive archive(&file);
        archive >> reference_skeleton;
    }

    OzzKinematics kinematics;
    ASSERT_TRUE(kinematics.InitializeFromOzz(skeleton_path.string().c_str()))
        << "OzzKinematics failed to load skeleton";

    EXPECT_EQ(reference_skeleton.num_joints(), kinematics.LL_BoneCount())
        << "Loaded joint count mismatch";
}

TEST(OzzKinematicsBootstrap, BoneNameLookupsAndVisibilityDefaults)
{
    const std::filesystem::path skeleton_path =
        ResolveProjectPath("src/xrAnimation/tests/testdata/stalker_hero_1.ozz");
    ASSERT_TRUE(std::filesystem::exists(skeleton_path))
        << "Missing sample skeleton at " << skeleton_path;

    ozz::animation::Skeleton reference_skeleton;
    {
        ozz::io::File file(skeleton_path.string().c_str(), "rb");
        ASSERT_TRUE(file.opened()) << "Failed to open " << skeleton_path;
        ozz::io::IArchive archive(&file);
        archive >> reference_skeleton;
    }

    const int joint_count = reference_skeleton.num_joints();
    ASSERT_GT(joint_count, 0);

    OzzKinematics kinematics;
    ASSERT_TRUE(kinematics.InitializeFromOzz(skeleton_path.string().c_str()))
        << "OzzKinematics failed to load skeleton";

    EXPECT_EQ(joint_count, kinematics.LL_BoneCount());

    const auto joint_names = reference_skeleton.joint_names();
    for (int joint = 0; joint < joint_count; ++joint)
    {
        const char* joint_name =
            (static_cast<size_t>(joint) < joint_names.size()) ? joint_names[joint] : nullptr;

        const std::string_view name_view = joint_name ? std::string_view(joint_name) : std::string_view();

        if (!name_view.empty())
        {
            const u16 expected_id = static_cast<u16>(joint);
            EXPECT_EQ(expected_id, kinematics.LL_BoneID(joint_name))
                << "Name lookup mismatch for joint " << joint << " (" << joint_name << ")";

            const shared_str shared_name(joint_name);
            EXPECT_EQ(expected_id, kinematics.LL_BoneID(shared_name))
                << "shared_str lookup mismatch for joint " << joint << " (" << joint_name << ")";

            EXPECT_STREQ(joint_name, kinematics.LL_BoneName_dbg(expected_id))
                << "Debug name mismatch for bone " << joint;
        }
    }

    const u16 visible_limit = std::min<u16>(kinematics.LL_BoneCount(), 64);
    for (u16 bone = 0; bone < visible_limit; ++bone)
        EXPECT_TRUE(kinematics.LL_GetBoneVisible(bone)) << "Bone " << bone << " should be visible by default";

    if (kinematics.LL_BoneCount() == 0)
        EXPECT_EQ(0u, kinematics.LL_GetBonesVisible());
    else if (kinematics.LL_BoneCount() >= 64)
        EXPECT_EQ(u64(-1), kinematics.LL_GetBonesVisible());
    else
        EXPECT_EQ((u64(1) << kinematics.LL_BoneCount()) - 1, kinematics.LL_GetBonesVisible());
}

TEST(OzzKinematicsPose, MatchesBaselineBindPoseTranslations)
{
    const auto skeleton_path = ResolveProjectPath("src/xrAnimation/tests/testdata/stalker_hero_1.ozz");
    const auto baseline_path = ResolveProjectPath("src/xrAnimation/tests/testdata/stalker_hero_bind_pose.csv");

    ASSERT_TRUE(std::filesystem::exists(skeleton_path));
    ASSERT_TRUE(std::filesystem::exists(baseline_path));

    std::unordered_map<std::string, Fvector3> baseline_positions;
    std::ifstream baseline_file(baseline_path);
    ASSERT_TRUE(baseline_file.is_open());

    std::string line;
    std::getline(baseline_file, line); // header
    while (std::getline(baseline_file, line))
    {
        if (line.empty())
            continue;

        std::vector<std::string> tokens;
        size_t start = 0;
        while (start <= line.size())
        {
            size_t end = line.find(',', start);
            if (end == std::string::npos)
                end = line.size();
            tokens.emplace_back(line.substr(start, end - start));
            start = end + 1;
        }

        if (tokens.size() < 8)
            continue;

        Fvector3 global;
        global.set(std::stof(tokens[5]), std::stof(tokens[6]), std::stof(tokens[7]));
        baseline_positions.emplace(tokens[0], global);
    }

    ASSERT_FALSE(baseline_positions.empty());

    OzzKinematics kinematics;
    ASSERT_TRUE(kinematics.InitializeFromOzz(skeleton_path.string().c_str()));

    kinematics.CalculateBones(TRUE);

    for (const auto& [bone_name, expected_position] : baseline_positions)
    {
        const u16 bone_id = kinematics.LL_BoneID(bone_name.c_str());
        ASSERT_NE(bone_id, BI_NONE) << "Missing bone in OzzKinematics: " << bone_name;

        const Fmatrix& transform = kinematics.LL_GetTransform(bone_id);
        EXPECT_NEAR(transform.c.x, expected_position.x, 1e-3f) << "Bone: " << bone_name;
        EXPECT_NEAR(transform.c.y, expected_position.y, 1e-3f) << "Bone: " << bone_name;
        EXPECT_NEAR(transform.c.z, expected_position.z, 1e-3f) << "Bone: " << bone_name;
    }
}
#include <fstream>
#include <unordered_map>
