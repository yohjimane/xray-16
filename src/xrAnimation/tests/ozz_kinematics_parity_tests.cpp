#include "Common/Platform.hpp"
#include "xrCore/Animation/Bone.hpp"
#include "xrCore/xrCore.h"

#include "gtest/gtest.h"

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "OzzKinematics.h"

#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"

using XRay::Animation::OzzKinematics;

namespace
{
std::filesystem::path ResolveProjectPath(const std::string& relative)
{
    const std::filesystem::path root(PROJECT_ROOT);
    return root / relative;
}

struct BindPoseSample
{
    std::unordered_map<std::string, Fmatrix> world_space_transforms;
};

std::optional<BindPoseSample> LoadLegacyBindPoseSample(const std::filesystem::path& ogf_path, const std::filesystem::path& omf_path)
{
    (void)ogf_path;
    (void)omf_path;

    // TODO: Hook up CKinematics/IKinematics sampling for legacy `.ogf/.omf` assets.
    return std::nullopt;
}

std::optional<BindPoseSample> LoadOzzBindPoseSample(const std::filesystem::path& skeleton_path)
{
    if (!std::filesystem::exists(skeleton_path))
        return std::nullopt;

    OzzKinematics kinematics;
    if (!kinematics.InitializeFromOzz(skeleton_path.string().c_str()))
        return std::nullopt;

    kinematics.CalculateBones(TRUE);

    BindPoseSample sample;
    const u16 bone_count = kinematics.LL_BoneCount();
    sample.world_space_transforms.reserve(bone_count);

    for (u16 bone = 0; bone < bone_count; ++bone)
    {
        const char* bone_name = kinematics.LL_BoneName_dbg(bone);
        if (!bone_name || !bone_name[0])
            continue;

        Fmatrix transform = kinematics.LL_GetTransform(bone);
        sample.world_space_transforms.emplace(std::string(bone_name), transform);
    }

    return sample;
}
} // namespace

TEST(OzzKinematicsBootstrap, MatchesJointCountWithReferenceSkeleton)
{
    const std::filesystem::path skeleton_path = ResolveProjectPath("src/xrAnimation/tests/testdata/stalker_hero_1.ozz");
    ASSERT_TRUE(std::filesystem::exists(skeleton_path)) << "Missing sample skeleton at " << skeleton_path;

    ozz::animation::Skeleton reference_skeleton;
    {
        ozz::io::File file(skeleton_path.string().c_str(), "rb");
        ASSERT_TRUE(file.opened()) << "Failed to open " << skeleton_path;
        ozz::io::IArchive archive(&file);
        archive >> reference_skeleton;
    }

    OzzKinematics kinematics;
    ASSERT_TRUE(kinematics.InitializeFromOzz(skeleton_path.string().c_str())) << "OzzKinematics failed to load skeleton";

    EXPECT_EQ(reference_skeleton.num_joints(), kinematics.LL_BoneCount()) << "Loaded joint count mismatch";
}

TEST(OzzKinematicsBootstrap, BoneNameLookupsAndVisibilityDefaults)
{
    const std::filesystem::path skeleton_path = ResolveProjectPath("src/xrAnimation/tests/testdata/stalker_hero_1.ozz");
    ASSERT_TRUE(std::filesystem::exists(skeleton_path)) << "Missing sample skeleton at " << skeleton_path;

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
    ASSERT_TRUE(kinematics.InitializeFromOzz(skeleton_path.string().c_str())) << "OzzKinematics failed to load skeleton";

    EXPECT_EQ(joint_count, kinematics.LL_BoneCount());

    const auto joint_names = reference_skeleton.joint_names();
    for (int joint = 0; joint < joint_count; ++joint)
    {
        const char* joint_name = (static_cast<size_t>(joint) < joint_names.size()) ? joint_names[joint] : nullptr;

        const std::string_view name_view = joint_name ? std::string_view(joint_name) : std::string_view();

        if (!name_view.empty())
        {
            const u16 expected_id = static_cast<u16>(joint);
            EXPECT_EQ(expected_id, kinematics.LL_BoneID(joint_name)) << "Name lookup mismatch for joint " << joint << " (" << joint_name << ")";

            const shared_str shared_name(joint_name);
            EXPECT_EQ(expected_id, kinematics.LL_BoneID(shared_name)) << "shared_str lookup mismatch for joint " << joint << " (" << joint_name << ")";

            EXPECT_STREQ(joint_name, kinematics.LL_BoneName_dbg(expected_id)) << "Debug name mismatch for bone " << joint;
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

TEST(OzzKinematicsParity, BindPoseMatchesLegacySkeleton)
{
    const auto ogf_path = ResolveProjectPath("res/testdata/npc/stalker_hero_1.ogf");
    const auto omf_path = ResolveProjectPath("res/testdata/npc/critical_hit_grup_1.omf");
    const auto ozz_skeleton_path = ResolveProjectPath("src/xrAnimation/tests/testdata/stalker_hero_1.ozz");

    ASSERT_TRUE(std::filesystem::exists(ogf_path)) << "Missing legacy .ogf: " << ogf_path;
    ASSERT_TRUE(std::filesystem::exists(omf_path)) << "Missing legacy .omf: " << omf_path;
    ASSERT_TRUE(std::filesystem::exists(ozz_skeleton_path)) << "Missing converted skeleton: " << ozz_skeleton_path;

    const auto legacy_sample = LoadLegacyBindPoseSample(ogf_path, omf_path);
    ASSERT_TRUE(legacy_sample.has_value()) << "Legacy bind-pose sampling not implemented yet for " << ogf_path.filename();

    const auto ozz_sample = LoadOzzBindPoseSample(ozz_skeleton_path);
    ASSERT_TRUE(ozz_sample.has_value()) << "Failed to load Ozz bind-pose sample from " << ozz_skeleton_path;

    ASSERT_EQ(legacy_sample->world_space_transforms.size(), ozz_sample->world_space_transforms.size())
        << "Bone count mismatch between legacy and Ozz skeletons";

    for (const auto& [bone_name, legacy_transform] : legacy_sample->world_space_transforms)
    {
        const auto ozz_iter = ozz_sample->world_space_transforms.find(bone_name);
        ASSERT_NE(ozz_iter, ozz_sample->world_space_transforms.end()) << "Ozz skeleton missing bone " << bone_name;

        const Fmatrix& ozz_transform = ozz_iter->second;

        EXPECT_NEAR(legacy_transform.c.x, ozz_transform.c.x, 1e-4f) << "Bind-pose X mismatch for bone " << bone_name;
        EXPECT_NEAR(legacy_transform.c.y, ozz_transform.c.y, 1e-4f) << "Bind-pose Y mismatch for bone " << bone_name;
        EXPECT_NEAR(legacy_transform.c.z, ozz_transform.c.z, 1e-4f) << "Bind-pose Z mismatch for bone " << bone_name;
    }
}
