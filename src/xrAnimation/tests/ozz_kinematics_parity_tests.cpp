// clang-format off
#include "Common/Platform.hpp"
#include "xrCore/xrCore.h"
#include "xrCore/FMesh.hpp"
#include "xrCore/Animation/Bone.hpp"
// clang-format on

#include "gtest/gtest.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <stdexcept>
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

struct AnimationSample
{
    std::unordered_map<std::string, Fmatrix> world_space_transforms;
};

struct LegacyChunk
{
    const std::byte* data = nullptr;
    size_t size = 0;
};

struct LegacyBinaryReader
{
    const std::byte* data = nullptr;
    size_t size = 0;
    size_t offset = 0;

    template <class T>
    T Read()
    {
        if (offset + sizeof(T) > size)
            throw std::runtime_error("unexpected end of chunk while reading typed data");

        T value{};
        std::memcpy(&value, data + offset, sizeof(T));
        offset += sizeof(T);
        return value;
    }

    template <class T>
    T ReadStruct()
    {
        return Read<T>();
    }

    std::string ReadStringZ()
    {
        const auto* begin = data + offset;
        const auto* end = data + size;
        const auto* cursor = begin;
        while (cursor < end && *reinterpret_cast<const char*>(cursor) != '\0')
            ++cursor;

        if (cursor == end)
            throw std::runtime_error("unterminated string in chunk");

        std::string value(reinterpret_cast<const char*>(begin), static_cast<size_t>(cursor - begin));
        offset += static_cast<size_t>(cursor - begin) + 1;
        return value;
    }

    Fvector ReadFvector3()
    {
        Fvector v{};
        v.x = Read<float>();
        v.y = Read<float>();
        v.z = Read<float>();
        return v;
    }

    void Skip(size_t count)
    {
        if (offset + count > size)
            throw std::runtime_error("attempted to skip past end of chunk");
        offset += count;
    }
};

using LegacyChunkMap = std::unordered_map<u32, LegacyChunk>;

std::vector<std::byte> LoadBinaryFile(const std::filesystem::path& path)
{
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream)
        throw std::runtime_error("failed to open input file: " + path.string());

    const auto size = stream.tellg();
    if (size <= 0)
        throw std::runtime_error("input file is empty: " + path.string());

    std::vector<std::byte> data(static_cast<size_t>(size));
    stream.seekg(0, std::ios::beg);
    stream.read(reinterpret_cast<char*>(data.data()), data.size());
    if (!stream)
        throw std::runtime_error("failed to read input file: " + path.string());

    return data;
}

LegacyChunkMap ParseChunks(const std::byte* data, size_t size)
{
    LegacyChunkMap chunks;

    size_t offset = 0;
    while (offset + sizeof(u32) * 2 <= size)
    {
        u32 id = 0;
        u32 chunk_size = 0;
        std::memcpy(&id, data + offset, sizeof(u32));
        offset += sizeof(u32);
        std::memcpy(&chunk_size, data + offset, sizeof(u32));
        offset += sizeof(u32);

        if (offset + chunk_size > size)
            throw std::runtime_error("chunk extends past end of file");

        chunks[id] = LegacyChunk{ data + offset, chunk_size };
        offset += chunk_size;
    }

    return chunks;
}

struct LegacyBoneRecord
{
    std::string name;
    std::string parent_name;
    int parent_index = -1;
    Fvector rest_translation{};
    Fvector rest_rotation{};
    Fmatrix local_transform{};
    Fmatrix global_transform{};
};

std::vector<LegacyBoneRecord> ReadBoneNames(const LegacyChunk& chunk)
{
    LegacyBinaryReader reader{ chunk.data, chunk.size, 0 };
    const u32 bone_count = reader.Read<u32>();

    std::vector<LegacyBoneRecord> bones;
    bones.reserve(bone_count);

    for (u32 idx = 0; idx < bone_count; ++idx)
    {
        LegacyBoneRecord record;
        record.name = reader.ReadStringZ();
        record.parent_name = reader.ReadStringZ();
        reader.Skip(sizeof(Fobb));
        bones.emplace_back(std::move(record));
    }

    return bones;
}

void ReadIkData(const LegacyChunk& chunk, std::vector<LegacyBoneRecord>& bones)
{
    LegacyBinaryReader reader{ chunk.data, chunk.size, 0 };

    for (auto& bone : bones)
    {
        const u32 version = reader.Read<u32>();
        reader.ReadStringZ();
        (void)reader.ReadStruct<SBoneShape>();

        reader.Read<u32>();
        for (int axis = 0; axis < 3; ++axis)
        {
            reader.Read<float>();
            reader.Read<float>();
            reader.Read<float>();
            reader.Read<float>();
        }

        reader.Read<float>();
        reader.Read<float>();
        reader.Read<u32>();
        reader.Read<float>();
        reader.Read<float>();
        if (version > 0)
            reader.Read<float>();

        bone.rest_rotation = reader.ReadFvector3();
        bone.rest_translation = reader.ReadFvector3();
        reader.Read<float>();
        reader.ReadFvector3();
    }
}

void ResolveHierarchy(std::vector<LegacyBoneRecord>& bones)
{
    std::unordered_map<std::string, int> index_by_name;
    index_by_name.reserve(bones.size());

    for (size_t idx = 0; idx < bones.size(); ++idx)
        index_by_name[bones[idx].name] = static_cast<int>(idx);

    for (auto& bone : bones)
    {
        if (bone.parent_name.empty())
        {
            bone.parent_index = -1;
            continue;
        }

        const auto it = index_by_name.find(bone.parent_name);
        if (it == index_by_name.end())
            throw std::runtime_error("bone parent not found: " + bone.parent_name + " for bone " + bone.name);
        bone.parent_index = it->second;
    }
}

void BuildLocalTransforms(std::vector<LegacyBoneRecord>& bones)
{
    for (auto& bone : bones)
    {
        Fmatrix matrix;
        matrix.identity();
        matrix.setXYZi(bone.rest_rotation);
        matrix.translate_over(bone.rest_translation);
        bone.local_transform = matrix;
        bone.global_transform.identity();
    }
}

void BuildGlobalTransforms(std::vector<LegacyBoneRecord>& bones)
{
    std::vector<std::vector<int>> children(bones.size());
    for (size_t idx = 0; idx < bones.size(); ++idx)
        if (bones[idx].parent_index >= 0)
            children[static_cast<size_t>(bones[idx].parent_index)].push_back(static_cast<int>(idx));

    std::function<void(int, const Fmatrix&)> visit;
    visit = [&](int index, const Fmatrix& parent_matrix)
    {
        auto& bone = bones[static_cast<size_t>(index)];
        bone.global_transform.mul_43(parent_matrix, bone.local_transform);
        for (int child : children[static_cast<size_t>(index)])
            visit(child, bone.global_transform);
    };

    Fmatrix identity;
    identity.identity();

    for (size_t idx = 0; idx < bones.size(); ++idx)
        if (bones[idx].parent_index < 0)
            visit(static_cast<int>(idx), identity);
}

std::optional<BindPoseSample> LoadLegacyBindPoseSample(const std::filesystem::path& ogf_path, const std::filesystem::path& omf_path)
{
    (void)omf_path;

    try
    {
        const auto data = LoadBinaryFile(ogf_path);
        const auto chunks = ParseChunks(data.data(), data.size());

        const auto bone_names_it = chunks.find(OGF_S_BONE_NAMES);
        if (bone_names_it == chunks.end())
            throw std::runtime_error("OGF missing bone names chunk: " + ogf_path.string());

        const auto ik_it = chunks.find(OGF_S_IKDATA);
        if (ik_it == chunks.end())
            throw std::runtime_error("OGF missing IK data chunk: " + ogf_path.string());

        auto bones = ReadBoneNames(bone_names_it->second);
        ReadIkData(ik_it->second, bones);
        ResolveHierarchy(bones);
        BuildLocalTransforms(bones);
        BuildGlobalTransforms(bones);

        BindPoseSample sample;
        sample.world_space_transforms.reserve(bones.size());
        for (const auto& bone : bones)
            sample.world_space_transforms.emplace(bone.name, bone.global_transform);

        return sample;
    }
    catch (const std::exception& ex)
    {
        ADD_FAILURE() << "Legacy bind-pose load failed: " << ex.what();
        return std::nullopt;
    }
}

std::optional<AnimationSample> LoadLegacyAnimationSample(const std::filesystem::path& ogf_path, const std::filesystem::path& omf_path,
    std::string_view motion_name, float sample_time_seconds)
{
    (void)ogf_path;
    (void)omf_path;
    (void)motion_name;
    (void)sample_time_seconds;

    // TODO: Drive CKinematics sampling against legacy `.ogf/.omf` data for animation parity.
    return std::nullopt;
}

std::optional<AnimationSample> LoadOzzAnimationSample(const std::filesystem::path& skeleton_path, const std::filesystem::path& animation_path,
    std::string_view motion_name, float sample_time_seconds)
{
    (void)skeleton_path;
    (void)animation_path;
    (void)motion_name;
    (void)sample_time_seconds;

    // TODO: Drive Ozz sampling (SamplingJob + LocalToModelJob) for converted animation data.
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

TEST(OzzKinematicsPose, MatchesLegacyBindPoseTranslations)
{
    const auto skeleton_path = ResolveProjectPath("src/xrAnimation/tests/testdata/stalker_hero_1.ozz");
    const auto ogf_path = ResolveProjectPath("res/testdata/npc/stalker_hero_1.ogf");
    const auto omf_path = ResolveProjectPath("res/testdata/npc/critical_hit_grup_1.omf");

    ASSERT_TRUE(std::filesystem::exists(skeleton_path));
    ASSERT_TRUE(std::filesystem::exists(ogf_path));
    ASSERT_TRUE(std::filesystem::exists(omf_path));

    const auto legacy_sample = LoadLegacyBindPoseSample(ogf_path, omf_path);
    ASSERT_TRUE(legacy_sample.has_value());

    OzzKinematics kinematics;
    ASSERT_TRUE(kinematics.InitializeFromOzz(skeleton_path.string().c_str()));
    kinematics.CalculateBones(TRUE);

    ASSERT_EQ(legacy_sample->world_space_transforms.size(), static_cast<size_t>(kinematics.LL_BoneCount()));

    const std::vector<std::string> sentinel_bones = { "bip01", "bip01_pelvis", "bip01_spine", "bip01_head", "bip01_l_hand", "bip01_r_hand" };

    for (const auto& bone_name : sentinel_bones)
    {
        const auto legacy_it = legacy_sample->world_space_transforms.find(bone_name);
        ASSERT_NE(legacy_it, legacy_sample->world_space_transforms.end()) << "Legacy skeleton missing " << bone_name;

        const u16 bone_id = kinematics.LL_BoneID(bone_name.c_str());
        ASSERT_NE(bone_id, BI_NONE) << "OzzKinematics missing bone: " << bone_name;

        const Fmatrix& transform = kinematics.LL_GetTransform(bone_id);
        const Fmatrix& expected = legacy_it->second;

        EXPECT_NEAR(transform.c.x, expected.c.x, 1e-4f) << "Bone: " << bone_name;
        EXPECT_NEAR(transform.c.y, expected.c.y, 1e-4f) << "Bone: " << bone_name;
        EXPECT_NEAR(transform.c.z, expected.c.z, 1e-4f) << "Bone: " << bone_name;
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

TEST(OzzKinematicsParity, AnimationPoseMatchesLegacySkeleton)
{
    const auto ogf_path = ResolveProjectPath("res/testdata/npc/stalker_hero_1.ogf");
    const auto omf_path = ResolveProjectPath("res/testdata/npc/critical_hit_grup_1.omf");
    const auto ozz_skeleton_path = ResolveProjectPath("src/xrAnimation/tests/testdata/stalker_hero_1.ozz");
    const auto ozz_animation_path = ResolveProjectPath("src/xrAnimation/tests/testdata/critical_hit_grup_1.ozz");

    ASSERT_TRUE(std::filesystem::exists(ogf_path)) << "Missing legacy .ogf: " << ogf_path;
    ASSERT_TRUE(std::filesystem::exists(omf_path)) << "Missing legacy .omf: " << omf_path;
    ASSERT_TRUE(std::filesystem::exists(ozz_skeleton_path)) << "Missing converted skeleton: " << ozz_skeleton_path;
    ASSERT_TRUE(std::filesystem::exists(ozz_animation_path)) << "Missing converted animation: " << ozz_animation_path;

    constexpr std::string_view kMotionName = "critical_hit_grup_1";
    constexpr float kSampleTimeSeconds = 0.2f;

    const auto legacy_sample = LoadLegacyAnimationSample(ogf_path, omf_path, kMotionName, kSampleTimeSeconds);
    ASSERT_TRUE(legacy_sample.has_value()) << "Legacy animation sampling not implemented yet for " << kMotionName;

    const auto ozz_sample = LoadOzzAnimationSample(ozz_skeleton_path, ozz_animation_path, kMotionName, kSampleTimeSeconds);
    ASSERT_TRUE(ozz_sample.has_value()) << "Ozz animation sampling not implemented yet for " << kMotionName;

    ASSERT_EQ(legacy_sample->world_space_transforms.size(), ozz_sample->world_space_transforms.size()) << "Bone count mismatch between sampled poses";

    const std::vector<std::string> sentinel_bones = { "bip01", "bip01_spine", "bip01_head", "bip01_l_hand", "bip01_r_hand" };

    for (const auto& bone_name : sentinel_bones)
    {
        const auto legacy_it = legacy_sample->world_space_transforms.find(bone_name);
        ASSERT_NE(legacy_it, legacy_sample->world_space_transforms.end()) << "Legacy sample missing bone " << bone_name;

        const auto ozz_it = ozz_sample->world_space_transforms.find(bone_name);
        ASSERT_NE(ozz_it, ozz_sample->world_space_transforms.end()) << "Ozz sample missing bone " << bone_name;

        const Fmatrix& legacy_transform = legacy_it->second;
        const Fmatrix& ozz_transform = ozz_it->second;

        EXPECT_NEAR(legacy_transform.c.x, ozz_transform.c.x, 1e-4f) << "Animation X mismatch for bone " << bone_name;
        EXPECT_NEAR(legacy_transform.c.y, ozz_transform.c.y, 1e-4f) << "Animation Y mismatch for bone " << bone_name;
        EXPECT_NEAR(legacy_transform.c.z, ozz_transform.c.z, 1e-4f) << "Animation Z mismatch for bone " << bone_name;
    }
}
