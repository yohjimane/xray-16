// clang-format off
#include "Common/Platform.hpp"
#include "xrCore/xrCore.h"
#include "xrCore/FMesh.hpp"
#include "xrCore/Animation/Bone.hpp"
// clang-format on

#include "gtest/gtest.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "OzzKinematics.h"

#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/local_to_model_job.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/span.h"

#include "xrCore/Animation/SkeletonMotionDefs.hpp"
#include "xrCore/Animation/SkeletonMotions.hpp"

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
    float applied_time_seconds = 0.f;
};

std::string ToLowerCopy(std::string_view value)
{
    std::string lower(value.begin(), value.end());
    std::transform(lower.begin(), lower.end(), lower.begin(),
        [](unsigned char ch)
        {
            return static_cast<char>(std::tolower(ch));
        });
    return lower;
}

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

std::string ReadStringCrlf(LegacyBinaryReader& reader)
{
    std::string value;
    while (reader.offset < reader.size)
    {
        const char ch = static_cast<char>(reader.Read<uint8_t>());
        if (ch == '\r')
        {
            if (reader.offset >= reader.size)
                throw std::runtime_error("unexpected end of data while reading CRLF string");
            const char lf = static_cast<char>(reader.Read<uint8_t>());
            if (lf != '\n')
                throw std::runtime_error("expected LF after CR");
            break;
        }
        value.push_back(ch);
    }
    return value;
}

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

std::vector<std::pair<u32, LegacyChunk>> ParseSubchunks(const LegacyChunk& chunk)
{
    std::vector<std::pair<u32, LegacyChunk>> subchunks;

    size_t offset = 0;
    while (offset + sizeof(u32) * 2 <= chunk.size)
    {
        u32 id = 0;
        u32 sub_size = 0;
        std::memcpy(&id, chunk.data + offset, sizeof(u32));
        offset += sizeof(u32);
        std::memcpy(&sub_size, chunk.data + offset, sizeof(u32));
        offset += sizeof(u32);

        if (offset + sub_size > chunk.size)
            throw std::runtime_error("sub-chunk extends past parent chunk");

        subchunks.emplace_back(id, LegacyChunk{ chunk.data + offset, sub_size });
        offset += sub_size;
    }

    return subchunks;
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

struct LegacyAnimationTrack
{
    std::vector<Fquaternion> rotations;
    std::vector<Fvector> translations;
};

struct LegacyMotion
{
    std::string name;
    u32 frame_count = 0;
    std::vector<LegacyAnimationTrack> tracks;
};

struct LegacyAnimationData
{
    std::vector<uint16_t> bone_remap;
    std::vector<std::string> remap_bone_names;
    std::vector<LegacyMotion> motions;
    std::vector<std::string> motion_names;
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

std::vector<Fmatrix> BuildWorldTransformsFromLocals(const std::vector<LegacyBoneRecord>& bones, const std::vector<Fmatrix>& locals)
{
    std::vector<std::vector<int>> children(bones.size());
    for (size_t idx = 0; idx < bones.size(); ++idx)
        if (bones[idx].parent_index >= 0)
            children[static_cast<size_t>(bones[idx].parent_index)].push_back(static_cast<int>(idx));

    std::vector<Fmatrix> world(bones.size());

    std::function<void(int, const Fmatrix&)> visit;
    visit = [&](int index, const Fmatrix& parent_matrix)
    {
        const size_t bone_index = static_cast<size_t>(index);
        world[bone_index].mul_43(parent_matrix, locals[bone_index]);
        for (int child : children[bone_index])
            visit(child, world[bone_index]);
    };

    Fmatrix identity;
    identity.identity();

    for (size_t idx = 0; idx < bones.size(); ++idx)
        if (bones[idx].parent_index < 0)
            visit(static_cast<int>(idx), identity);

    return world;
}

void ParseLegacySmparams(const LegacyChunk& chunk, const std::vector<LegacyBoneRecord>& skeleton_bones, LegacyAnimationData& output)
{
    LegacyBinaryReader reader{ chunk.data, chunk.size, 0 };

    const u16 version = reader.Read<u16>();
    if (version > xrOGF_SMParamsVersion)
        throw std::runtime_error("unsupported OMF params version");

    const u16 part_count = reader.Read<u16>();

    output.bone_remap.assign(skeleton_bones.size(), std::numeric_limits<uint16_t>::max());
    output.remap_bone_names.assign(skeleton_bones.size(), std::string());

    std::unordered_map<std::string, uint16_t> skeleton_index_by_name;
    skeleton_index_by_name.reserve(skeleton_bones.size());
    for (uint16_t idx = 0; idx < skeleton_bones.size(); ++idx)
        skeleton_index_by_name.emplace(ToLowerCopy(skeleton_bones[idx].name), idx);

    uint32_t mapped = 0;

    for (u16 part = 0; part < part_count; ++part)
    {
        reader.ReadStringZ();
        const u16 bone_count = reader.Read<u16>();
        for (u16 bone_idx = 0; bone_idx < bone_count; ++bone_idx)
        {
            const std::string bone_name = reader.ReadStringZ();
            const uint32_t remap_index = reader.Read<u32>();

            if (remap_index >= output.bone_remap.size())
                throw std::runtime_error("bone remap index out of range");

            const auto it = skeleton_index_by_name.find(ToLowerCopy(bone_name));
            if (it == skeleton_index_by_name.end())
                throw std::runtime_error("bone " + bone_name + " not found in skeleton remap");

            output.bone_remap[remap_index] = it->second;
            output.remap_bone_names[remap_index] = bone_name;
            ++mapped;
        }
    }

    if (mapped != skeleton_bones.size())
        throw std::runtime_error("OMF bone remap does not cover entire skeleton");

    const u16 motion_count = reader.Read<u16>();
    output.motion_names.clear();
    output.motion_names.reserve(motion_count);

    for (u16 motion_idx = 0; motion_idx < motion_count; ++motion_idx)
    {
        std::string motion_name = reader.ReadStringZ();
        output.motion_names.emplace_back(motion_name);

        reader.Read<u32>();   // flags
        reader.Read<u16>();   // bone_or_part
        reader.Read<u16>();   // motion_id
        reader.Read<float>(); // speed
        reader.Read<float>(); // power
        reader.Read<float>(); // accrue
        reader.Read<float>(); // falloff

        if (version >= 4)
        {
            const u32 mark_count = reader.Read<u32>();
            for (u32 mark_idx = 0; mark_idx < mark_count; ++mark_idx)
            {
                ReadStringCrlf(reader);
                const u32 interval_count = reader.Read<u32>();
                for (u32 interval_idx = 0; interval_idx < interval_count; ++interval_idx)
                {
                    reader.Read<float>();
                    reader.Read<float>();
                }
            }
        }
    }
}

void ParseLegacyMotions(const LegacyChunk& chunk, const LegacyAnimationData& params, LegacyAnimationData& output)
{
    const auto subchunks = ParseSubchunks(chunk);
    if (subchunks.empty())
        throw std::runtime_error("OMF motion chunk missing count");

    const auto& count_chunk = subchunks.front();
    if (count_chunk.first != 0)
        throw std::runtime_error("OMF motion chunk missing count sub-chunk");

    LegacyBinaryReader count_reader{ count_chunk.second.data, count_chunk.second.size, 0 };
    const u32 motion_count = count_reader.Read<u32>();
    if (motion_count != params.motion_names.size())
        throw std::runtime_error("motion metadata count mismatch");

    output.motions.clear();
    output.motions.reserve(motion_count);

    if (subchunks.size() - 1 != motion_count)
        throw std::runtime_error("motion chunk count mismatch");

    for (u32 motion_idx = 0; motion_idx < motion_count; ++motion_idx)
    {
        const auto& item = subchunks[motion_idx + 1];
        LegacyBinaryReader reader{ item.second.data, item.second.size, 0 };

        LegacyMotion motion;
        motion.name = reader.ReadStringZ();
        motion.frame_count = reader.Read<u32>();
        if (motion.frame_count == 0)
            throw std::runtime_error("motion has zero frames: " + motion.name);

        motion.tracks.resize(params.bone_remap.size());

        for (size_t track_idx = 0; track_idx < params.bone_remap.size(); ++track_idx)
        {
            LegacyAnimationTrack track;
            track.rotations.resize(motion.frame_count);
            track.translations.resize(motion.frame_count);

            const uint8_t flags = reader.Read<uint8_t>();
            const bool rotation_present = (flags & flRKeyAbsent) == 0;
            const bool translation_present = (flags & flTKeyPresent) != 0;
            const bool high_quality_translation = (flags & flTKey16IsBit) != 0;

            auto read_quaternion = [&reader]()
            {
                const int16_t x = reader.Read<int16_t>();
                const int16_t y = reader.Read<int16_t>();
                const int16_t z = reader.Read<int16_t>();
                const int16_t w = reader.Read<int16_t>();
                Fquaternion q;
                q.x = static_cast<float>(x) * KEY_QuantI;
                q.y = static_cast<float>(y) * KEY_QuantI;
                q.z = static_cast<float>(z) * KEY_QuantI;
                q.w = static_cast<float>(w) * KEY_QuantI;
                q.normalize();
                return q;
            };

            if (rotation_present)
            {
                reader.Read<u32>();
                for (u32 frame = 0; frame < motion.frame_count; ++frame)
                    track.rotations[frame] = read_quaternion();
            }
            else
            {
                const Fquaternion q = read_quaternion();
                std::fill(track.rotations.begin(), track.rotations.end(), q);
            }

            if (translation_present)
            {
                reader.Read<u32>();

                if (high_quality_translation)
                {
                    std::vector<std::array<int16_t, 3>> samples(motion.frame_count);
                    for (u32 frame = 0; frame < motion.frame_count; ++frame)
                    {
                        samples[frame][0] = reader.Read<int16_t>();
                        samples[frame][1] = reader.Read<int16_t>();
                        samples[frame][2] = reader.Read<int16_t>();
                    }

                    const Fvector size = reader.ReadFvector3();
                    const Fvector init = reader.ReadFvector3();

                    for (u32 frame = 0; frame < motion.frame_count; ++frame)
                    {
                        Fvector t;
                        t.x = static_cast<float>(samples[frame][0]) * size.x + init.x;
                        t.y = static_cast<float>(samples[frame][1]) * size.y + init.y;
                        t.z = static_cast<float>(samples[frame][2]) * size.z + init.z;
                        track.translations[frame] = t;
                    }
                }
                else
                {
                    std::vector<std::array<int8_t, 3>> samples(motion.frame_count);
                    for (u32 frame = 0; frame < motion.frame_count; ++frame)
                    {
                        samples[frame][0] = reader.Read<int8_t>();
                        samples[frame][1] = reader.Read<int8_t>();
                        samples[frame][2] = reader.Read<int8_t>();
                    }

                    const Fvector size = reader.ReadFvector3();
                    const Fvector init = reader.ReadFvector3();

                    for (u32 frame = 0; frame < motion.frame_count; ++frame)
                    {
                        Fvector t;
                        t.x = static_cast<float>(samples[frame][0]) * size.x + init.x;
                        t.y = static_cast<float>(samples[frame][1]) * size.y + init.y;
                        t.z = static_cast<float>(samples[frame][2]) * size.z + init.z;
                        track.translations[frame] = t;
                    }
                }
            }
            else
            {
                const Fvector init = reader.ReadFvector3();
                std::fill(track.translations.begin(), track.translations.end(), init);
            }

            motion.tracks[track_idx] = std::move(track);
        }

        if (reader.offset != reader.size)
            throw std::runtime_error("unexpected data remaining in motion chunk");

        output.motions.emplace_back(std::move(motion));
    }
}

LegacyAnimationData ParseLegacyAnimationData(const LegacyChunkMap& chunks, const std::vector<LegacyBoneRecord>& bones)
{
    const auto params_it = chunks.find(OGF_S_SMPARAMS);
    if (params_it == chunks.end())
        throw std::runtime_error("OMF missing params chunk");

    const auto motions_it = chunks.find(OGF_S_MOTIONS);
    if (motions_it == chunks.end())
        throw std::runtime_error("OMF missing motions chunk");

    LegacyAnimationData data;
    ParseLegacySmparams(params_it->second, bones, data);
    ParseLegacyMotions(motions_it->second, data, data);
    return data;
}

std::string ReadOzzString(ozz::io::IArchive& archive)
{
    uint32_t length = 0;
    archive >> length;
    std::string value(length, '\0');
    if (length > 0)
        archive >> ozz::io::MakeArray(value.data(), length);
    return value;
}

std::string ReadOzzMotionMetadataName(ozz::io::IArchive& archive)
{
    std::string name = ReadOzzString(archive);

    uint32_t flags = 0;
    uint16_t bone_or_part = 0;
    uint16_t motion_id = 0;
    float speed = 0.f;
    float power = 0.f;
    float accrue = 0.f;
    float falloff = 0.f;

    archive >> flags;
    archive >> bone_or_part;
    archive >> motion_id;
    archive >> speed;
    archive >> power;
    archive >> accrue;
    archive >> falloff;

    uint32_t mark_count = 0;
    archive >> mark_count;
    for (uint32_t mark_idx = 0; mark_idx < mark_count; ++mark_idx)
    {
        ReadOzzString(archive);
        uint32_t interval_count = 0;
        archive >> interval_count;
        for (uint32_t interval_idx = 0; interval_idx < interval_count; ++interval_idx)
        {
            float first = 0.f;
            float second = 0.f;
            archive >> first;
            archive >> second;
        }
    }

    (void)flags;
    (void)bone_or_part;
    (void)motion_id;
    (void)speed;
    (void)power;
    (void)accrue;
    (void)falloff;

    return name;
}

Fvector3 ConvertOzzVectorToXRay(float x, float y, float z)
{
    Fvector3 result;
    result.x = x;
    result.y = y;
    result.z = -z;
    return result;
}

Fmatrix ConvertOzzMatrixToXRay(const ozz::math::Float4x4& matrix)
{
    float column[4];

    Fmatrix out;
    out.identity();

    ozz::math::Store3PtrU(matrix.cols[0], column);
    out.i = ConvertOzzVectorToXRay(column[0], column[1], column[2]);

    ozz::math::Store3PtrU(matrix.cols[1], column);
    out.j = ConvertOzzVectorToXRay(column[0], column[1], column[2]);

    ozz::math::Store3PtrU(matrix.cols[2], column);
    out.k = ConvertOzzVectorToXRay(column[0], column[1], column[2]);

    ozz::math::Store3PtrU(matrix.cols[3], column);
    out.c = ConvertOzzVectorToXRay(column[0], column[1], column[2]);

    out._14_ = 0.0f;
    out._24_ = 0.0f;
    out._34_ = 0.0f;
    out._44_ = 1.0f;

    return out;
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
    try
    {
        const auto ogf_data = LoadBinaryFile(ogf_path);
        const auto ogf_chunks = ParseChunks(ogf_data.data(), ogf_data.size());

        const auto bone_names_it = ogf_chunks.find(OGF_S_BONE_NAMES);
        if (bone_names_it == ogf_chunks.end())
            throw std::runtime_error("OGF missing bone names chunk for animation sampling");

        const auto ik_it = ogf_chunks.find(OGF_S_IKDATA);
        if (ik_it == ogf_chunks.end())
            throw std::runtime_error("OGF missing IK data chunk for animation sampling");

        auto bones = ReadBoneNames(bone_names_it->second);
        ReadIkData(ik_it->second, bones);
        ResolveHierarchy(bones);
        BuildLocalTransforms(bones);

        const auto omf_data = LoadBinaryFile(omf_path);
        const auto omf_chunks = ParseChunks(omf_data.data(), omf_data.size());
        LegacyAnimationData animation_data = ParseLegacyAnimationData(omf_chunks, bones);

        const std::string target = ToLowerCopy(motion_name);
        const LegacyMotion* motion = nullptr;
        for (const auto& candidate : animation_data.motions)
        {
            if (ToLowerCopy(candidate.name) == target)
            {
                motion = &candidate;
                break;
            }
        }

        if (!motion)
            throw std::runtime_error("motion not found in OMF: " + std::string(motion_name));

        if (motion->frame_count == 0)
            throw std::runtime_error("motion has zero frames: " + motion->name);

        const float clamped_time = std::max(0.0f, sample_time_seconds);
        const size_t frame_index = std::min(static_cast<size_t>(motion->frame_count - 1), static_cast<size_t>(clamped_time / SAMPLE_SPF));
        const float applied_time = static_cast<float>(frame_index) * SAMPLE_SPF;

        std::vector<Fmatrix> locals(bones.size());
        for (size_t idx = 0; idx < bones.size(); ++idx)
            locals[idx] = bones[idx].local_transform;

        for (size_t track_idx = 0; track_idx < animation_data.bone_remap.size(); ++track_idx)
        {
            const uint16_t bone_index = animation_data.bone_remap[track_idx];
            if (bone_index == std::numeric_limits<uint16_t>::max() || bone_index >= bones.size())
                continue;

            const LegacyAnimationTrack& track = motion->tracks[track_idx];
            if (frame_index >= track.rotations.size() || frame_index >= track.translations.size())
                continue;

            Fmatrix local;
            local.mk_xform(track.rotations[frame_index], track.translations[frame_index]);
            locals[bone_index] = local;
        }

        const std::vector<Fmatrix> world = BuildWorldTransformsFromLocals(bones, locals);

        AnimationSample sample;
        sample.world_space_transforms.reserve(bones.size());
        for (size_t idx = 0; idx < bones.size(); ++idx)
            sample.world_space_transforms.emplace(bones[idx].name, world[idx]);
        sample.applied_time_seconds = applied_time;

        return sample;
    }
    catch (const std::exception& ex)
    {
        ADD_FAILURE() << "Legacy animation sampling failed: " << ex.what();
        return std::nullopt;
    }
}

std::optional<AnimationSample> LoadOzzAnimationSample(const std::filesystem::path& skeleton_path, const std::filesystem::path& animation_path,
    std::string_view motion_name, float sample_time_seconds)
{
    try
    {
        ozz::animation::Skeleton skeleton;
        {
            ozz::io::File file(skeleton_path.string().c_str(), "rb");
            if (!file.opened())
                throw std::runtime_error("failed to open skeleton: " + skeleton_path.string());
            ozz::io::IArchive archive(&file);
            if (!archive.TestTag<ozz::animation::Skeleton>())
                throw std::runtime_error("file is not an ozz skeleton: " + skeleton_path.string());
            archive >> skeleton;
        }

        std::unique_ptr<ozz::animation::Animation> selected_animation;
        {
            ozz::io::File file(animation_path.string().c_str(), "rb");
            if (!file.opened())
                throw std::runtime_error("failed to open animation: " + animation_path.string());
            ozz::io::IArchive archive(&file);

            uint32_t animation_count = 0;
            archive >> animation_count;

            const std::string target = ToLowerCopy(motion_name);

            for (uint32_t index = 0; index < animation_count; ++index)
            {
                auto animation = std::make_unique<ozz::animation::Animation>();
                archive >> *animation;

                std::string metadata_name = ReadOzzMotionMetadataName(archive);
                if (!selected_animation && ToLowerCopy(metadata_name) == target)
                    selected_animation = std::move(animation);
            }
        }

        if (!selected_animation)
            throw std::runtime_error("animation motion not found: " + std::string(motion_name));

        const float duration = selected_animation->duration();
        const float clamped_time = std::max(0.0f, sample_time_seconds);

        size_t frame_count = 1;
        if (duration > 0.f)
        {
            frame_count = static_cast<size_t>(std::round(duration / SAMPLE_SPF)) + 1;
            frame_count = std::max<size_t>(frame_count, 1);
        }

        const size_t frame_index = frame_count > 1 ? std::min(frame_count - 1, static_cast<size_t>(clamped_time / SAMPLE_SPF)) : 0;
        const float applied_time = static_cast<float>(frame_index) * SAMPLE_SPF;
        const float ratio = frame_count > 1 ? static_cast<float>(frame_index) / static_cast<float>(frame_count - 1) : 0.f;

        std::vector<ozz::math::SoaTransform> locals(static_cast<size_t>(skeleton.num_soa_joints()));
        std::vector<ozz::math::Float4x4> models(static_cast<size_t>(skeleton.num_joints()));

        ozz::animation::SamplingJob::Context context;
        context.Resize(selected_animation->num_tracks());

        ozz::animation::SamplingJob sampling_job;
        sampling_job.animation = selected_animation.get();
        sampling_job.context = &context;
        sampling_job.ratio = ratio;
        sampling_job.output = ozz::span<ozz::math::SoaTransform>(locals.data(), locals.size());
        if (!sampling_job.Run())
            throw std::runtime_error("ozz sampling job failed");

        ozz::animation::LocalToModelJob ltm_job;
        ltm_job.skeleton = &skeleton;
        ltm_job.input = ozz::span<const ozz::math::SoaTransform>(locals.data(), locals.size());
        ltm_job.output = ozz::span<ozz::math::Float4x4>(models.data(), models.size());
        if (!ltm_job.Run())
            throw std::runtime_error("ozz local-to-model job failed");

        AnimationSample sample;
        const auto joint_names = skeleton.joint_names();
        for (int joint = 0; joint < skeleton.num_joints(); ++joint)
        {
            const size_t index = static_cast<size_t>(joint);
            const char* name = (index < joint_names.size()) ? joint_names[index] : nullptr;
            if (!name || !name[0])
                continue;

            sample.world_space_transforms.emplace(name, ConvertOzzMatrixToXRay(models[index]));
        }
        sample.applied_time_seconds = applied_time;

        return sample;
    }
    catch (const std::exception& ex)
    {
        ADD_FAILURE() << "Ozz animation sampling failed: " << ex.what();
        return std::nullopt;
    }
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

    constexpr std::string_view kMotionName = "norm_2_critical_hit_hend_left_0";
    constexpr float kSampleTimeSeconds = 0.2f;

    const auto legacy_sample = LoadLegacyAnimationSample(ogf_path, omf_path, kMotionName, kSampleTimeSeconds);
    ASSERT_TRUE(legacy_sample.has_value()) << "Legacy animation sampling not implemented yet for " << kMotionName;

    const auto ozz_sample = LoadOzzAnimationSample(ozz_skeleton_path, ozz_animation_path, kMotionName, legacy_sample->applied_time_seconds);
    ASSERT_TRUE(ozz_sample.has_value()) << "Ozz animation sampling not implemented yet for " << kMotionName;

    ASSERT_EQ(legacy_sample->world_space_transforms.size(), ozz_sample->world_space_transforms.size()) << "Bone count mismatch between sampled poses";

    const std::vector<std::string> sentinel_bones = { "bip01", "bip01_spine", "bip01_head", "bip01_l_hand", "bip01_r_hand" };
    constexpr float kAnimationTolerance = 5e-4f; // Conversion quantization + basis changes introduce ~2e-4 deltas.

    for (const auto& bone_name : sentinel_bones)
    {
        const auto legacy_it = legacy_sample->world_space_transforms.find(bone_name);
        ASSERT_NE(legacy_it, legacy_sample->world_space_transforms.end()) << "Legacy sample missing bone " << bone_name;

        const auto ozz_it = ozz_sample->world_space_transforms.find(bone_name);
        ASSERT_NE(ozz_it, ozz_sample->world_space_transforms.end()) << "Ozz sample missing bone " << bone_name;

        const Fmatrix& legacy_transform = legacy_it->second;
        const Fmatrix& ozz_transform = ozz_it->second;

        const float dx = legacy_transform.c.x - ozz_transform.c.x;
        const float dy = legacy_transform.c.y - ozz_transform.c.y;
        const float dz = legacy_transform.c.z - ozz_transform.c.z;
        const float max_abs = std::max({ std::fabs(dx), std::fabs(dy), std::fabs(dz) });
        SCOPED_TRACE(::testing::Message() << "bone=" << bone_name << " legacy_t=" << legacy_sample->applied_time_seconds << " diff(x,y,z)=(" << dx << ", " << dy
                                          << ", " << dz << ") max=" << max_abs);

        EXPECT_NEAR(legacy_transform.c.x, ozz_transform.c.x, kAnimationTolerance) << "Animation X mismatch for bone " << bone_name;
        EXPECT_NEAR(legacy_transform.c.y, ozz_transform.c.y, kAnimationTolerance) << "Animation Y mismatch for bone " << bone_name;
        EXPECT_NEAR(legacy_transform.c.z, ozz_transform.c.z, kAnimationTolerance) << "Animation Z mismatch for bone " << bone_name;
    }
}
