#include "stdafx.h"

#include "LegacyOmfConverter.h"

#include "xrCore/Animation/SkeletonMotionDefs.hpp"
#include "xrCore/Animation/SkeletonMotions.hpp"

#include <ozz/animation/offline/animation_builder.h>
#include <ozz/animation/offline/animation_optimizer.h>
#include <ozz/animation/offline/raw_animation.h>

#include <ozz/base/io/archive.h>
#include <ozz/base/maths/math_ex.h>
#include <ozz/base/maths/quaternion.h>
#include <ozz/base/maths/soa_transform.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace XRay
{
namespace Animation
{
namespace
{
using Matrix4 = std::array<std::array<float, 4>, 4>;

Matrix4 ToColumnMajor(const Fmatrix& matrix)
{
    Matrix4 result{};
    for (int row = 0; row < 4; ++row)
    {
        result[static_cast<size_t>(row)][0] = matrix.m[row][0];
        result[static_cast<size_t>(row)][1] = matrix.m[row][1];
        result[static_cast<size_t>(row)][2] = matrix.m[row][2];
        result[static_cast<size_t>(row)][3] = matrix.m[row][3];
    }
    return result;
}

Matrix4 Multiply(const Matrix4& lhs, const Matrix4& rhs)
{
    Matrix4 result{};
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            float value = 0.f;
            for (int k = 0; k < 4; ++k)
                value += lhs[static_cast<size_t>(row)][static_cast<size_t>(k)] * rhs[static_cast<size_t>(k)][static_cast<size_t>(col)];
            result[static_cast<size_t>(row)][static_cast<size_t>(col)] = value;
        }
    }
    return result;
}

Matrix4 ChangeBasis(const Matrix4& matrix, const Matrix4& basis, const Matrix4& basis_inverse)
{
    return Multiply(Multiply(basis, matrix), basis_inverse);
}

constexpr Matrix4 kXrayToOzz = {
    std::array<float, 4>{ 1.f, 0.f,  0.f, 0.f },
    std::array<float, 4>{ 0.f, 1.f,  0.f, 0.f },
    std::array<float, 4>{ 0.f, 0.f, -1.f, 0.f },
    std::array<float, 4>{ 0.f, 0.f,  0.f, 1.f }
};

constexpr Matrix4 kOzzToXray = kXrayToOzz;

Matrix4 ConvertXrayLocalToOzz(const Fmatrix& matrix)
{
    return ChangeBasis(ToColumnMajor(matrix), kXrayToOzz, kOzzToXray);
}

ozz::math::Float3 ExtractTranslation(const Matrix4& matrix)
{
    return { matrix[0][3], matrix[1][3], matrix[2][3] };
}

ozz::math::Quaternion ExtractQuaternion(const Matrix4& matrix)
{
    const float m00 = matrix[0][0];
    const float m01 = matrix[0][1];
    const float m02 = matrix[0][2];
    const float m10 = matrix[1][0];
    const float m11 = matrix[1][1];
    const float m12 = matrix[1][2];
    const float m20 = matrix[2][0];
    const float m21 = matrix[2][1];
    const float m22 = matrix[2][2];

    float qw = 0.f;
    float qx = 0.f;
    float qy = 0.f;
    float qz = 0.f;

    const float trace = m00 + m11 + m22;
    if (trace > 0.f)
    {
        const float s = std::sqrt(trace + 1.f) * 2.f;
        qw = 0.25f * s;
        qx = (m21 - m12) / s;
        qy = (m02 - m20) / s;
        qz = (m10 - m01) / s;
    }
    else if (m00 > m11 && m00 > m22)
    {
        const float s = std::sqrt(1.f + m00 - m11 - m22) * 2.f;
        qw = (m21 - m12) / s;
        qx = 0.25f * s;
        qy = (m01 + m10) / s;
        qz = (m02 + m20) / s;
    }
    else if (m11 > m22)
    {
        const float s = std::sqrt(1.f + m11 - m00 - m22) * 2.f;
        qw = (m02 - m20) / s;
        qx = (m01 + m10) / s;
        qy = 0.25f * s;
        qz = (m12 + m21) / s;
    }
    else
    {
        const float s = std::sqrt(1.f + m22 - m00 - m11) * 2.f;
        qw = (m10 - m01) / s;
        qx = (m02 + m20) / s;
        qy = (m12 + m21) / s;
        qz = 0.25f * s;
    }

    ozz::math::Quaternion quaternion;
    quaternion.x = qx;
    quaternion.y = qy;
    quaternion.z = qz;
    quaternion.w = qw;
    return quaternion;
}

struct BinaryReader
{
    const std::byte* data = nullptr;
    size_t size = 0;
    size_t offset = 0;

    template <class T>
    T Read()
    {
        if (offset + sizeof(T) > size)
            throw std::runtime_error("unexpected end of chunk while reading typed data");
        T value;
        std::memcpy(&value, data + offset, sizeof(T));
        offset += sizeof(T);
        return value;
    }

    int8_t ReadInt8()
    {
        if (offset + sizeof(int8_t) > size)
            throw std::runtime_error("unexpected end of chunk while reading int8");
        const int8_t value = *reinterpret_cast<const int8_t*>(data + offset);
        offset += sizeof(int8_t);
        return value;
    }

    uint8_t ReadUInt8()
    {
        if (offset + sizeof(uint8_t) > size)
            throw std::runtime_error("unexpected end of chunk while reading uint8");
        const uint8_t value = *reinterpret_cast<const uint8_t*>(data + offset);
        offset += sizeof(uint8_t);
        return value;
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
};

struct Chunk
{
    const std::byte* data = nullptr;
    size_t size = 0;
};

std::vector<std::byte> LoadFileBytes(const std::filesystem::path& path)
{
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream)
        throw std::runtime_error("failed to open file: " + path.string());

    const auto size = stream.tellg();
    if (size <= 0)
        throw std::runtime_error("input file is empty: " + path.string());

    std::vector<std::byte> data(static_cast<size_t>(size));
    stream.seekg(0, std::ios::beg);
    stream.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));
    if (!stream)
        throw std::runtime_error("failed to read file: " + path.string());

    return data;
}

std::unordered_map<u32, Chunk> ParseChunks(const std::byte* data, size_t size)
{
    std::unordered_map<u32, Chunk> chunks;
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
        chunks.emplace(id, Chunk{ data + offset, chunk_size });
        offset += chunk_size;
    }
    return chunks;
}

std::vector<std::pair<u32, Chunk>> ParseSubchunks(const Chunk& chunk)
{
    std::vector<std::pair<u32, Chunk>> subchunks;
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
        subchunks.emplace_back(id, Chunk{ chunk.data + offset, sub_size });
        offset += sub_size;
    }
    return subchunks;
}

std::string ToLowerCopy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char ch)
        {
            return static_cast<char>(std::tolower(ch));
        });
    return value;
}

std::string ReadStringCRLF(BinaryReader& reader)
{
    std::string value;
    while (reader.offset < reader.size)
    {
        const char ch = static_cast<char>(reader.ReadInt8());
        if (ch == '\r')
        {
            if (reader.offset >= reader.size)
                throw std::runtime_error("unexpected end of data while reading CRLF string");
            const char lf = static_cast<char>(reader.ReadInt8());
            if (lf != '\n')
                throw std::runtime_error("expected LF after CR in motion mark name");
            break;
        }
        value.push_back(ch);
    }
    return value;
}

void ParseSmparams(const Chunk& chunk, const xr_vector<xr_string>& skeleton_bone_names, LegacyOmfData& output)
{
    BinaryReader reader{ chunk.data, chunk.size };

    const u16 version = reader.Read<u16>();
    if (version > xrOGF_SMParamsVersion)
        throw std::runtime_error("unsupported OMF params version");

    const u16 part_count = reader.Read<u16>();

    output.bone_remap.assign(skeleton_bone_names.size(), std::numeric_limits<u16>::max());
    output.remap_bone_names.assign(skeleton_bone_names.size(), xr_string());

    xr_vector<xr_string> lower_names;
    lower_names.reserve(skeleton_bone_names.size());
    for (const xr_string& name : skeleton_bone_names)
        lower_names.emplace_back(ToLowerCopy(std::string(name.c_str())));

    std::unordered_map<std::string, u16> skeleton_index_by_name;
    for (u16 idx = 0; idx < lower_names.size(); ++idx)
        skeleton_index_by_name.emplace(lower_names[idx], idx);

    u32 bones_mapped = 0;

    for (u16 part = 0; part < part_count; ++part)
    {
        reader.ReadStringZ();
        const u16 bone_count = reader.Read<u16>();
        for (u16 bone_idx = 0; bone_idx < bone_count; ++bone_idx)
        {
            const xr_string bone_name_raw = reader.ReadStringZ().c_str();
            const u32 remap_index = reader.Read<u32>();
            if (remap_index >= output.bone_remap.size())
                throw std::runtime_error("bone remap index out of range in OMF params");
            const std::string lowered = ToLowerCopy(std::string(bone_name_raw.c_str()));
            const auto it = skeleton_index_by_name.find(lowered);
            if (it == skeleton_index_by_name.end())
            {
                std::string message = "bone ";
                message += bone_name_raw.c_str();
                message += " referenced in OMF params not found in skeleton";
                throw std::runtime_error(message);
            }
            output.bone_remap[remap_index] = it->second;
            output.remap_bone_names[remap_index] = bone_name_raw;
            ++bones_mapped;
        }
    }

    if (bones_mapped != skeleton_bone_names.size())
        throw std::runtime_error("OMF bone remap does not cover all skeleton bones");

    const u16 motion_count = reader.Read<u16>();
    output.metadata.clear();
    output.metadata.reserve(motion_count);

    for (u16 motion_idx = 0; motion_idx < motion_count; ++motion_idx)
    {
        LegacyMotionMetadata meta;
        meta.name = reader.ReadStringZ().c_str();
        meta.flags = reader.Read<u32>();
        meta.bone_or_part = reader.Read<u16>();
        meta.motion_id = reader.Read<u16>();
        meta.speed = reader.Read<float>();
        meta.power = reader.Read<float>();
        meta.accrue = reader.Read<float>();
        meta.falloff = reader.Read<float>();

        if (version >= 4)
        {
            const u32 mark_count = reader.Read<u32>();
            meta.marks.reserve(mark_count);
            for (u32 mark_idx = 0; mark_idx < mark_count; ++mark_idx)
            {
                LegacyMotionMark mark;
                mark.name = ReadStringCRLF(reader).c_str();
                const u32 interval_count = reader.Read<u32>();
                mark.intervals.reserve(interval_count);
                for (u32 interval_idx = 0; interval_idx < interval_count; ++interval_idx)
                {
                    LegacyMotionInterval interval;
                    interval.first = reader.Read<float>();
                    interval.second = reader.Read<float>();
                    mark.intervals.emplace_back(interval);
                }
                meta.marks.emplace_back(std::move(mark));
            }
        }

        output.metadata.emplace_back(std::move(meta));
    }
}

void ParseMotions(const Chunk& chunk, const LegacyOmfData& params, LegacyOmfData& output)
{
    const auto subchunks = ParseSubchunks(chunk);
    if (subchunks.empty())
        throw std::runtime_error("OMF motion chunk missing motion count");

    const auto& count_chunk = subchunks.front();
    if (count_chunk.first != 0)
        throw std::runtime_error("OMF motion chunk missing count sub-chunk");

    BinaryReader count_reader{ count_chunk.second.data, count_chunk.second.size };
    const u32 motion_count = count_reader.Read<u32>();
    if (motion_count != output.metadata.size())
        throw std::runtime_error("motion metadata count mismatch");

    output.motions.clear();
    output.motions.reserve(motion_count);

    std::vector<std::pair<u32, Chunk>> motion_chunks(subchunks.begin() + 1, subchunks.end());
    if (motion_chunks.size() != motion_count)
        throw std::runtime_error("OMF motion chunk count mismatch");

    for (u32 motion_idx = 0; motion_idx < motion_count; ++motion_idx)
    {
        const auto& item = motion_chunks[motion_idx];
        BinaryReader reader{ item.second.data, item.second.size };

        LegacyOmfMotion motion;
        motion.name = reader.ReadStringZ().c_str();
        motion.frame_count = reader.Read<u32>();
        if (motion.frame_count == 0)
            throw std::runtime_error("motion has zero frames: " + std::string(motion.name.c_str()));

        motion.metadata = output.metadata[motion_idx];
        motion.bone_tracks.resize(params.bone_remap.size());

        for (size_t track_idx = 0; track_idx < params.bone_remap.size(); ++track_idx)
        {
            LegacyBoneTrack track;
            track.rotations.resize(motion.frame_count);
            track.translations.resize(motion.frame_count);

            const uint8_t flags = reader.ReadUInt8();
            const bool rotation_present = (flags & flRKeyAbsent) == 0;
            const bool translation_present = (flags & flTKeyPresent) != 0;
            const bool high_quality_translation = (flags & flTKey16IsBit) != 0;

            auto read_quaternion = [&reader]() -> Fquaternion
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
                    xr_vector<std::array<int16_t, 3>> samples(motion.frame_count);
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
                    xr_vector<std::array<int8_t, 3>> samples(motion.frame_count);
                    for (u32 frame = 0; frame < motion.frame_count; ++frame)
                    {
                        samples[frame][0] = reader.ReadInt8();
                        samples[frame][1] = reader.ReadInt8();
                        samples[frame][2] = reader.ReadInt8();
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

            motion.bone_tracks[track_idx] = std::move(track);
        }

        if (reader.offset != reader.size)
            throw std::runtime_error("unexpected extra data in motion chunk");

        output.motions.emplace_back(std::move(motion));
    }
}

LegacyOmfData ParseOmfBuffer(const std::byte* data, size_t size, const xr_vector<xr_string>& skeleton_bone_names)
{
    const auto chunks = ParseChunks(data, size);

    const auto params_it = chunks.find(OGF_S_SMPARAMS);
    if (params_it == chunks.end())
        throw std::runtime_error("OMF file missing smparams chunk");

    const auto motions_it = chunks.find(OGF_S_MOTIONS);
    if (motions_it == chunks.end())
        throw std::runtime_error("OMF file missing motions chunk");

    LegacyOmfData output;
    ParseSmparams(params_it->second, skeleton_bone_names, output);
    ParseMotions(motions_it->second, output, output);
    return output;
}

LegacyOmfData ParseOmfFile(const std::filesystem::path& path, const xr_vector<xr_string>& skeleton_bone_names)
{
    const auto data = LoadFileBytes(path);
    return ParseOmfBuffer(data.data(), data.size(), skeleton_bone_names);
}

ozz::animation::offline::RawAnimation BuildRawAnimation(const LegacyOmfMotion& motion, const LegacyOmfData& omf)
{
    const size_t joint_count = omf.bone_remap.size();
    if (joint_count == 0)
        throw std::runtime_error("OMF bone remap is empty");

    ozz::animation::offline::RawAnimation raw_animation;
    raw_animation.name = motion.name.c_str();
    raw_animation.duration = motion.frame_count > 1 ? (motion.frame_count - 1) * SAMPLE_SPF : SAMPLE_SPF;
    raw_animation.tracks.resize(joint_count);

    for (size_t remap_index = 0; remap_index < omf.bone_remap.size(); ++remap_index)
    {
        const u16 joint_index = omf.bone_remap[remap_index];
        if (joint_index >= joint_count)
            throw std::runtime_error("bone remap references invalid joint index");

        const LegacyBoneTrack& source_track = motion.bone_tracks[remap_index];
        auto& track = raw_animation.tracks[joint_index];

        track.translations.resize(motion.frame_count);
        track.rotations.resize(motion.frame_count);
        track.scales.resize(1);
        track.scales[0].time = 0.f;
        track.scales[0].value = ozz::math::Float3(1.f, 1.f, 1.f);

        for (u32 frame = 0; frame < motion.frame_count; ++frame)
        {
            const float time = static_cast<float>(frame) * SAMPLE_SPF;

            const Fquaternion& xr_quat = source_track.rotations[frame];
            const Fvector& xr_translation = source_track.translations[frame];

            Fmatrix local;
            local.mk_xform(xr_quat, xr_translation);

            const auto ozz_matrix = ConvertXrayLocalToOzz(local);
            track.translations[frame].time = time;
            track.translations[frame].value = ExtractTranslation(ozz_matrix);
            track.rotations[frame].time = time;
            track.rotations[frame].value = ExtractQuaternion(ozz_matrix);
        }
    }

    return raw_animation;
}

ConvertedOmfAnimation BuildConvertedAnimation(const LegacyOmfMotion& motion, const LegacyOmfData& omf, const ozz::animation::Skeleton& skeleton, bool optimize)
{
    ozz::animation::offline::RawAnimation raw_animation = BuildRawAnimation(motion, omf);

    ozz::animation::offline::AnimationBuilder builder;
    ozz::animation::offline::AnimationOptimizer optimizer;

    ozz::animation::offline::RawAnimation prepared = raw_animation;
    if (optimize)
    {
        ozz::animation::offline::RawAnimation optimized_raw;
        if (!optimizer(raw_animation, skeleton, &optimized_raw))
            throw std::runtime_error("animation optimization failed for motion: " + std::string(motion.name.c_str()));
        prepared = std::move(optimized_raw);
    }

    auto animation = builder(prepared);
    if (!animation)
        throw std::runtime_error("ozz animation build failed for motion: " + std::string(motion.name.c_str()));

    ConvertedOmfAnimation converted;
    converted.name = motion.name;
    converted.metadata = motion.metadata;
    converted.animation = std::move(animation);
    return converted;
}

LegacyOmfMotion const* FindMotionByName(const LegacyOmfData& omf, const xr_string& name)
{
    const std::string target = ToLowerCopy(std::string(name.c_str()));
    for (const auto& motion : omf.motions)
    {
        if (ToLowerCopy(std::string(motion.name.c_str())) == target)
            return &motion;
    }
    return nullptr;
}

bool ConvertLegacyOmfImpl(const LegacyOmfData& omf,
    const ozz::animation::Skeleton& skeleton,
    xr_vector<ConvertedOmfAnimation>& out_animations,
    const std::optional<xr_string>& motion_filter,
    bool optimize)
{
    out_animations.clear();

    if (motion_filter)
    {
        const LegacyOmfMotion* motion = FindMotionByName(omf, *motion_filter);
        if (!motion)
            return false;
        out_animations.emplace_back(BuildConvertedAnimation(*motion, omf, skeleton, optimize));
        return true;
    }

    out_animations.reserve(omf.motions.size());
    for (const auto& motion : omf.motions)
        out_animations.emplace_back(BuildConvertedAnimation(motion, omf, skeleton, optimize));

    return true;
}
} // namespace

bool ConvertLegacyOmf(const std::filesystem::path& omf_path,
                      const xr_vector<xr_string>& skeleton_bone_names,
                      const ozz::animation::Skeleton& skeleton,
                      xr_vector<ConvertedOmfAnimation>& out_animations,
                      std::optional<xr_string> motion_filter,
                      bool optimize)
{
    LegacyOmfData omf = ParseOmfFile(omf_path, skeleton_bone_names);
    return ConvertLegacyOmfImpl(omf, skeleton, out_animations, motion_filter, optimize);
}

bool ConvertLegacyOmf(const std::byte* data,
                      size_t size,
                      const xr_vector<xr_string>& skeleton_bone_names,
                      const ozz::animation::Skeleton& skeleton,
                      xr_vector<ConvertedOmfAnimation>& out_animations,
                      std::optional<xr_string> motion_filter,
                      bool optimize)
{
    if (!data || size == 0)
        return false;

    LegacyOmfData omf = ParseOmfBuffer(data, size, skeleton_bone_names);
    return ConvertLegacyOmfImpl(omf, skeleton, out_animations, motion_filter, optimize);
}
}
} // namespace XRay::Animation
