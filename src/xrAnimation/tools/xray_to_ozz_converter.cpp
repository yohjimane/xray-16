#include "stdafx.h"

#include "Common/Platform.hpp"

#include "xrCore/xrCore.h"
#include "xrCore/FMesh.hpp"
#include "xrCore/_matrix.h"
#include "xrCore/_quaternion.h"
#include "xrCore/_vector3d.h"
#include "xrCore/Animation/Bone.hpp"
#include "xrCore/Animation/SkeletonMotions.hpp"

#include <ozz/animation/offline/raw_skeleton.h>
#include <ozz/animation/offline/skeleton_builder.h>
#include <ozz/animation/offline/raw_animation.h>
#include <ozz/animation/offline/animation_builder.h>
#include <ozz/animation/runtime/skeleton.h>
#include <ozz/animation/runtime/animation.h>
#include <ozz/base/io/archive.h>
#include <ozz/base/io/stream.h>
#include <ozz/base/log.h>
#include <ozz/base/maths/quaternion.h>
#include <ozz/base/maths/vec_float.h>
#include <ozz/base/maths/soa_transform.h>
#include <ozz/base/containers/vector_archive.h>

#include "../Externals/ozz-animation/samples/framework/mesh.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <cmath>
#include <functional>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <memory>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <map>
#include <utility>
#include <vector>

#include "xrCore/Animation/SkeletonMotionDefs.hpp"

namespace fs = std::filesystem;

namespace
{
namespace detail
{
using Matrix4 = std::array<std::array<float, 4>, 4>;

constexpr Matrix4 kXrayToBlender =
{
    std::array<float, 4>{1.f, 0.f, 0.f, 0.f},
    std::array<float, 4>{0.f, 0.f, 1.f, 0.f},
    std::array<float, 4>{0.f, 1.f, 0.f, 0.f},
    std::array<float, 4>{0.f, 0.f, 0.f, 1.f}
};

constexpr Matrix4 kXrayToBlenderInverse = kXrayToBlender;

constexpr Matrix4 kBlenderToOzz =
{
    std::array<float, 4>{1.f, 0.f, 0.f, 0.f},
    std::array<float, 4>{0.f, 0.f, 1.f, 0.f},
    std::array<float, 4>{0.f, -1.f, 0.f, 0.f},
    std::array<float, 4>{0.f, 0.f, 0.f, 1.f}
};

constexpr Matrix4 kOzzToBlender =
{
    std::array<float, 4>{1.f, 0.f, 0.f, 0.f},
    std::array<float, 4>{0.f, 0.f, -1.f, 0.f},
    std::array<float, 4>{0.f, 1.f, 0.f, 0.f},
    std::array<float, 4>{0.f, 0.f, 0.f, 1.f}
};

Matrix4 ToColumnMajor(const Fmatrix& source)
{
    Matrix4 result{};
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            result[static_cast<size_t>(row)][static_cast<size_t>(col)] = source.m[col][row];
    return result;
}

Fmatrix ToRowMajor(const Matrix4& matrix)
{
    Fmatrix result;
    result.identity();
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            result.m[col][row] = matrix[static_cast<size_t>(row)][static_cast<size_t>(col)];
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
                value += lhs[static_cast<size_t>(row)][static_cast<size_t>(k)]
                    * rhs[static_cast<size_t>(k)][static_cast<size_t>(col)];
            result[static_cast<size_t>(row)][static_cast<size_t>(col)] = value;
        }
    }
    return result;
}

Matrix4 ChangeBasis(const Matrix4& matrix, const Matrix4& basis, const Matrix4& basis_inverse)
{
    return Multiply(Multiply(basis, matrix), basis_inverse);
}

Matrix4 ConvertXrayToBlender(const Matrix4& matrix)
{
    return ChangeBasis(matrix, kXrayToBlender, kXrayToBlenderInverse);
}

Matrix4 ConvertBlenderToOzz(const Matrix4& matrix)
{
    return ChangeBasis(matrix, kBlenderToOzz, kOzzToBlender);
}

Fmatrix ApplyXrayToBlender(const Fmatrix& matrix)
{
    return ToRowMajor(ConvertXrayToBlender(ToColumnMajor(matrix)));
}

Matrix4 ConvertBlenderLocalToOzz(const Fmatrix& matrix)
{
    return ConvertBlenderToOzz(ToColumnMajor(matrix));
}

Matrix4 ConvertXrayLocalToOzz(const Fmatrix& matrix)
{
    return ConvertBlenderToOzz(ConvertXrayToBlender(ToColumnMajor(matrix)));
}

std::array<float, 3> ApplyBasis(const Matrix4& matrix, const std::array<float, 3>& vector)
{
    std::array<float, 3> result{};
    for (int row = 0; row < 3; ++row)
    {
        result[static_cast<size_t>(row)] =
            matrix[static_cast<size_t>(row)][0] * vector[0] +
            matrix[static_cast<size_t>(row)][1] * vector[1] +
            matrix[static_cast<size_t>(row)][2] * vector[2];
    }
    return result;
}

std::array<float, 3> ConvertVectorXrayToOzz(const Fvector& v)
{
    const std::array<float, 3> source{v.x, v.y, v.z};
    const auto blender = ApplyBasis(kXrayToBlender, source);
    const auto ozz_vec = ApplyBasis(kBlenderToOzz, blender);
    return ozz_vec;
}

std::array<float, 2> ConvertUV(const Fvector2& uv)
{
    return {uv.x, uv.y};
}

std::array<float, 3> Normalize(const std::array<float, 3>& v)
{
    const float len_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if (len_sq <= std::numeric_limits<float>::epsilon())
        return v;
    const float inv_len = 1.f / std::sqrt(len_sq);
    return {v[0] * inv_len, v[1] * inv_len, v[2] * inv_len};
}

float Dot(const std::array<float, 3>& a, const std::array<float, 3>& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

std::array<float, 3> Cross(const std::array<float, 3>& a, const std::array<float, 3>& b)
{
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    };
}

Matrix4 InvertMatrix(const Matrix4& matrix)
{
    const Fmatrix row_major = ToRowMajor(matrix);
    Fmatrix inverted;
    inverted.invert(row_major);
    return ToColumnMajor(inverted);
}

ozz::math::Float4x4 ToOzzFloat4x4(const Matrix4& matrix)
{
    ozz::math::Float4x4 result;
    for (int col = 0; col < 4; ++col)
    {
        float column[4] = {
            matrix[0][col],
            matrix[1][col],
            matrix[2][col],
            matrix[3][col],
        };
        result.cols[col] = ozz::math::simd_float4::LoadPtrU(column);
    }
    return result;
}

ozz::math::Float3 ExtractTranslation(const Matrix4& matrix)
{
    return {matrix[0][3], matrix[1][3], matrix[2][3]};
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

    const float length_sq = qx * qx + qy * qy + qz * qz + qw * qw;
    const float inv_length = length_sq > 0.f ? 1.f / std::sqrt(length_sq) : 1.f;
    return ozz::math::Quaternion(qx * inv_length, qy * inv_length, qz * inv_length, qw * inv_length);
}
} // namespace detail

struct Chunk
{
    const std::byte* data = nullptr;
    size_t size = 0;
};

struct BinaryReader
{
    const std::byte* data = nullptr;
    size_t size = 0;
    size_t offset = 0;

    template <class T>
    T read()
    {
        if (offset + sizeof(T) > size)
            throw std::runtime_error("unexpected end of chunk while reading typed data");

        T value;
        std::memcpy(&value, data + offset, sizeof(T));
        offset += sizeof(T);
        return value;
    }

    template <class T>
    T read_struct()
    {
        return read<T>();
    }

    int8_t read_int8()
    {
        if (offset + sizeof(int8_t) > size)
            throw std::runtime_error("unexpected end of chunk while reading int8");
        const int8_t value = *reinterpret_cast<const int8_t*>(data + offset);
        offset += sizeof(int8_t);
        return value;
    }

    uint8_t read_uint8()
    {
        if (offset + sizeof(uint8_t) > size)
            throw std::runtime_error("unexpected end of chunk while reading uint8");
        const uint8_t value = *reinterpret_cast<const uint8_t*>(data + offset);
        offset += sizeof(uint8_t);
        return value;
    }

    std::string read_stringz()
    {
        const auto* begin = data + offset;
        const auto* end = data + size;
        const auto* cursor = begin;
        while (cursor < end && *reinterpret_cast<const char*>(cursor) != '\0')
            ++cursor;

        if (cursor == end)
            throw std::runtime_error("unterminated string in chunk");

        std::string value(reinterpret_cast<const char*>(begin), static_cast<size_t>(cursor - begin));
        offset += static_cast<size_t>(cursor - begin) + 1; // consume null terminator
        return value;
    }

    Fvector read_fvector3()
    {
        Fvector v{};
        v.x = read<float>();
        v.y = read<float>();
        v.z = read<float>();
        return v;
    }

    void skip(size_t count)
    {
        if (offset + count > size)
            throw std::runtime_error("attempted to skip past end of chunk");
        offset += count;
    }
};

struct BoneRecord
{
    std::string name;
    std::string parent_name;
    int parent_index = -1;
    Fvector rest_translation{};
    Fvector rest_rotation{}; // XYZ angles in radians (engine convention)
    Fmatrix local_transform{};
    Fmatrix global_transform{};
    float mass = 0.f;
    Fvector center_of_mass{};
};

struct MotionMark
{
    std::string name;
    std::vector<std::pair<float, float>> intervals;
};

struct MotionMetadata
{
    std::string name;
    uint32_t flags = 0;
    uint16_t bone_or_part = 0;
    uint16_t motion_id = 0;
    float speed = 0.f;
    float power = 0.f;
    float accrue = 0.f;
    float falloff = 0.f;
    std::vector<MotionMark> marks;
};

struct MeshVertex
{
    Fvector position{};
    Fvector normal{};
    Fvector tangent{};
    Fvector binormal{};
    Fvector2 uv{};
    std::array<uint16_t, 4> bones{0, 0, 0, 0};
    std::array<float, 4> weights{0.f, 0.f, 0.f, 0.f};
    uint8_t influence_count = 0;
};

struct ConvertedVertex
{
    std::array<float, 3> position{0.f, 0.f, 0.f};
    std::array<float, 3> normal{0.f, 0.f, 0.f};
    std::array<float, 4> tangent{0.f, 0.f, 0.f, 1.f};
    std::array<float, 2> uv{0.f, 0.f};
    std::array<uint16_t, 4> joint_indices{0, 0, 0, 0};
    std::array<float, 4> joint_weights{0.f, 0.f, 0.f, 0.f};
    uint8_t influence_count = 0;
    size_t original_index = 0;
};

struct PartData
{
    int influence_count = 0;
    std::vector<ConvertedVertex> vertices;
};

struct BoneTrack
{
    std::vector<Fquaternion> rotations;
    std::vector<Fvector> translations;
};

struct OmfMotion
{
    std::string name;
    uint32_t frame_count = 0;
    std::vector<BoneTrack> bone_tracks;
    MotionMetadata metadata;
};

struct OmfFile
{
    std::vector<uint16_t> bone_remap; // index inside OMF -> skeleton bone index
    std::vector<std::string> remap_bone_names;
    std::vector<MotionMetadata> metadata;
    std::vector<OmfMotion> motions;
};

constexpr u32 kChunkHeader = OGF_HEADER;
constexpr u32 kChunkBoneNames = OGF_S_BONE_NAMES;
constexpr u32 kChunkIkData = OGF_S_IKDATA;
constexpr u32 kChunkVertices = OGF_VERTICES;
constexpr u32 kChunkIndices = OGF_INDICES;
constexpr u32 kChunkChildren = OGF_CHILDREN;

std::vector<std::byte> load_file(const fs::path& path)
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

std::unordered_map<u32, Chunk> parse_chunks(const std::byte* data, size_t size)
{
    std::unordered_map<u32, Chunk> chunks;

    size_t offset = 0;
    while (offset + sizeof(u32) * 2 <= size)
    {
        u32 id;
        u32 chunk_size;
        std::memcpy(&id, data + offset, sizeof(u32));
        offset += sizeof(u32);
        std::memcpy(&chunk_size, data + offset, sizeof(u32));
        offset += sizeof(u32);

        if (offset + chunk_size > size)
            throw std::runtime_error("chunk extends past end of file");

        chunks[id] = Chunk{data + offset, chunk_size};
        offset += chunk_size;
    }

    return chunks;
}

std::vector<std::pair<u32, Chunk>> parse_subchunks(const Chunk& chunk)
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
            throw std::runtime_error("sub-chunk extends past end of parent chunk");

        subchunks.emplace_back(id, Chunk{chunk.data + offset, sub_size});
        offset += sub_size;
    }

    return subchunks;
}

std::string to_lower_copy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c)
                   { return static_cast<char>(std::tolower(c)); });
    return value;
}

std::string read_string_crlf(BinaryReader& reader)
{
    std::string value;
    while (reader.offset < reader.size)
    {
        const char ch = static_cast<char>(reader.read_int8());
        if (ch == '\r')
        {
            if (reader.offset >= reader.size)
                throw std::runtime_error("unexpected end of data while reading CRLF string");
            const char lf = static_cast<char>(reader.read_int8());
            if (lf != '\n')
                throw std::runtime_error("expected LF after CR in motion mark name");
            break;
        }
        value.push_back(ch);
    }
    return value;
}

std::vector<BoneRecord> read_bone_names(const Chunk& chunk)
{
    BinaryReader reader{chunk.data, chunk.size};
    const u32 bone_count = reader.read<u32>();

    std::vector<BoneRecord> bones;
    bones.reserve(bone_count);

    for (u32 idx = 0; idx < bone_count; ++idx)
    {
        BoneRecord record;
        record.name = reader.read_stringz();
        record.parent_name = reader.read_stringz();
        reader.skip(sizeof(Fobb));
        bones.emplace_back(std::move(record));
    }

    return bones;
}

void read_ik_data(const Chunk& chunk, std::vector<BoneRecord>& bones)
{
    BinaryReader reader{chunk.data, chunk.size};

    for (auto& bone : bones)
    {
        const u32 version = reader.read<u32>();

        reader.read_stringz(); // game material
        (void)reader.read_struct<SBoneShape>();

        reader.read<u32>(); // joint type
        for (int axis = 0; axis < 3; ++axis)
        {
            reader.read<float>(); // min
            reader.read<float>(); // max
            reader.read<float>(); // spring
            reader.read<float>(); // damping
        }

        reader.read<float>(); // spring factor
        reader.read<float>(); // damping factor
        reader.read<u32>(); // IK flags
        reader.read<float>(); // break force
        reader.read<float>(); // break torque
        if (version > 0)
            reader.read<float>(); // friction

        bone.rest_rotation = reader.read_fvector3();
        bone.rest_translation = reader.read_fvector3();
        bone.mass = reader.read<float>();
        bone.center_of_mass = reader.read_fvector3();
    }
}

void compute_hierarchy(std::vector<BoneRecord>& bones)
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

        auto it = index_by_name.find(bone.parent_name);
        if (it == index_by_name.end())
            throw std::runtime_error("bone parent not found: " + bone.parent_name + " for bone " + bone.name);
        bone.parent_index = it->second;
    }
}

void compute_local_transforms(std::vector<BoneRecord>& bones)
{
    for (auto& bone : bones)
    {
        Fmatrix m;
        m.identity();
        m.setXYZi(bone.rest_rotation);
        m.translate_over(bone.rest_translation);
        bone.local_transform = m;
        bone.global_transform.identity();
    }
}

void compute_global_transforms(std::vector<BoneRecord>& bones)
{
    std::vector<std::vector<int>> children(bones.size());
    for (size_t idx = 0; idx < bones.size(); ++idx)
        if (bones[idx].parent_index >= 0)
            children[bones[idx].parent_index].push_back(static_cast<int>(idx));

    std::function<void(int, const Fmatrix&)> visit;
    visit = [&](int index, const Fmatrix& parent_matrix)
    {
        auto& bone = bones[index];
        bone.global_transform.mul_43(parent_matrix, bone.local_transform);
        for (int child : children[index])
            visit(child, bone.global_transform);
    };

    for (size_t idx = 0; idx < bones.size(); ++idx)
        if (bones[idx].parent_index < 0)
        {
            Fmatrix identity;
            identity.identity();
            visit(static_cast<int>(idx), identity);
        }
}

void convert_locals_to_blender_basis(std::vector<BoneRecord>& bones)
{
    for (auto& bone : bones)
    {
        bone.local_transform = detail::ApplyXrayToBlender(bone.local_transform);
    }
}

ozz::animation::offline::RawSkeleton build_raw_skeleton(const std::vector<BoneRecord>& bones)
{
    std::vector<std::vector<int>> children(bones.size());
    std::vector<int> roots;
    for (size_t idx = 0; idx < bones.size(); ++idx)
    {
        if (bones[idx].parent_index >= 0)
            children[bones[idx].parent_index].push_back(static_cast<int>(idx));
        else
            roots.push_back(static_cast<int>(idx));
    }

    ozz::animation::offline::RawSkeleton skeleton;
    skeleton.roots.resize(roots.size());

    std::function<void(int, ozz::animation::offline::RawSkeleton::Joint&)> populate;
    populate = [&](int index, ozz::animation::offline::RawSkeleton::Joint& joint)
    {
        const auto& bone = bones[static_cast<size_t>(index)];
        joint.name = bone.name.c_str();
        const auto ozz_matrix = detail::ConvertBlenderLocalToOzz(bone.local_transform);
        joint.transform.translation = detail::ExtractTranslation(ozz_matrix);
        joint.transform.rotation = detail::ExtractQuaternion(ozz_matrix);
        joint.transform.scale = {1.0f, 1.0f, 1.0f};

        const auto& child_indices = children[static_cast<size_t>(index)];
        joint.children.resize(child_indices.size());
        for (size_t child_idx = 0; child_idx < child_indices.size(); ++child_idx)
            populate(child_indices[child_idx], joint.children[child_idx]);
    };

    for (size_t root_idx = 0; root_idx < roots.size(); ++root_idx)
        populate(roots[root_idx], skeleton.roots[root_idx]);

    return skeleton;
}

std::vector<BoneRecord> load_skeleton_bones_from_ogf(const fs::path& path)
{
    const auto file_data = load_file(path);
    const auto chunks = parse_chunks(file_data.data(), file_data.size());

    auto bone_names_it = chunks.find(kChunkBoneNames);
    if (bone_names_it == chunks.end())
        throw std::runtime_error("OGF file missing bone names chunk");

    auto ik_data_it = chunks.find(kChunkIkData);
    if (ik_data_it == chunks.end())
        throw std::runtime_error("OGF file missing IK data chunk");

    auto bones = read_bone_names(bone_names_it->second);
    read_ik_data(ik_data_it->second, bones);
    compute_hierarchy(bones);
    compute_local_transforms(bones);
    convert_locals_to_blender_basis(bones);
    compute_global_transforms(bones);
    return bones;
}

void finalize_influences(MeshVertex& vertex, const std::array<uint16_t, 4>& bone_ids, const std::array<float, 4>& weights, uint32_t link_type)
{
    std::map<uint16_t, float> accum;
    for (uint32_t idx = 0; idx < link_type && idx < 4; ++idx)
    {
        const uint16_t bone = bone_ids[idx];
        const float weight = weights[idx];
        accum[bone] += weight;
    }

    std::vector<std::pair<uint16_t, float>> combined(accum.begin(), accum.end());
    combined.erase(std::remove_if(combined.begin(), combined.end(), [](const auto& entry) {
        return std::fabs(entry.second) <= std::numeric_limits<float>::epsilon();
    }), combined.end());

    if (combined.empty())
    {
        combined.emplace_back(bone_ids[0], 1.f);
    }

    if (combined.size() > 4)
    {
        std::stable_sort(combined.begin(), combined.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.second > rhs.second;
        });
        combined.resize(4);
    }

    std::sort(combined.begin(), combined.end(), [](const auto& lhs, const auto& rhs) {
        if (lhs.first != rhs.first)
            return lhs.first < rhs.first;
        return lhs.second > rhs.second;
    });

    float sum = 0.f;
    for (const auto& entry : combined)
        sum += entry.second;

    if (sum <= std::numeric_limits<float>::epsilon())
    {
        combined.clear();
        combined.emplace_back(bone_ids[0], 1.f);
        sum = 1.f;
    }

    const float inv_sum = 1.f / sum;
    for (auto& entry : combined)
        entry.second *= inv_sum;

    vertex.influence_count = static_cast<uint8_t>(combined.size());
    for (size_t idx = 0; idx < combined.size(); ++idx)
    {
        vertex.bones[idx] = combined[idx].first;
        vertex.weights[idx] = combined[idx].second;
    }

    for (size_t idx = combined.size(); idx < 4; ++idx)
    {
        vertex.bones[idx] = vertex.bones[0];
        vertex.weights[idx] = idx == 0 ? 1.f : 0.f;
    }
}

std::vector<MeshVertex> read_mesh_vertices(const Chunk& chunk)
{
    BinaryReader reader{chunk.data, chunk.size};
    const uint32_t link_type = reader.read<u32>();
    const uint32_t vertex_count = reader.read<u32>();

    if (link_type == 0 || link_type > 4)
        throw std::runtime_error("unsupported vertex influence count in OGF mesh");

    std::vector<MeshVertex> vertices;
    vertices.reserve(vertex_count);

    for (uint32_t idx = 0; idx < vertex_count; ++idx)
    {
        MeshVertex vertex;
        std::array<uint16_t, 4> bone_ids{0, 0, 0, 0};
        std::array<float, 4> raw_weights{0.f, 0.f, 0.f, 0.f};

        if (link_type == 1)
        {
            vertex.position = reader.read<Fvector>();
            vertex.normal = reader.read<Fvector>();
            vertex.tangent = reader.read<Fvector>();
            vertex.binormal = reader.read<Fvector>();
            vertex.uv = reader.read<Fvector2>();
            bone_ids[0] = static_cast<uint16_t>(reader.read<u32>());
            raw_weights[0] = 1.f;
        }
        else if (link_type == 2)
        {
            bone_ids[0] = reader.read<u16>();
            bone_ids[1] = reader.read<u16>();
            vertex.position = reader.read<Fvector>();
            vertex.normal = reader.read<Fvector>();
            vertex.tangent = reader.read<Fvector>();
            vertex.binormal = reader.read<Fvector>();
            const float secondary = reader.read<float>();
            raw_weights[1] = secondary;
            raw_weights[0] = 1.f - secondary;
            vertex.uv = reader.read<Fvector2>();
        }
        else if (link_type == 3 || link_type == 4)
        {
            for (uint32_t influence = 0; influence < link_type; ++influence)
                bone_ids[influence] = reader.read<u16>();

            vertex.position = reader.read<Fvector>();
            vertex.normal = reader.read<Fvector>();
            vertex.tangent = reader.read<Fvector>();
            vertex.binormal = reader.read<Fvector>();

            float weight_sum = 0.f;
            for (uint32_t influence = 0; influence < link_type - 1; ++influence)
            {
                const float w = reader.read<float>();
                raw_weights[influence] = w;
                weight_sum += w;
            }
            raw_weights[link_type - 1] = std::max(0.f, 1.f - weight_sum);

            vertex.uv = reader.read<Fvector2>();
        }
        else
        {
            throw std::runtime_error("unsupported skeleton link type in OGF mesh");
        }

        finalize_influences(vertex, bone_ids, raw_weights, link_type);
        vertices.emplace_back(vertex);
    }

    return vertices;
}

std::vector<uint16_t> read_mesh_indices(const Chunk& chunk)
{
    BinaryReader reader{chunk.data, chunk.size};
    const uint32_t index_count = reader.read<u32>();
    std::vector<uint16_t> indices;
    indices.reserve(index_count);
    for (uint32_t idx = 0; idx < index_count; ++idx)
        indices.push_back(reader.read<u16>());
    return indices;
}

ozz::sample::Mesh build_mesh(const std::vector<MeshVertex>& vertices,
                             const std::vector<uint16_t>& indices,
                             const std::vector<BoneRecord>& bones)
{
    std::array<PartData, 5> parts{};
    for (int influences = 0; influences < 5; ++influences)
        parts[static_cast<size_t>(influences)].influence_count = influences;

    std::unordered_map<uint16_t, uint16_t> joint_map;
    std::vector<uint16_t> joint_remaps;
    std::vector<ozz::math::Float4x4> inverse_bind_poses;

    std::vector<uint32_t> vertex_remap(vertices.size(), 0);

    for (size_t vertex_index = 0; vertex_index < vertices.size(); ++vertex_index)
    {
        const MeshVertex& source = vertices[vertex_index];
        ConvertedVertex converted;
        converted.original_index = vertex_index;

        const auto position = detail::ConvertVectorXrayToOzz(source.position);
        const auto normal = detail::Normalize(detail::ConvertVectorXrayToOzz(source.normal));
        const auto tangent = detail::Normalize(detail::ConvertVectorXrayToOzz(source.tangent));
        const auto binormal = detail::Normalize(detail::ConvertVectorXrayToOzz(source.binormal));

        converted.position = position;
        converted.normal = normal;

        const float handedness = detail::Dot(detail::Cross(normal, tangent), binormal) < 0.f ? -1.f : 1.f;
        converted.tangent = {tangent[0], tangent[1], tangent[2], handedness};
        const auto uv = detail::ConvertUV(source.uv);
        converted.uv = uv;

        uint8_t influence_count = std::max<uint8_t>(1, source.influence_count);
        float weight_sum = 0.f;
        for (uint8_t influence = 0; influence < influence_count; ++influence)
        {
            const uint16_t bone_index = source.bones[influence];
            const float weight = source.weights[influence];
            weight_sum += weight;

            const auto [it, inserted] = joint_map.try_emplace(bone_index, static_cast<uint16_t>(joint_remaps.size()));
            if (inserted)
            {
                joint_remaps.push_back(bone_index);
                const auto global_matrix = detail::ConvertBlenderToOzz(detail::ToColumnMajor(bones[bone_index].global_transform));
                const auto inverse_matrix = detail::InvertMatrix(global_matrix);
                inverse_bind_poses.push_back(detail::ToOzzFloat4x4(inverse_matrix));
            }

            converted.joint_indices[influence] = it->second;
            converted.joint_weights[influence] = weight;
        }

        if (weight_sum <= std::numeric_limits<float>::epsilon())
        {
            converted.joint_weights[0] = 1.f;
            influence_count = 1;
        }
        else
        {
            const float inv_sum = 1.f / weight_sum;
            for (uint8_t influence = 0; influence < influence_count; ++influence)
                converted.joint_weights[influence] *= inv_sum;
        }

        converted.influence_count = std::min<uint8_t>(4, influence_count);
        const int part_index = std::max<int>(1, converted.influence_count);
        parts[static_cast<size_t>(part_index)].vertices.emplace_back(converted);
    }

    ozz::sample::Mesh mesh;
    uint32_t next_vertex_index = 0;

    for (int influences = 1; influences <= 4; ++influences)
    {
        auto& data = parts[static_cast<size_t>(influences)];
        if (data.vertices.empty())
            continue;

        ozz::sample::Mesh::Part part;
        const size_t count = data.vertices.size();
        part.positions.resize(count * ozz::sample::Mesh::Part::kPositionsCpnts);
        part.normals.resize(count * ozz::sample::Mesh::Part::kNormalsCpnts);
        part.tangents.resize(count * ozz::sample::Mesh::Part::kTangentsCpnts);
        part.uvs.resize(count * ozz::sample::Mesh::Part::kUVsCpnts);
        part.joint_indices.resize(count * influences);
        if (influences > 1)
            part.joint_weights.resize(count * (influences - 1));

        for (size_t local_index = 0; local_index < count; ++local_index)
        {
            const ConvertedVertex& v = data.vertices[local_index];
            vertex_remap[v.original_index] = next_vertex_index + static_cast<uint32_t>(local_index);

            part.positions[local_index * 3 + 0] = v.position[0];
            part.positions[local_index * 3 + 1] = v.position[1];
            part.positions[local_index * 3 + 2] = v.position[2];

            part.normals[local_index * 3 + 0] = v.normal[0];
            part.normals[local_index * 3 + 1] = v.normal[1];
            part.normals[local_index * 3 + 2] = v.normal[2];

            part.tangents[local_index * 4 + 0] = v.tangent[0];
            part.tangents[local_index * 4 + 1] = v.tangent[1];
            part.tangents[local_index * 4 + 2] = v.tangent[2];
            part.tangents[local_index * 4 + 3] = v.tangent[3];

            part.uvs[local_index * 2 + 0] = v.uv[0];
            part.uvs[local_index * 2 + 1] = v.uv[1];

            for (int influence = 0; influence < influences; ++influence)
                part.joint_indices[local_index * influences + influence] = v.joint_indices[influence];

            if (influences > 1)
            {
                float accumulated = 0.f;
                for (int influence = 0; influence < influences - 1; ++influence)
                {
                    const float weight = v.joint_weights[influence];
                    part.joint_weights[local_index * (influences - 1) + influence] = weight;
                    accumulated += weight;
                }
            }
        }

        mesh.parts.push_back(std::move(part));
        next_vertex_index += static_cast<uint32_t>(count);
    }

    mesh.triangle_indices.resize(indices.size());
    for (size_t idx = 0; idx < indices.size(); ++idx)
    {
        const uint16_t original = indices[idx];
        if (original >= vertex_remap.size())
            throw std::runtime_error("index references vertex outside range");
        const uint32_t remapped = vertex_remap[original];
        if (remapped > std::numeric_limits<uint16_t>::max())
            throw std::runtime_error("remapped vertex index exceeds 16-bit range");
        mesh.triangle_indices[idx] = static_cast<uint16_t>(remapped);
    }

    for (size_t tri = 0; tri + 2 < mesh.triangle_indices.size(); tri += 3)
        std::swap(mesh.triangle_indices[tri + 1], mesh.triangle_indices[tri + 2]);

    mesh.joint_remaps.resize(joint_remaps.size());
    std::copy(joint_remaps.begin(), joint_remaps.end(), mesh.joint_remaps.begin());

    mesh.inverse_bind_poses.resize(inverse_bind_poses.size());
    std::copy(inverse_bind_poses.begin(), inverse_bind_poses.end(), mesh.inverse_bind_poses.begin());

    return mesh;
}

void parse_smparams(const Chunk& chunk, const std::vector<BoneRecord>& skeleton_bones, OmfFile& output)
{
    BinaryReader reader{chunk.data, chunk.size};

    const u16 version = reader.read<u16>();
    if (version > xrOGF_SMParamsVersion)
        throw std::runtime_error("unsupported OMF params version");

    const u16 part_count = reader.read<u16>();

    output.bone_remap.assign(skeleton_bones.size(), std::numeric_limits<uint16_t>::max());
    output.remap_bone_names.assign(skeleton_bones.size(), std::string{});

    std::unordered_map<std::string, uint16_t> skeleton_index_by_name;
    skeleton_index_by_name.reserve(skeleton_bones.size());
    for (uint16_t idx = 0; idx < skeleton_bones.size(); ++idx)
        skeleton_index_by_name.emplace(to_lower_copy(skeleton_bones[idx].name), idx);

    uint32_t bones_mapped = 0;

    for (u16 part = 0; part < part_count; ++part)
    {
        reader.read_stringz();
        const u16 bone_count = reader.read<u16>();
        for (u16 bone_idx = 0; bone_idx < bone_count; ++bone_idx)
        {
            const std::string bone_name_raw = reader.read_stringz();
            const uint32_t remap_index = reader.read<u32>();

            if (remap_index >= output.bone_remap.size())
                throw std::runtime_error("bone remap index out of range in OMF params");

            const auto it = skeleton_index_by_name.find(to_lower_copy(bone_name_raw));
            if (it == skeleton_index_by_name.end())
                throw std::runtime_error("bone " + bone_name_raw + " referenced in OMF params not found in skeleton");

            output.bone_remap[remap_index] = it->second;
            output.remap_bone_names[remap_index] = bone_name_raw;
            ++bones_mapped;
        }
    }

    if (bones_mapped != skeleton_bones.size())
        throw std::runtime_error("OMF bone remap does not cover all skeleton bones");

    const u16 motion_count = reader.read<u16>();
    output.metadata.clear();
    output.metadata.reserve(motion_count);

    for (u16 motion_idx = 0; motion_idx < motion_count; ++motion_idx)
    {
        MotionMetadata meta;
        meta.name = reader.read_stringz();
        meta.flags = reader.read<u32>();
        meta.bone_or_part = reader.read<u16>();
        meta.motion_id = reader.read<u16>();
        meta.speed = reader.read<float>();
        meta.power = reader.read<float>();
        meta.accrue = reader.read<float>();
        meta.falloff = reader.read<float>();

        if (version >= 4)
        {
            const u32 mark_count = reader.read<u32>();
            meta.marks.reserve(mark_count);
            for (u32 mark_idx = 0; mark_idx < mark_count; ++mark_idx)
            {
                MotionMark mark;
                mark.name = read_string_crlf(reader);
                const u32 interval_count = reader.read<u32>();
                mark.intervals.reserve(interval_count);
                for (u32 interval_idx = 0; interval_idx < interval_count; ++interval_idx)
                {
                    const float first = reader.read<float>();
                    const float second = reader.read<float>();
                    mark.intervals.emplace_back(first, second);
                }
                meta.marks.emplace_back(std::move(mark));
            }
        }

        output.metadata.emplace_back(std::move(meta));
    }
}

void parse_motions(const Chunk& chunk, const OmfFile& params, OmfFile& output)
{
    const auto subchunks = parse_subchunks(chunk);
    if (subchunks.empty())
        throw std::runtime_error("OMF motion chunk missing motion count");

    const auto& count_chunk = subchunks.front();
    if (count_chunk.first != 0)
        throw std::runtime_error("OMF motion chunk missing count sub-chunk");

    BinaryReader count_reader{count_chunk.second.data, count_chunk.second.size};
    const u32 motion_count = count_reader.read<u32>();
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
        BinaryReader reader{item.second.data, item.second.size};

        OmfMotion motion;
        motion.name = reader.read_stringz();
        motion.frame_count = reader.read<u32>();
        if (motion.frame_count == 0)
            throw std::runtime_error("motion has zero frames: " + motion.name);

        motion.metadata = output.metadata[motion_idx];
        motion.bone_tracks.resize(params.bone_remap.size());

        for (size_t track_idx = 0; track_idx < params.bone_remap.size(); ++track_idx)
        {
            BoneTrack track;
            track.rotations.resize(motion.frame_count);
            track.translations.resize(motion.frame_count);

            const uint8_t flags = reader.read_uint8();
            const bool rotation_present = (flags & flRKeyAbsent) == 0;
            const bool translation_present = (flags & flTKeyPresent) != 0;
            const bool high_quality_translation = (flags & flTKey16IsBit) != 0;

            auto read_quaternion = [&reader]() -> Fquaternion
            {
                const int16_t x = reader.read<int16_t>();
                const int16_t y = reader.read<int16_t>();
                const int16_t z = reader.read<int16_t>();
                const int16_t w = reader.read<int16_t>();
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
                reader.read<u32>(); // CRC, ignore for now
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
                reader.read<u32>(); // CRC

                if (high_quality_translation)
                {
                    std::vector<std::array<int16_t, 3>> samples(motion.frame_count);
                    for (u32 frame = 0; frame < motion.frame_count; ++frame)
                    {
                        samples[frame][0] = reader.read<int16_t>();
                        samples[frame][1] = reader.read<int16_t>();
                        samples[frame][2] = reader.read<int16_t>();
                    }

                    const Fvector size = reader.read_fvector3();
                    const Fvector init = reader.read_fvector3();

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
                        samples[frame][0] = reader.read_int8();
                        samples[frame][1] = reader.read_int8();
                        samples[frame][2] = reader.read_int8();
                    }

                    const Fvector size = reader.read_fvector3();
                    const Fvector init = reader.read_fvector3();

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
                const Fvector init = reader.read_fvector3();
                std::fill(track.translations.begin(), track.translations.end(), init);
            }

            motion.bone_tracks[track_idx] = std::move(track);
        }

        if (reader.offset != reader.size)
            throw std::runtime_error("unexpected extra data in motion chunk");

        output.motions.emplace_back(std::move(motion));
    }
}

OmfFile parse_omf_file(const fs::path& path, const std::vector<BoneRecord>& skeleton_bones)
{
    const auto data = load_file(path);
    const auto chunks = parse_chunks(data.data(), data.size());

    auto params_it = chunks.find(OGF_S_SMPARAMS);
    if (params_it == chunks.end())
        throw std::runtime_error("OMF file missing smparams chunk");

    auto motions_it = chunks.find(OGF_S_MOTIONS);
    if (motions_it == chunks.end())
        throw std::runtime_error("OMF file missing motions chunk");

    OmfFile result;
    parse_smparams(params_it->second, skeleton_bones, result);
    parse_motions(motions_it->second, result, result);
    return result;
}

ozz::animation::offline::RawAnimation build_raw_animation_from_omf(
    const OmfMotion& motion,
    const OmfFile& omf,
    const std::vector<BoneRecord>& bones)
{
    const size_t joint_count = bones.size();
    if (joint_count != omf.bone_remap.size())
        throw std::runtime_error("bone remap size mismatch with skeleton");

    ozz::animation::offline::RawAnimation raw_animation;
    raw_animation.name = motion.name.c_str();
    raw_animation.duration = motion.frame_count > 1 ? (motion.frame_count - 1) * SAMPLE_SPF : SAMPLE_SPF;
    raw_animation.tracks.resize(joint_count);

    for (size_t remap_index = 0; remap_index < omf.bone_remap.size(); ++remap_index)
    {
        const uint16_t joint_index = omf.bone_remap[remap_index];
        if (joint_index >= joint_count)
            throw std::runtime_error("bone remap references invalid joint index");

        const BoneTrack& source_track = motion.bone_tracks[remap_index];
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

            const auto ozz_matrix = detail::ConvertXrayLocalToOzz(local);
            track.translations[frame].time = time;
            track.translations[frame].value = detail::ExtractTranslation(ozz_matrix);
            track.rotations[frame].time = time;
            track.rotations[frame].value = detail::ExtractQuaternion(ozz_matrix);
        }
    }

    return raw_animation;
}

std::string escape_json(const std::string& value)
{
    std::string escaped;
    escaped.reserve(value.size());
    for (char ch : value)
    {
        switch (ch)
        {
        case '"': escaped += "\\\""; break;
        case '\\': escaped += "\\\\"; break;
        case '\n': escaped += "\\n"; break;
        case '\r': escaped += "\\r"; break;
        case '\t': escaped += "\\t"; break;
        default:
            if (static_cast<unsigned char>(ch) < 0x20)
            {
                char buffer[7];
                std::snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<unsigned char>(ch));
                escaped += buffer;
            }
            else
            {
                escaped += ch;
            }
            break;
        }
    }
    return escaped;
}

void SerializeString(ozz::io::OArchive& archive, const std::string& value)
{
    const uint32_t length = static_cast<uint32_t>(value.size());
    archive << length;
    if (length > 0)
        archive << ozz::io::MakeArray(value.c_str(), length);
}

void SerializeMotionMarks(ozz::io::OArchive& archive, const MotionMetadata& metadata)
{
    const uint32_t mark_count = static_cast<uint32_t>(metadata.marks.size());
    archive << mark_count;
    for (const auto& mark : metadata.marks)
    {
        SerializeString(archive, mark.name);
        const uint32_t interval_count = static_cast<uint32_t>(mark.intervals.size());
        archive << interval_count;
        for (const auto& interval : mark.intervals)
        {
            archive << interval.first;
            archive << interval.second;
        }
    }
}

void SerializeMotionMetadata(ozz::io::OArchive& archive, const MotionMetadata& metadata)
{
    SerializeString(archive, metadata.name);
    archive << metadata.flags;
    archive << metadata.bone_or_part;
    archive << metadata.motion_id;
    archive << metadata.speed;
    archive << metadata.power;
    archive << metadata.accrue;
    archive << metadata.falloff;
    SerializeMotionMarks(archive, metadata);
}

void write_metadata_json(const fs::path& path,
                         const std::vector<MotionMetadata>& metadata_list,
                         const fs::path& source_omf)
{
    std::error_code ec;
    if (const auto parent = path.parent_path(); !parent.empty())
        fs::create_directories(parent, ec);

    std::ofstream stream(path);
    if (!stream)
        throw std::runtime_error("failed to open metadata output file: " + path.string());

    stream << std::fixed << std::setprecision(6);
    stream << "{\n";
    stream << "  \"source_omf\": \"" << escape_json(source_omf.string()) << "\",\n";
    stream << "  \"motions\": [\n";
    for (size_t index = 0; index < metadata_list.size(); ++index)
    {
        const MotionMetadata& metadata = metadata_list[index];
        stream << "    {\n";
        stream << "      \"motion_name\": \"" << escape_json(metadata.name) << "\",\n";
        stream << "      \"flags\": " << metadata.flags << ",\n";
        stream << "      \"bone_or_part\": " << metadata.bone_or_part << ",\n";
        stream << "      \"motion_id\": " << metadata.motion_id << ",\n";
        stream << "      \"speed\": " << metadata.speed << ",\n";
        stream << "      \"power\": " << metadata.power << ",\n";
        stream << "      \"accrue\": " << metadata.accrue << ",\n";
        stream << "      \"falloff\": " << metadata.falloff << ",\n";
        stream << "      \"marks\": [\n";
        for (size_t mark_index = 0; mark_index < metadata.marks.size(); ++mark_index)
        {
            const auto& mark = metadata.marks[mark_index];
            stream << "        {\n";
            stream << "          \"name\": \"" << escape_json(mark.name) << "\",\n";
            stream << "          \"intervals\": [";
            for (size_t interval_index = 0; interval_index < mark.intervals.size(); ++interval_index)
            {
                const auto& interval = mark.intervals[interval_index];
                stream << '[' << interval.first << ", " << interval.second << ']';
                if (interval_index + 1 < mark.intervals.size())
                    stream << ", ";
            }
            stream << "]\n";
            stream << "        }";
            if (mark_index + 1 < metadata.marks.size())
                stream << ',';
            stream << "\n";
        }
        stream << "      ]\n";
        stream << "    }";
        if (index + 1 < metadata_list.size())
            stream << ',';
        stream << "\n";
    }
    stream << "  ]\n";
    stream << "}\n";
}

void dump_bind_pose_csv(const fs::path& output, const std::vector<BoneRecord>& bones)
{
    std::ofstream stream(output);
    if (!stream)
        throw std::runtime_error("failed to open dump file: " + output.string());

    stream << std::fixed << std::setprecision(6);
    stream << "bone,parent,local_tx,local_ty,local_tz,global_tx,global_ty,global_tz\n";
    for (const auto& bone : bones)
    {
        stream << bone.name << ','
               << (bone.parent_name.empty() ? "" : bone.parent_name) << ','
               << bone.local_transform.c.x << ','
               << bone.local_transform.c.y << ','
               << bone.local_transform.c.z << ','
               << bone.global_transform.c.x << ','
               << bone.global_transform.c.y << ','
               << bone.global_transform.c.z << '\n';
    }
}

struct SkeletonConfig
{
    fs::path input_ogf;
    fs::path output_ozz;
    std::optional<fs::path> dump_csv;
};

struct AnimationConfig
{
    fs::path input_omf;
    fs::path output_ozz;
    fs::path skeleton_ogf;
    std::optional<std::string> motion_name;
    std::optional<fs::path> metadata_path;
};

struct MeshConfig
{
    fs::path input_ogf;
    fs::path output_ozz;
};

SkeletonConfig parse_skeleton_arguments(int argc, char** argv)
{
    if (argc < 4)
        throw std::runtime_error("usage: xray_to_ozz_converter skeleton <input.ogf> <output.ozz> [--dump-bind <csv>]");

    SkeletonConfig config;
    config.input_ogf = fs::path(argv[2]);
    config.output_ozz = fs::path(argv[3]);

    for (int idx = 4; idx < argc; ++idx)
    {
        std::string_view arg(argv[idx]);
        if (arg == "--dump-bind")
        {
            if (idx + 1 >= argc)
                throw std::runtime_error("--dump-bind requires a path argument");
            config.dump_csv = fs::path(argv[++idx]);
        }
        else
        {
            throw std::runtime_error("unknown option: " + std::string(arg));
        }
    }

    return config;
}

AnimationConfig parse_animation_arguments(int argc, char** argv)
{
    if (argc < 5)
        throw std::runtime_error(
            "usage: xray_to_ozz_converter animation <input.omf> <output.ozz> <skeleton.ogf> [--motion <name>] [--metadata <json>]" );

    AnimationConfig config;
    config.input_omf = fs::path(argv[2]);
    config.output_ozz = fs::path(argv[3]);
    config.skeleton_ogf = fs::path(argv[4]);

    for (int idx = 5; idx < argc; ++idx)
    {
        std::string_view arg(argv[idx]);
        if (arg == "--motion")
        {
            if (idx + 1 >= argc)
                throw std::runtime_error("--motion requires a name argument");
            config.motion_name = std::string(argv[++idx]);
        }
        else if (arg == "--metadata")
        {
            if (idx + 1 >= argc)
                throw std::runtime_error("--metadata requires a path argument");
            config.metadata_path = fs::path(argv[++idx]);
        }
        else
        {
            throw std::runtime_error("unknown option: " + std::string(arg));
        }
    }

    return config;
}

MeshConfig parse_mesh_arguments(int argc, char** argv)
{
    if (argc < 4)
        throw std::runtime_error(
            "usage: xray_to_ozz_converter mesh <input.ogf> <output.ozz>");

    MeshConfig config;
    config.input_ogf = fs::path(argv[2]);
    config.output_ozz = fs::path(argv[3]);
    return config;
}

void convert_skeleton(const SkeletonConfig& config)
{
    auto bones = load_skeleton_bones_from_ogf(config.input_ogf);
    const auto raw = build_raw_skeleton(bones);
    ozz::animation::offline::SkeletonBuilder builder;
    auto skeleton = builder(raw);
    if (!skeleton)
        throw std::runtime_error("ozz skeleton build failed");

    std::error_code ec;
    if (const auto parent = config.output_ozz.parent_path(); !parent.empty())
        fs::create_directories(parent, ec);

    ozz::io::File output(config.output_ozz.string().c_str(), "wb");
    if (!output.opened())
        throw std::runtime_error("failed to open output file: " + config.output_ozz.string());

    ozz::io::OArchive archive(&output);
    archive << *skeleton;

    if (config.dump_csv)
        dump_bind_pose_csv(*config.dump_csv, bones);

    std::cout << "Converted skeleton written to " << config.output_ozz << std::endl;
}

void convert_animation(const AnimationConfig& config)
{
    auto bones = load_skeleton_bones_from_ogf(config.skeleton_ogf);
    const auto raw_skeleton = build_raw_skeleton(bones);
    ozz::animation::offline::SkeletonBuilder skeleton_builder;
    auto skeleton = skeleton_builder(raw_skeleton);
    if (!skeleton)
        throw std::runtime_error("ozz skeleton build failed");

    OmfFile omf = parse_omf_file(config.input_omf, bones);
    if (omf.motions.empty())
        throw std::runtime_error("OMF file contains no motions");

    std::vector<const OmfMotion*> motions_to_export;
    motions_to_export.reserve(omf.motions.size());

    if (config.motion_name)
    {
        const std::string target = to_lower_copy(*config.motion_name);
        for (const auto& motion : omf.motions)
        {
            if (to_lower_copy(motion.name) == target)
            {
                motions_to_export.push_back(&motion);
                break;
            }
        }
        if (motions_to_export.empty())
            throw std::runtime_error("requested motion '" + *config.motion_name + "' not found in OMF");
    }
    else
    {
        for (const auto& motion : omf.motions)
            motions_to_export.push_back(&motion);
    }

    if (motions_to_export.empty())
        throw std::runtime_error("no animations selected for export");

    std::vector<MotionMetadata> metadata_to_write;
    metadata_to_write.reserve(motions_to_export.size());

    std::error_code ec;
    if (const auto parent = config.output_ozz.parent_path(); !parent.empty())
        fs::create_directories(parent, ec);

    ozz::io::File output(config.output_ozz.string().c_str(), "wb");
    if (!output.opened())
        throw std::runtime_error("failed to open output animation file: " + config.output_ozz.string());

    ozz::io::OArchive archive(&output);
    const uint32_t animation_count = static_cast<uint32_t>(motions_to_export.size());
    archive << animation_count;

    ozz::animation::offline::AnimationBuilder builder;
    for (const OmfMotion* motion : motions_to_export)
    {
        auto raw_animation = build_raw_animation_from_omf(*motion, omf, bones);
        auto animation = builder(raw_animation);
        if (!animation)
            throw std::runtime_error("ozz animation build failed");

        archive << *animation;

        MotionMetadata metadata = motion->metadata;
        if (metadata.name.empty())
        {
            metadata.name = motion->name;
        }
        SerializeMotionMetadata(archive, metadata);
        metadata_to_write.push_back(std::move(metadata));
    }

    fs::path metadata_path = config.metadata_path.value_or(config.output_ozz);
    if (!config.metadata_path)
        metadata_path.replace_extension(".json");
    write_metadata_json(metadata_path, metadata_to_write, config.input_omf);

    if (motions_to_export.size() == 1)
    {
        std::cout << "Converted animation '" << motions_to_export.front()->name
                  << "' written to " << config.output_ozz << std::endl;
    }
    else
    {
        std::cout << "Converted " << motions_to_export.size() << " animations written to "
                  << config.output_ozz << std::endl;
    }
}

void convert_mesh(const MeshConfig& config)
{
    auto bones = load_skeleton_bones_from_ogf(config.input_ogf);

    const auto file_data = load_file(config.input_ogf);
    const auto chunks = parse_chunks(file_data.data(), file_data.size());

    std::vector<MeshVertex> all_vertices;
    std::vector<uint16_t> all_indices;

    auto children_it = chunks.find(kChunkChildren);
    if (children_it != chunks.end())
    {
        const Chunk& children_chunk = children_it->second;
        BinaryReader reader{children_chunk.data, children_chunk.size};
        while (reader.offset + sizeof(u32) * 2 <= children_chunk.size)
        {
        reader.read<u32>();
            const u32 child_size = reader.read<u32>();
            if (reader.offset + child_size > children_chunk.size)
                break;

            Chunk child_chunk{children_chunk.data + reader.offset, child_size};
            reader.offset += child_size;

            size_t section_offset = 0;
            Chunk vertices_chunk{};
            Chunk indices_chunk{};
            bool has_vertices = false;
            bool has_indices = false;

            while (section_offset + sizeof(u32) * 2 <= child_chunk.size)
            {
                u32 section_id;
                u32 section_size;
                std::memcpy(&section_id, child_chunk.data + section_offset, sizeof(u32));
                section_offset += sizeof(u32);
                std::memcpy(&section_size, child_chunk.data + section_offset, sizeof(u32));
                section_offset += sizeof(u32);
                if (section_offset + section_size > child_chunk.size)
                    break;

                if (section_id == kChunkVertices)
                {
                    vertices_chunk.data = child_chunk.data + section_offset;
                    vertices_chunk.size = section_size;
                    has_vertices = true;
                }
                else if (section_id == kChunkIndices)
                {
                    indices_chunk.data = child_chunk.data + section_offset;
                    indices_chunk.size = section_size;
                    has_indices = true;
                }

                section_offset += section_size;
            }

            if (has_vertices && has_indices)
            {
                auto section_vertices = read_mesh_vertices(vertices_chunk);
                auto section_indices = read_mesh_indices(indices_chunk);

                const uint32_t base = static_cast<uint32_t>(all_vertices.size());
                all_vertices.insert(all_vertices.end(), section_vertices.begin(), section_vertices.end());
                all_indices.reserve(all_indices.size() + section_indices.size());
                for (uint16_t index : section_indices)
                    all_indices.push_back(static_cast<uint16_t>(base + index));
            }
        }
    }

    if (all_vertices.empty())
    {
        auto vertices_it = chunks.find(kChunkVertices);
        auto indices_it = chunks.find(kChunkIndices);
        if (vertices_it == chunks.end() || indices_it == chunks.end())
            throw std::runtime_error("OGF file missing vertices chunk");

        all_vertices = read_mesh_vertices(vertices_it->second);
        auto base_indices = read_mesh_indices(indices_it->second);
        all_indices.assign(base_indices.begin(), base_indices.end());
    }

    auto mesh = build_mesh(all_vertices, all_indices, bones);

    std::error_code ec;
    if (const auto parent = config.output_ozz.parent_path(); !parent.empty())
        fs::create_directories(parent, ec);

    ozz::io::File output(config.output_ozz.string().c_str(), "wb");
    if (!output.opened())
        throw std::runtime_error("failed to open output mesh file: " + config.output_ozz.string());

    ozz::io::OArchive archive(&output);
    archive << mesh;

    std::cout << "Converted mesh written to " << config.output_ozz << std::endl;
}

} // namespace

int main(int argc, char** argv)
{
    try
    {
        if (argc < 2)
            throw std::runtime_error(
                "usage: xray_to_ozz_converter <command> ...\n"
                "Commands:\n"
                "  skeleton <input.ogf> <output.ozz> [--dump-bind <csv>]\n"
                "  animation <input.omf> <output.ozz> <skeleton.ogf> [--motion <name>] [--metadata <json>]\n"
                "  mesh <input.ogf> <output.ozz>" );

        const std::string command = argv[1];
        if (command == "skeleton")
        {
            convert_skeleton(parse_skeleton_arguments(argc, argv));
        }
        else if (command == "animation")
        {
            convert_animation(parse_animation_arguments(argc, argv));
        }
        else if (command == "mesh")
        {
            convert_mesh(parse_mesh_arguments(argc, argv));
        }
        else
        {
            throw std::runtime_error("unknown command: " + command);
        }

        return EXIT_SUCCESS;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "ERROR: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (...)
    {
        std::cerr << "ERROR: unknown failure" << std::endl;
        return EXIT_FAILURE;
    }
}
