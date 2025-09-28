#include "stdafx.h"

#include "Common/Platform.hpp"

#include "xrCore/Animation/Bone.hpp"
#include "xrCore/Animation/SkeletonMotions.hpp"
#include "xrCore/FMesh.hpp"
#include "xrCore/_matrix.h"
#include "xrCore/_quaternion.h"
#include "xrCore/_vector3d.h"
#include "xrCore/xrCore.h"

#include <ozz/animation/offline/animation_builder.h>
#include <ozz/animation/offline/animation_optimizer.h>
#include <ozz/animation/offline/raw_animation.h>
#include <ozz/animation/offline/raw_skeleton.h>
#include <ozz/animation/offline/skeleton_builder.h>
#include <ozz/animation/runtime/animation.h>
#include <ozz/animation/runtime/skeleton.h>
#include <ozz/base/containers/vector_archive.h>
#include <ozz/base/io/archive.h>
#include <ozz/base/io/stream.h>
#include <ozz/base/log.h>
#include <ozz/base/maths/quaternion.h>
#include <ozz/base/maths/soa_transform.h>
#include <ozz/base/maths/vec_float.h>

#include "../Externals/ozz-animation/samples/framework/mesh.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "xrCore/Animation/SkeletonMotionDefs.hpp"

#include "../OzzBundle.h"

#ifdef main
#    undef main
#endif

namespace fs = std::filesystem;

namespace
{
namespace detail
{
using Matrix4 = std::array<std::array<float, 4>, 4>;

constexpr Matrix4 kXrayToOzz = {
    std::array<float, 4>{ 1.f, 0.f,  0.f, 0.f },
    std::array<float, 4>{ 0.f, 1.f,  0.f, 0.f },
    std::array<float, 4>{ 0.f, 0.f, -1.f, 0.f },
    std::array<float, 4>{ 0.f, 0.f,  0.f, 1.f }
};

constexpr Matrix4 kOzzToXray = kXrayToOzz;

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

Matrix4 ConvertXrayLocalToOzz(const Fmatrix& matrix)
{
    return ChangeBasis(ToColumnMajor(matrix), kXrayToOzz, kOzzToXray);
}

std::array<float, 3> ApplyBasis(const Matrix4& matrix, const std::array<float, 3>& vector)
{
    std::array<float, 3> result{};
    for (int row = 0; row < 3; ++row)
    {
        result[static_cast<size_t>(row)] =
            matrix[static_cast<size_t>(row)][0] * vector[0] + matrix[static_cast<size_t>(row)][1] * vector[1] + matrix[static_cast<size_t>(row)][2] * vector[2];
    }
    return result;
}

std::array<float, 3> ConvertVectorXrayToOzz(const Fvector& v)
{
    const std::array<float, 3> source{ v.x, v.y, v.z };
    return ApplyBasis(kXrayToOzz, source);
}

std::array<float, 2> ConvertUV(const Fvector2& uv)
{
    return { uv.x, uv.y };
}

std::array<float, 3> Normalize(const std::array<float, 3>& v)
{
    const float len_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if (len_sq <= std::numeric_limits<float>::epsilon())
        return v;
    const float inv_len = 1.f / std::sqrt(len_sq);
    return { v[0] * inv_len, v[1] * inv_len, v[2] * inv_len };
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
    std::array<uint16_t, 4> bones{ 0, 0, 0, 0 };
    std::array<float, 4> weights{ 0.f, 0.f, 0.f, 0.f };
    uint8_t influence_count = 0;
};

struct VertexDedupKey
{
    std::array<int32_t, 3> position{ 0, 0, 0 };
    std::array<uint16_t, 4> bones{ 0, 0, 0, 0 };
    std::array<int32_t, 4> weights{ 0, 0, 0, 0 };
    uint8_t influence_count = 0;
    uint8_t back_side = 0;

    bool operator==(const VertexDedupKey& other) const noexcept
    {
        return influence_count == other.influence_count &&
            back_side == other.back_side &&
            position == other.position &&
            bones == other.bones &&
            weights == other.weights;
    }
};

struct VertexDedupKeyHasher
{
    size_t operator()(const VertexDedupKey& key) const noexcept
    {
        size_t hash = 1469598103934665603ull;
        auto combine = [&hash](size_t value)
        {
            hash ^= value;
            hash *= 1099511628211ull;
        };

        combine(std::hash<uint8_t>{}(key.influence_count));
        combine(std::hash<uint8_t>{}(key.back_side));
        for (int32_t component : key.position)
            combine(std::hash<int32_t>{}(component));
        for (uint16_t bone : key.bones)
            combine(std::hash<uint16_t>{}(bone));
        for (int32_t weight : key.weights)
            combine(std::hash<int32_t>{}(weight));
        return hash;
    }
};

struct PositionQuantizedKey
{
    std::array<int32_t, 3> components{ 0, 0, 0 };

    bool operator==(const PositionQuantizedKey& other) const noexcept
    {
        return components == other.components;
    }
};

struct RoundedNormalKey
{
    std::array<int32_t, 3> components{ 0, 0, 0 };

    bool operator==(const RoundedNormalKey& other) const noexcept
    {
        return components == other.components;
    }
};

struct RoundedNormalHasher
{
    size_t operator()(const RoundedNormalKey& key) const noexcept
    {
        size_t hash = 1469598103934665603ull;
        auto combine = [&hash](size_t value)
        {
            hash ^= value;
            hash *= 1099511628211ull;
        };
        for (int32_t component : key.components)
            combine(std::hash<int32_t>{}(component));
        return hash;
    }
};

struct PositionQuantizedHasher
{
    size_t operator()(const PositionQuantizedKey& key) const noexcept
    {
        size_t hash = 1469598103934665603ull;
        auto combine = [&hash](size_t value)
        {
            hash ^= value;
            hash *= 1099511628211ull;
        };
        for (int32_t component : key.components)
            combine(std::hash<int32_t>{}(component));
        return hash;
    }
};

struct ConvertedVertex
{
    std::array<float, 3> position{ 0.f, 0.f, 0.f };
    std::array<float, 3> normal{ 0.f, 0.f, 0.f };
    std::array<float, 4> tangent{ 0.f, 0.f, 0.f, 1.f };
    std::array<float, 2> uv{ 0.f, 0.f };
    std::array<uint16_t, 4> joint_indices{ 0, 0, 0, 0 };
    std::array<float, 4> joint_weights{ 0.f, 0.f, 0.f, 0.f };
    uint8_t influence_count = 0;
    size_t original_index = 0;
};

struct PartData
{
    int influence_count = 0;
    std::vector<ConvertedVertex> vertices;
};

constexpr u32 kChunkHeader = OGF_HEADER;
constexpr u32 kChunkBoneNames = OGF_S_BONE_NAMES;
constexpr u32 kChunkIkData = OGF_S_IKDATA;
constexpr u32 kChunkVertices = OGF_VERTICES;
constexpr u32 kChunkIndices = OGF_INDICES;
constexpr u32 kChunkChildren = OGF_CHILDREN;
constexpr u32 kChunkTexture = OGF_TEXTURE;
constexpr u32 kChunkChildrenLinks = OGF_CHILDREN_L;
constexpr u32 kChunkLodDefinition = OGF_S_LODS;
constexpr u32 kChunkLodDefinition2 = OGF_LODDEF2;
constexpr u32 kChunkSwidata = OGF_SWIDATA;

std::vector<MeshVertex> read_mesh_vertices(const Chunk& chunk);
std::vector<uint16_t> read_mesh_indices(const Chunk& chunk);

struct SurfaceMetadata
{
    std::string texture_path;
    std::string shader_name;
    uint32_t original_vertex_count = 0;
    uint32_t original_face_count = 0;
    uint8_t ogf_type = 0;
    std::vector<std::string> lod_visuals;
    std::vector<uint8_t> lod_data;
    uint32_t progressive_collapse_count = 0;
    std::vector<uint8_t> progressive_data;
    std::vector<uint32_t> child_visual_links;
};

struct SurfaceDefinition
{
    std::vector<MeshVertex> vertices;
    std::vector<uint16_t> indices;
    SurfaceMetadata metadata;
};

std::vector<std::string> parse_lod_strings(const Chunk& chunk)
{
    std::vector<std::string> results;
    std::string current;
    current.reserve(chunk.size);

    for (size_t idx = 0; idx < chunk.size; ++idx)
    {
        const char ch = static_cast<char>(chunk.data[idx]);
        if (ch == '\0' || ch == '\n' || ch == '\r')
        {
            if (!current.empty())
            {
                results.push_back(current);
                current.clear();
            }
        }
        else
        {
            current.push_back(ch);
        }
    }

    if (!current.empty())
        results.push_back(current);

    return results;
}

void parse_texture_chunk(const Chunk& chunk, SurfaceMetadata& metadata)
{
    BinaryReader reader{ chunk.data, chunk.size };
    if (reader.offset >= reader.size)
        return;
    metadata.texture_path = reader.read_stringz();
    if (reader.offset < reader.size)
        metadata.shader_name = reader.read_stringz();
}

void parse_header_chunk(const Chunk& chunk, SurfaceMetadata& metadata)
{
    if (chunk.size < sizeof(ogf_header))
        return;
    ogf_header header{};
    std::memcpy(&header, chunk.data, sizeof(header));
    metadata.ogf_type = header.type;
}

void parse_lod_chunk(const Chunk& chunk, SurfaceMetadata& metadata)
{
    const auto* bytes = reinterpret_cast<const uint8_t*>(chunk.data);
    metadata.lod_data.assign(bytes, bytes + chunk.size);

    auto strings = parse_lod_strings(chunk);
    metadata.lod_visuals.insert(metadata.lod_visuals.end(), strings.begin(), strings.end());
}

void parse_children_links_chunk(const Chunk& chunk, SurfaceMetadata& metadata)
{
    if (chunk.size < sizeof(uint32_t))
        return;
    BinaryReader reader{ chunk.data, chunk.size };
    const uint32_t count = reader.read<u32>();
    metadata.child_visual_links.reserve(metadata.child_visual_links.size() + count);
    for (uint32_t idx = 0; idx < count && reader.offset + sizeof(uint32_t) <= reader.size; ++idx)
        metadata.child_visual_links.push_back(reader.read<u32>());
}

void parse_progressive_chunk(const Chunk& chunk, SurfaceMetadata& metadata)
{
    const auto* bytes = reinterpret_cast<const uint8_t*>(chunk.data);
    metadata.progressive_data.assign(bytes, bytes + chunk.size);

    if (chunk.size < sizeof(uint32_t) * 5)
        return;

    BinaryReader reader{ chunk.data, chunk.size };
    reader.skip(sizeof(uint32_t) * 4);
    metadata.progressive_collapse_count = reader.read<u32>();
}

std::optional<SurfaceDefinition> build_surface_from_chunk_list(const std::vector<std::pair<u32, Chunk>>& chunk_list)
{
    SurfaceDefinition surface;
    const Chunk* vertices_chunk = nullptr;
    const Chunk* indices_chunk = nullptr;

    for (const auto& entry : chunk_list)
    {
        const u32 id = entry.first;
        const Chunk& chunk = entry.second;

        switch (id)
        {
        case kChunkVertices:       vertices_chunk = &chunk; break;
        case kChunkIndices:        indices_chunk = &chunk; break;
        case kChunkTexture:        parse_texture_chunk(chunk, surface.metadata); break;
        case kChunkHeader:         parse_header_chunk(chunk, surface.metadata); break;
        case kChunkLodDefinition:
        case kChunkLodDefinition2: parse_lod_chunk(chunk, surface.metadata); break;
        case kChunkChildrenLinks:  parse_children_links_chunk(chunk, surface.metadata); break;
        case kChunkSwidata:        parse_progressive_chunk(chunk, surface.metadata); break;
        default:                   break;
        }
    }

    if (!vertices_chunk || !indices_chunk)
        return std::nullopt;

    surface.vertices = read_mesh_vertices(*vertices_chunk);
    surface.metadata.original_vertex_count = static_cast<uint32_t>(surface.vertices.size());
    surface.indices = read_mesh_indices(*indices_chunk);
    surface.metadata.original_face_count = static_cast<uint32_t>(surface.indices.size() / 3);

    return surface;
}

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

std::vector<std::uint8_t> ReadBinaryFileBytes(const fs::path& path)
{
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream)
        throw std::runtime_error("failed to open file: " + path.string());

    const auto size = stream.tellg();
    if (size < 0)
        throw std::runtime_error("failed to query size of file: " + path.string());

    std::vector<std::uint8_t> data(static_cast<std::size_t>(size));
    stream.seekg(0, std::ios::beg);
    stream.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));
    if (!stream)
        throw std::runtime_error("failed to read file: " + path.string());

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

        chunks[id] = Chunk{ data + offset, chunk_size };
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

        subchunks.emplace_back(id, Chunk{ chunk.data + offset, sub_size });
        offset += sub_size;
    }

    return subchunks;
}

std::string to_lower_copy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char c)
        {
            return static_cast<char>(std::tolower(c));
        });
    return value;
}

std::string trim_copy(std::string_view value)
{
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string_view::npos)
        return {};

    const auto end = value.find_last_not_of(" \t\r\n");
    return std::string(value.substr(begin, end - begin + 1));
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

std::vector<std::string> split_motion_ref_string(const std::string& combined)
{
    std::vector<std::string> result;
    std::string_view source(combined);
    size_t cursor = 0;
    while (cursor < source.size())
    {
        const size_t separator = source.find(',', cursor);
        const size_t length = separator == std::string_view::npos ? source.size() - cursor : separator - cursor;
        const std::string trimmed = trim_copy(source.substr(cursor, length));
        if (!trimmed.empty())
            result.emplace_back(trimmed);

        if (separator == std::string_view::npos)
            break;
        cursor = separator + 1;
    }
    return result;
}

std::vector<BoneRecord> read_bone_names(const Chunk& chunk)
{
    BinaryReader reader{ chunk.data, chunk.size };
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
    BinaryReader reader{ chunk.data, chunk.size };

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
        reader.read<u32>();   // IK flags
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
        const auto ozz_matrix = detail::ConvertXrayLocalToOzz(bone.local_transform);
        joint.transform.translation = detail::ExtractTranslation(ozz_matrix);
        joint.transform.rotation = detail::ExtractQuaternion(ozz_matrix);
        joint.transform.scale = { 1.0f, 1.0f, 1.0f };

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
    compute_global_transforms(bones);
    return bones;
}

std::vector<std::string> load_motion_refs_from_ogf(const fs::path& path)
{
    const auto file_data = load_file(path);
    const auto chunks = parse_chunks(file_data.data(), file_data.size());

    std::vector<std::string> references;

    const auto refs2_it = chunks.find(OGF_S_MOTION_REFS2);
    if (refs2_it != chunks.end())
    {
        BinaryReader reader{ refs2_it->second.data, refs2_it->second.size };
        const u32 count = reader.read<u32>();
        references.reserve(count);
        for (u32 index = 0; index < count; ++index)
        {
            std::string entry = reader.read_stringz();
            entry = trim_copy(entry);
            if (!entry.empty())
                references.emplace_back(std::move(entry));
        }
        return references;
    }

    const auto refs_it = chunks.find(OGF_S_MOTION_REFS);
    if (refs_it == chunks.end())
        return references;

    BinaryReader reader{ refs_it->second.data, refs_it->second.size };
    if (reader.size == 0)
        return references;

    std::string combined = reader.read_stringz();
    references = split_motion_ref_string(combined);
    return references;
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
    combined.erase(std::remove_if(combined.begin(), combined.end(),
                       [](const auto& entry)
                       {
                           return std::fabs(entry.second) <= std::numeric_limits<float>::epsilon();
                       }),
        combined.end());

    if (combined.empty())
    {
        combined.emplace_back(bone_ids[0], 1.f);
    }

    if (combined.size() > 4)
    {
        std::stable_sort(combined.begin(), combined.end(),
            [](const auto& lhs, const auto& rhs)
            {
                return lhs.second > rhs.second;
            });
        combined.resize(4);
    }

    std::sort(combined.begin(), combined.end(),
        [](const auto& lhs, const auto& rhs)
        {
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
    BinaryReader reader{ chunk.data, chunk.size };
    const uint32_t link_type = reader.read<u32>();
    const uint32_t vertex_count = reader.read<u32>();

    if (link_type == 0 || link_type > 4)
        throw std::runtime_error("unsupported vertex influence count in OGF mesh");

    std::vector<MeshVertex> vertices;
    vertices.reserve(vertex_count);

    for (uint32_t idx = 0; idx < vertex_count; ++idx)
    {
        MeshVertex vertex;
        std::array<uint16_t, 4> bone_ids{ 0, 0, 0, 0 };
        std::array<float, 4> raw_weights{ 0.f, 0.f, 0.f, 0.f };

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
    BinaryReader reader{ chunk.data, chunk.size };
    const uint32_t index_count = reader.read<u32>();
    std::vector<uint16_t> indices;
    indices.reserve(index_count);
    for (uint32_t idx = 0; idx < index_count; ++idx)
        indices.push_back(reader.read<u16>());
    return indices;
}

constexpr float kPositionQuantizeScale = 100000.f;
constexpr float kWeightQuantizeScale = 1000000.f;
constexpr float kNormalQuantizeScale = 1000.f;

PositionQuantizedKey quantize_position(const Fvector& position)
{
    PositionQuantizedKey key;
    key.components[0] = static_cast<int32_t>(std::llround(position.x * kPositionQuantizeScale));
    key.components[1] = static_cast<int32_t>(std::llround(position.y * kPositionQuantizeScale));
    key.components[2] = static_cast<int32_t>(std::llround(position.z * kPositionQuantizeScale));
    return key;
}

VertexDedupKey make_dedup_key(const MeshVertex& vertex, uint8_t back_side_flag)
{
    VertexDedupKey key;
    key.position = quantize_position(vertex.position).components;
    key.influence_count = vertex.influence_count;
    key.back_side = back_side_flag;

    for (size_t idx = 0; idx < key.bones.size(); ++idx)
    {
        if (idx < vertex.influence_count)
        {
            key.bones[idx] = vertex.bones[idx];
            key.weights[idx] = static_cast<int32_t>(std::llround(vertex.weights[idx] * kWeightQuantizeScale));
        }
        else
        {
            key.bones[idx] = 0;
            key.weights[idx] = 0;
        }
    }

    return key;
}

RoundedNormalKey round_normal(const Fvector& normal)
{
    RoundedNormalKey key;
    key.components[0] = static_cast<int32_t>(std::llround(normal.x * kNormalQuantizeScale));
    key.components[1] = static_cast<int32_t>(std::llround(normal.y * kNormalQuantizeScale));
    key.components[2] = static_cast<int32_t>(std::llround(normal.z * kNormalQuantizeScale));
    return key;
}

std::vector<uint8_t> compute_back_face_flags(const std::vector<MeshVertex>& vertices)
{
    std::unordered_map<PositionQuantizedKey, std::vector<size_t>, PositionQuantizedHasher> position_groups;
    position_groups.reserve(vertices.size());

    for (size_t index = 0; index < vertices.size(); ++index)
    {
        position_groups[quantize_position(vertices[index].position)].push_back(index);
    }

    std::vector<uint8_t> back_flags(vertices.size(), 0);

    for (const auto& group : position_groups)
    {
        const auto& indices = group.second;
        std::unordered_set<RoundedNormalKey, RoundedNormalHasher> reference_normals;
        reference_normals.reserve(indices.size());

        for (size_t vertex_index : indices)
        {
            const Fvector& normal = vertices[vertex_index].normal;
            const RoundedNormalKey rounded = round_normal(normal);
            RoundedNormalKey opposite = rounded;
            opposite.components[0] = -opposite.components[0];
            opposite.components[1] = -opposite.components[1];
            opposite.components[2] = -opposite.components[2];

            if (reference_normals.find(opposite) != reference_normals.end())
            {
                back_flags[vertex_index] = 1;
            }
            else
            {
                back_flags[vertex_index] = 0;
                reference_normals.insert(rounded);
            }
        }
    }

    return back_flags;
}

[[maybe_unused]] void deduplicate_vertices(std::vector<MeshVertex>& vertices, std::vector<uint16_t>& indices)
{
    if (vertices.empty())
        return;

    const size_t original_vertex_count = vertices.size();

    const std::vector<uint8_t> back_flags = compute_back_face_flags(vertices);

    std::unordered_map<VertexDedupKey, uint32_t, VertexDedupKeyHasher> map;
    map.reserve(vertices.size());

    std::vector<MeshVertex> deduplicated;
    deduplicated.reserve(vertices.size());

    std::vector<uint16_t> remap(vertices.size(), 0);
    std::vector<uint32_t> merge_counts;
    merge_counts.reserve(vertices.size());

    for (size_t idx = 0; idx < vertices.size(); ++idx)
    {
        const VertexDedupKey key = make_dedup_key(vertices[idx], back_flags[idx]);
        const auto [it, inserted] = map.emplace(key, static_cast<uint32_t>(deduplicated.size()));

        if (inserted)
        {
            deduplicated.push_back(vertices[idx]);
            merge_counts.push_back(1);
        }
        else
        {
            const uint32_t target = it->second;
            MeshVertex& existing = deduplicated[target];
            const uint32_t count = ++merge_counts[target];

            const float inv = 1.f / static_cast<float>(count);
            const float prev = static_cast<float>(count - 1);

            existing.position.x = (existing.position.x * prev + vertices[idx].position.x) * inv;
            existing.position.y = (existing.position.y * prev + vertices[idx].position.y) * inv;
            existing.position.z = (existing.position.z * prev + vertices[idx].position.z) * inv;

            existing.normal.x = (existing.normal.x * prev + vertices[idx].normal.x) * inv;
            existing.normal.y = (existing.normal.y * prev + vertices[idx].normal.y) * inv;
            existing.normal.z = (existing.normal.z * prev + vertices[idx].normal.z) * inv;

            existing.tangent.x = (existing.tangent.x * prev + vertices[idx].tangent.x) * inv;
            existing.tangent.y = (existing.tangent.y * prev + vertices[idx].tangent.y) * inv;
            existing.tangent.z = (existing.tangent.z * prev + vertices[idx].tangent.z) * inv;

            existing.binormal.x = (existing.binormal.x * prev + vertices[idx].binormal.x) * inv;
            existing.binormal.y = (existing.binormal.y * prev + vertices[idx].binormal.y) * inv;
            existing.binormal.z = (existing.binormal.z * prev + vertices[idx].binormal.z) * inv;

            existing.uv.x = (existing.uv.x * prev + vertices[idx].uv.x) * inv;
            existing.uv.y = (existing.uv.y * prev + vertices[idx].uv.y) * inv;
        }

        remap[idx] = it->second;
    }

    for (auto& vertex : deduplicated)
    {
        const float normal_length = std::sqrt(vertex.normal.x * vertex.normal.x + vertex.normal.y * vertex.normal.y + vertex.normal.z * vertex.normal.z);
        if (normal_length > std::numeric_limits<float>::epsilon())
        {
            const float inv = 1.f / normal_length;
            vertex.normal.x *= inv;
            vertex.normal.y *= inv;
            vertex.normal.z *= inv;
        }

        const float tangent_length = std::sqrt(vertex.tangent.x * vertex.tangent.x + vertex.tangent.y * vertex.tangent.y + vertex.tangent.z * vertex.tangent.z);
        if (tangent_length > std::numeric_limits<float>::epsilon())
        {
            const float inv = 1.f / tangent_length;
            vertex.tangent.x *= inv;
            vertex.tangent.y *= inv;
            vertex.tangent.z *= inv;
        }

        const float binormal_length =
            std::sqrt(vertex.binormal.x * vertex.binormal.x + vertex.binormal.y * vertex.binormal.y + vertex.binormal.z * vertex.binormal.z);
        if (binormal_length > std::numeric_limits<float>::epsilon())
        {
            const float inv = 1.f / binormal_length;
            vertex.binormal.x *= inv;
            vertex.binormal.y *= inv;
            vertex.binormal.z *= inv;
        }
    }

    for (uint16_t& index : indices)
    {
        if (index >= remap.size())
            throw std::runtime_error("index references vertex outside range during deduplication");
        index = remap[index];
    }

    vertices.swap(deduplicated);

    if (vertices.size() != original_vertex_count)
    {
        std::cout << "Deduplicated mesh vertices: " << original_vertex_count << " -> " << vertices.size() << std::endl;
    }
}

ozz::sample::Mesh build_mesh(const std::vector<MeshVertex>& vertices, const std::vector<uint16_t>& indices, const std::vector<BoneRecord>& bones,
    const SurfaceMetadata& metadata)
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
        converted.tangent = { tangent[0], tangent[1], tangent[2], handedness };
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
                const auto global_matrix = detail::ConvertXrayLocalToOzz(bones[bone_index].global_transform);
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

    mesh.xray_metadata.texture_path = metadata.texture_path;
    mesh.xray_metadata.shader_name = metadata.shader_name;
    mesh.xray_metadata.texture_link_present = false;
    mesh.xray_metadata.texture_link = 0;
    mesh.xray_metadata.shader_link_present = false;
    mesh.xray_metadata.shader_link = 0;
    mesh.xray_metadata.original_vertex_count = metadata.original_vertex_count;
    mesh.xray_metadata.original_face_count = metadata.original_face_count;
    mesh.xray_metadata.ogf_type = metadata.ogf_type;
    mesh.xray_metadata.lod_visuals = metadata.lod_visuals;
    mesh.xray_metadata.lod_data = metadata.lod_data;
    mesh.xray_metadata.progressive_collapse_count = metadata.progressive_collapse_count;
    mesh.xray_metadata.progressive_data = metadata.progressive_data;
    mesh.xray_metadata.child_visual_links = metadata.child_visual_links;

    return mesh;
}

void parse_smparams(const Chunk& chunk, const std::vector<BoneRecord>& skeleton_bones, OmfFile& output)
{
    BinaryReader reader{ chunk.data, chunk.size };

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

    BinaryReader count_reader{ count_chunk.second.data, count_chunk.second.size };
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
        BinaryReader reader{ item.second.data, item.second.size };

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

ozz::animation::offline::RawAnimation build_raw_animation_from_omf(const OmfMotion& motion, const OmfFile& omf, const std::vector<BoneRecord>& bones)
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
        case '"':  escaped += "\\\""; break;
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

void write_metadata_json(const fs::path& path, const std::vector<MotionMetadata>& metadata_list, const fs::path& source_omf)
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
        stream << bone.name << ',' << (bone.parent_name.empty() ? "" : bone.parent_name) << ',' << bone.local_transform.c.x << ',' << bone.local_transform.c.y
               << ',' << bone.local_transform.c.z << ',' << bone.global_transform.c.x << ',' << bone.global_transform.c.y << ',' << bone.global_transform.c.z
               << '\n';
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
    bool optimize = false;
};

struct MeshConfig
{
    fs::path input_ogf;
    fs::path output_ozz;
};

struct BundleConfig
{
    fs::path input_ogf;
    fs::path output_ozzx;
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
        throw std::
            runtime_error("usage: xray_to_ozz_converter animation <input.omf> <output.ozz> <skeleton.ogf> [--motion <name>] [--metadata <json>] [--optimize]");

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
        else if (arg == "--optimize")
        {
            config.optimize = true;
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
        throw std::runtime_error("usage: xray_to_ozz_converter mesh <input.ogf> <output.ozz>");

    MeshConfig config;
    config.input_ogf = fs::path(argv[2]);
    config.output_ozz = fs::path(argv[3]);
    return config;
}

BundleConfig parse_bundle_arguments(int argc, char** argv)
{
    if (argc < 4)
        throw std::runtime_error("usage: xray_to_ozz_converter bundle <input.ogf> <output.ozzx>");

    BundleConfig config;
    config.input_ogf = fs::path(argv[2]);
    config.output_ozzx = fs::path(argv[3]);
    return config;
}

void convert_skeleton(const SkeletonConfig& config)
{
    const auto start_time = std::chrono::steady_clock::now();

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

    const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();

    std::cout << "Converted skeleton written to " << config.output_ozz << " (" << duration_ms << " ms)" << std::endl;
}

void convert_animation(const AnimationConfig& config)
{
    const auto start_time = std::chrono::steady_clock::now();

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
    ozz::animation::offline::AnimationOptimizer optimizer;
    for (const OmfMotion* motion : motions_to_export)
    {
        auto raw_animation = build_raw_animation_from_omf(*motion, omf, bones);
        if (config.optimize)
        {
            ozz::animation::offline::RawAnimation optimized_raw;
            if (!optimizer(raw_animation, *skeleton, &optimized_raw))
                throw std::runtime_error("animation optimization failed for motion: " + motion->name);
            raw_animation = std::move(optimized_raw);
        }
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

    const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();

    if (motions_to_export.size() == 1)
    {
        std::cout << "Converted animation '" << motions_to_export.front()->name << "' written to " << config.output_ozz << " (" << duration_ms << " ms)"
                  << std::endl;
    }
    else
    {
        std::cout << "Converted " << motions_to_export.size() << " animations written to " << config.output_ozz << " (" << duration_ms << " ms)" << std::endl;
    }
}

void convert_mesh(const MeshConfig& config)
{
    const auto start_time = std::chrono::steady_clock::now();

    auto bones = load_skeleton_bones_from_ogf(config.input_ogf);

    const auto file_data = load_file(config.input_ogf);
    const auto chunks = parse_chunks(file_data.data(), file_data.size());

    std::vector<SurfaceDefinition> surfaces;

    if (const auto children_it = chunks.find(kChunkChildren); children_it != chunks.end())
    {
        const Chunk& children_chunk = children_it->second;
        BinaryReader reader{ children_chunk.data, children_chunk.size };
        while (reader.offset + sizeof(u32) * 2 <= children_chunk.size)
        {
            reader.read<u32>();
            const u32 child_size = reader.read<u32>();
            if (reader.offset + child_size > children_chunk.size)
                break;

            Chunk child_chunk{ children_chunk.data + reader.offset, child_size };
            reader.offset += child_size;

            const auto child_subchunks = parse_subchunks(child_chunk);
            if (auto surface = build_surface_from_chunk_list(child_subchunks))
                surfaces.push_back(std::move(*surface));
        }
    }

    if (surfaces.empty())
    {
        std::vector<std::pair<u32, Chunk>> root_chunks;
        const auto add_if_present = [&](u32 id)
        {
            const auto it = chunks.find(id);
            if (it != chunks.end())
                root_chunks.emplace_back(id, it->second);
        };

        add_if_present(kChunkHeader);
        add_if_present(kChunkTexture);
        add_if_present(kChunkLodDefinition);
        add_if_present(kChunkLodDefinition2);
        add_if_present(kChunkChildrenLinks);
        add_if_present(kChunkSwidata);
        add_if_present(kChunkVertices);
        add_if_present(kChunkIndices);

        if (auto surface = build_surface_from_chunk_list(root_chunks))
            surfaces.push_back(std::move(*surface));
    }

    if (surfaces.empty())
        throw std::runtime_error("OGF file does not contain any mesh surfaces");

    std::vector<ozz::sample::Mesh> meshes;
    meshes.reserve(surfaces.size());

    for (size_t surface_index = 0; surface_index < surfaces.size(); ++surface_index)
    {
        auto& surface = surfaces[surface_index];
        meshes.push_back(build_mesh(surface.vertices, surface.indices, bones, surface.metadata));
    }

    std::error_code ec;
    if (const auto parent = config.output_ozz.parent_path(); !parent.empty())
        fs::create_directories(parent, ec);

    ozz::io::File output(config.output_ozz.string().c_str(), "wb");
    if (!output.opened())
        throw std::runtime_error("failed to open output mesh file: " + config.output_ozz.string());

    ozz::io::OArchive archive(&output);
    for (const auto& mesh : meshes)
        archive << mesh;

    const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();

    std::cout << "Converted " << meshes.size() << " mesh surface" << (meshes.size() == 1 ? "" : "s") << " written to " << config.output_ozz << " ("
              << duration_ms << " ms)" << std::endl;
}

void convert_bundle(const BundleConfig& config)
{
    const auto start_time = std::chrono::steady_clock::now();

    fs::path skeleton_temp = config.output_ozzx;
    skeleton_temp += ".skeleton.tmp";
    fs::path mesh_temp = config.output_ozzx;
    mesh_temp += ".mesh.tmp";

    SkeletonConfig skeleton_cfg;
    skeleton_cfg.input_ogf = config.input_ogf;
    skeleton_cfg.output_ozz = skeleton_temp;
    convert_skeleton(skeleton_cfg);

    MeshConfig mesh_cfg;
    mesh_cfg.input_ogf = config.input_ogf;
    mesh_cfg.output_ozz = mesh_temp;
    convert_mesh(mesh_cfg);

    XRay::Animation::OzzxBundle bundle;
    bundle.version = 2u;
    bundle.skeleton = ReadBinaryFileBytes(skeleton_temp);
    bundle.mesh = ReadBinaryFileBytes(mesh_temp);

    const auto motion_refs = load_motion_refs_from_ogf(config.input_ogf);
    bundle.motion_refs.clear();
    bundle.motion_refs.reserve(motion_refs.size());
    for (const auto& reference : motion_refs)
        bundle.motion_refs.emplace_back(reference.c_str());

    if (!XRay::Animation::WriteOzzxBundle(config.output_ozzx, bundle))
        throw std::runtime_error("failed to write .ozzx bundle: " + config.output_ozzx.string());

    std::error_code ec;
    fs::remove(skeleton_temp, ec);
    fs::remove(mesh_temp, ec);

    const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();

    std::cout << "Converted bundle written to " << config.output_ozzx << " (" << duration_ms << " ms, version=" << bundle.version <<")" << std::endl;
}
} // namespace

int main(int argc, char** argv)
{
    try
    {
        if (argc < 2)
            throw std::runtime_error("usage: xray_to_ozz_converter <command> ...\n"
                                     "Commands:\n"
                                     "  skeleton <input.ogf> <output.ozz> [--dump-bind <csv>]\n"
                                     "  animation <input.omf> <output.ozz> <skeleton.ogf> [--motion <name>] [--metadata <json>]\n"
                                     "  mesh <input.ogf> <output.ozz>\n"
                                     "  bundle <input.ogf> <output.ozzx>");

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
        else if (command == "bundle")
        {
            convert_bundle(parse_bundle_arguments(argc, argv));
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
