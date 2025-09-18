#include "stdafx.h"

#include "xrCore/_matrix.h"
#include "xrCore/_quaternion.h"
#include "xrCore/_vector3d.h"
#include "xrCore/Animation/Bone.hpp"

#include <ozz/animation/offline/raw_skeleton.h>
#include <ozz/animation/offline/skeleton_builder.h>
#include <ozz/base/io/archive.h>
#include <ozz/base/io/stream.h>
#include <ozz/base/log.h>
#include <ozz/base/maths/quaternion.h>
#include <ozz/base/maths/vec_float.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <cmath>
#include <functional>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace
{
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

constexpr u32 kChunkHeader = OGF_HEADER;
constexpr u32 kChunkBoneNames = OGF_S_BONE_NAMES;
constexpr u32 kChunkIkData = OGF_S_IKDATA;

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
    const auto to_column = [](const Fmatrix& source, float out[4][4])
    {
        for (int row = 0; row < 4; ++row)
            for (int col = 0; col < 4; ++col)
                out[row][col] = source.m[col][row];
    };

    const auto to_row = [](const float in[4][4], Fmatrix& dest)
    {
        for (int row = 0; row < 4; ++row)
            for (int col = 0; col < 4; ++col)
                dest.m[row][col] = in[col][row];
    };

    const auto multiply = [](const float left[4][4], const float right[4][4], float result[4][4])
    {
        for (int row = 0; row < 4; ++row)
        {
            for (int col = 0; col < 4; ++col)
            {
                float value = 0.f;
                for (int k = 0; k < 4; ++k)
                    value += left[row][k] * right[k][col];
                result[row][col] = value;
            }
        }
    };

    const float basis[4][4] =
    {
        {1.f, 0.f, 0.f, 0.f},
        {0.f, 0.f, 1.f, 0.f},
        {0.f, 1.f, 0.f, 0.f},
        {0.f, 0.f, 0.f, 1.f}
    };

    float tmp[4][4];
    float tmp2[4][4];

    for (auto& bone : bones)
    {
        float column_matrix[4][4];
        to_column(bone.local_transform, column_matrix);

        multiply(basis, column_matrix, tmp);
        multiply(tmp, basis, tmp2); // basis inverse equals basis

        to_row(tmp2, bone.local_transform);
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

    const auto multiply = [](const float lhs[4][4], const float rhs[4][4], float result[4][4])
    {
        for (int row = 0; row < 4; ++row)
        {
            for (int col = 0; col < 4; ++col)
            {
                float value = 0.f;
                for (int k = 0; k < 4; ++k)
                    value += lhs[row][k] * rhs[k][col];
                result[row][col] = value;
            }
        }
    };

    const float blender_to_ozz[4][4] =
    {
        {1.f, 0.f, 0.f, 0.f},
        {0.f, 0.f, 1.f, 0.f},
        {0.f, -1.f, 0.f, 0.f},
        {0.f, 0.f, 0.f, 1.f}
    };

    const float blender_to_ozz_inv[4][4] =
    {
        {1.f, 0.f, 0.f, 0.f},
        {0.f, 0.f, -1.f, 0.f},
        {0.f, 1.f, 0.f, 0.f},
        {0.f, 0.f, 0.f, 1.f}
    };

    std::function<void(int, ozz::animation::offline::RawSkeleton::Joint&)> populate;
    populate = [&](int index, ozz::animation::offline::RawSkeleton::Joint& joint)
    {
        const auto& bone = bones[static_cast<size_t>(index)];
        joint.name = bone.name.c_str();

        float column_matrix[4][4];
        for (int row = 0; row < 4; ++row)
            for (int col = 0; col < 4; ++col)
                column_matrix[row][col] = bone.local_transform.m[col][row];

        float tmp[4][4];
        float ozz_matrix[4][4];
        multiply(blender_to_ozz, column_matrix, tmp);
        multiply(tmp, blender_to_ozz_inv, ozz_matrix);

        const float tx = ozz_matrix[0][3];
        const float ty = ozz_matrix[1][3];
        const float tz = ozz_matrix[2][3];

        joint.transform.translation = {tx, ty, tz};

        const float m00 = ozz_matrix[0][0];
        const float m01 = ozz_matrix[0][1];
        const float m02 = ozz_matrix[0][2];
        const float m10 = ozz_matrix[1][0];
        const float m11 = ozz_matrix[1][1];
        const float m12 = ozz_matrix[1][2];
        const float m20 = ozz_matrix[2][0];
        const float m21 = ozz_matrix[2][1];
        const float m22 = ozz_matrix[2][2];

        float qw, qx, qy, qz;
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

        const float length = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        const float inv_length = length > 0.f ? 1.f / length : 1.f;
        joint.transform.rotation = {qx * inv_length, qy * inv_length, qz * inv_length, qw * inv_length};
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

struct ConverterConfig
{
    fs::path input_ogf;
    fs::path output_ozz;
    std::optional<fs::path> dump_csv;
};

ConverterConfig parse_arguments(int argc, char** argv)
{
    if (argc < 4)
        throw std::runtime_error("usage: xray_to_ozz_converter skeleton <input.ogf> <output.ozz> [--dump-bind <csv>]");

    if (std::string_view(argv[1]) != "skeleton")
        throw std::runtime_error("unknown command: " + std::string(argv[1]));

    ConverterConfig config;
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

} // namespace

int main(int argc, char** argv)
{
    try
    {
        const ConverterConfig config = parse_arguments(argc, argv);

        const auto file_data = load_file(config.input_ogf);
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

        const auto raw_skeleton = build_raw_skeleton(bones);
        ozz::animation::offline::SkeletonBuilder builder;
        auto skeleton = builder(raw_skeleton);
        if (!skeleton)
            throw std::runtime_error("ozz skeleton build failed");

        ozz::io::File output(config.output_ozz.string().c_str(), "wb");
        if (!output.opened())
            throw std::runtime_error("failed to open output file: " + config.output_ozz.string());

        ozz::io::OArchive archive(&output);
        archive << *skeleton;

        if (config.dump_csv)
            dump_bind_pose_csv(*config.dump_csv, bones);

        std::cout << "Converted skeleton written to " << config.output_ozz << std::endl;

        return EXIT_SUCCESS;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "ERROR: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }
}
