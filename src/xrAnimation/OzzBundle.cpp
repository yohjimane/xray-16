#include "stdafx.h"

#include "OzzBundle.h"

#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>

namespace XRay
{
namespace Animation
{
namespace
{
constexpr char kOzzxMagic[8] = { 'O', 'Z', 'Z', 'X', 'P', 'A', 'C', 'K' };
}

static bool ReadFully(std::ifstream& stream, void* buffer, std::size_t size)
{
    stream.read(reinterpret_cast<char*>(buffer), static_cast<std::streamsize>(size));
    return static_cast<std::size_t>(stream.gcount()) == size;
}

static bool WriteFully(std::ofstream& stream, const void* buffer, std::size_t size)
{
    stream.write(reinterpret_cast<const char*>(buffer), static_cast<std::streamsize>(size));
    return static_cast<bool>(stream);
}

bool ReadOzzxBundle(const std::filesystem::path& path, OzzxBundle& out_bundle)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
    {
        std::cerr << "Failed to open .ozzx bundle: " << path << std::endl;
        return false;
    }

    char magic[sizeof(kOzzxMagic)] = {};
    if (!ReadFully(stream, magic, sizeof(magic)))
    {
        std::cerr << "Failed to read bundle magic from: " << path << std::endl;
        return false;
    }

    if (std::memcmp(magic, kOzzxMagic, sizeof(kOzzxMagic)) != 0)
    {
        std::cerr << "Unexpected bundle magic in: " << path << std::endl;
        return false;
    }

    std::uint32_t version = 0;
    std::uint32_t skeleton_size = 0;
    std::uint32_t mesh_size = 0;

    if (!ReadFully(stream, &version, sizeof(version)) ||
        !ReadFully(stream, &skeleton_size, sizeof(skeleton_size)) ||
        !ReadFully(stream, &mesh_size, sizeof(mesh_size)))
    {
        std::cerr << "Failed to read bundle header from: " << path << std::endl;
        return false;
    }

    if (version < 2)
    {
        std::cerr << "Unsupported .ozzx bundle version " << version << " in: " << path << std::endl;
        return false;
    }

    out_bundle.version = version;
    out_bundle.skeleton.resize(skeleton_size);
    out_bundle.mesh.resize(mesh_size);

    if (!ReadFully(stream, out_bundle.skeleton.data(), out_bundle.skeleton.size()))
    {
        std::cerr << "Failed to read skeleton payload from: " << path << std::endl;
        return false;
    }

    if (!ReadFully(stream, out_bundle.mesh.data(), out_bundle.mesh.size()))
    {
        std::cerr << "Failed to read mesh payload from: " << path << std::endl;
        return false;
    }

    out_bundle.motion_refs.clear();

    std::uint32_t motion_ref_count = 0;
    if (!ReadFully(stream, &motion_ref_count, sizeof(motion_ref_count)))
    {
        std::cerr << "Failed to read motion reference count from: " << path << std::endl;
        return false;
    }

    if (motion_ref_count > 0)
    {
        out_bundle.motion_refs.reserve(motion_ref_count);
        for (std::uint32_t index = 0; index < motion_ref_count; ++index)
        {
            std::uint32_t length = 0;
            if (!ReadFully(stream, &length, sizeof(length)))
            {
                std::cerr << "Failed to read motion reference length from: " << path << std::endl;
                return false;
            }

            xr_string entry;
            entry.resize(length, '\0');

            if (length > 0)
            {
                if (!ReadFully(stream, entry.data(), length))
                {
                    std::cerr << "Failed to read motion reference data from: " << path << std::endl;
                    return false;
                }
            }

            out_bundle.motion_refs.emplace_back(std::move(entry));
        }
    }

    return true;
}

bool WriteOzzxBundle(const std::filesystem::path& path, const OzzxBundle& bundle)
{
    if (bundle.skeleton.size() > std::numeric_limits<std::uint32_t>::max())
    {
        std::cerr << "Skeleton payload too large for .ozzx bundle: " << bundle.skeleton.size() << std::endl;
        return false;
    }

    if (bundle.mesh.size() > std::numeric_limits<std::uint32_t>::max())
    {
        std::cerr << "Mesh payload too large for .ozzx bundle: " << bundle.mesh.size() << std::endl;
        return false;
    }

    std::error_code ec;
    if (const auto parent = path.parent_path(); !parent.empty())
        std::filesystem::create_directories(parent, ec);

    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream)
    {
        std::cerr << "Failed to open bundle for writing: " << path << std::endl;
        return false;
    }

    if (!WriteFully(stream, kOzzxMagic, sizeof(kOzzxMagic)))
    {
        std::cerr << "Failed to write bundle magic: " << path << std::endl;
        return false;
    }

    const std::uint32_t version = 2;
    const std::uint32_t skeleton_size = static_cast<std::uint32_t>(bundle.skeleton.size());
    const std::uint32_t mesh_size = static_cast<std::uint32_t>(bundle.mesh.size());
    const std::uint32_t motion_ref_count = static_cast<std::uint32_t>(bundle.motion_refs.size());

    if (!WriteFully(stream, &version, sizeof(version)) ||
        !WriteFully(stream, &skeleton_size, sizeof(skeleton_size)) ||
        !WriteFully(stream, &mesh_size, sizeof(mesh_size)))
    {
        std::cerr << "Failed to write bundle header: " << path << std::endl;
        return false;
    }

    if (!bundle.skeleton.empty() && !WriteFully(stream, bundle.skeleton.data(), bundle.skeleton.size()))
    {
        std::cerr << "Failed to write skeleton payload: " << path << std::endl;
        return false;
    }

    if (!bundle.mesh.empty() && !WriteFully(stream, bundle.mesh.data(), bundle.mesh.size()))
    {
        std::cerr << "Failed to write mesh payload: " << path << std::endl;
        return false;
    }

    if (!WriteFully(stream, &motion_ref_count, sizeof(motion_ref_count)))
    {
        std::cerr << "Failed to write motion reference count: " << path << std::endl;
        return false;
    }

    for (const auto& reference : bundle.motion_refs)
    {
        const std::uint32_t length = static_cast<std::uint32_t>(reference.size());
        if (!WriteFully(stream, &length, sizeof(length)))
        {
            std::cerr << "Failed to write motion reference length: " << path << std::endl;
            return false;
        }

        if (length > 0)
        {
            if (!WriteFully(stream, reference.data(), length))
            {
                std::cerr << "Failed to write motion reference data: " << path << std::endl;
                return false;
            }
        }
    }

    return true;
}
} // namespace Animation
} // namespace XRay
