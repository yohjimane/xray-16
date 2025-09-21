#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

namespace XRay
{
namespace Animation
{
struct OzzxBundle
{
    std::uint32_t version = 1;
    std::vector<std::uint8_t> skeleton;
    std::vector<std::uint8_t> mesh;
};

bool ReadOzzxBundle(const std::filesystem::path& path, OzzxBundle& out_bundle);
bool WriteOzzxBundle(const std::filesystem::path& path, const OzzxBundle& bundle);
} // namespace Animation
} // namespace XRay
