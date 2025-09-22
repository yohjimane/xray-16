#pragma once

#include <cstdint>
#include <filesystem>

#include "xrCommon/xr_vector.h"

namespace XRay
{
namespace Animation
{
struct OzzxBundle
{
    std::uint32_t version = 1;
    xr_vector<std::uint8_t> skeleton;
    xr_vector<std::uint8_t> mesh;
};

bool ReadOzzxBundle(const std::filesystem::path& path, OzzxBundle& out_bundle);
bool WriteOzzxBundle(const std::filesystem::path& path, const OzzxBundle& bundle);
} // namespace Animation
} // namespace XRay
