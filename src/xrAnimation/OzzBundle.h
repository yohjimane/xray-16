#pragma once

#include <cstdint>
#include <filesystem>

#include <vector>

#include "xrCommon/xr_string.h"
#include "xrCommon/xr_vector.h"

namespace XRay
{
namespace Animation
{
struct OzzxBundle
{
    std::uint32_t version = 1;
    std::vector<std::uint8_t> skeleton;
    std::vector<std::uint8_t> mesh;
    xr_vector<xr_string> motion_refs;
};

bool ReadOzzxBundle(const std::filesystem::path& path, OzzxBundle& out_bundle);
bool WriteOzzxBundle(const std::filesystem::path& path, const OzzxBundle& bundle);
} // namespace Animation
} // namespace XRay
