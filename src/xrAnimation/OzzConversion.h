#pragma once

#include "xrCore/_matrix.h"

#include <ozz/base/maths/simd_math.h>

#include <array>

namespace XRay
{
namespace Animation
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

inline Matrix4 Multiply(const Matrix4& lhs, const Matrix4& rhs)
{
    Matrix4 result{};
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            float value = 0.f;
            for (int k = 0; k < 4; ++k)
                value += lhs[static_cast<size_t>(row)][static_cast<size_t>(k)] *
                         rhs[static_cast<size_t>(k)][static_cast<size_t>(col)];
            result[static_cast<size_t>(row)][static_cast<size_t>(col)] = value;
        }
    }
    return result;
}

inline Matrix4 ChangeBasis(const Matrix4& matrix, const Matrix4& basis, const Matrix4& basis_inverse)
{
    return Multiply(Multiply(basis, matrix), basis_inverse);
}

inline Matrix4 LoadOzzMatrix(const ozz::math::Float4x4& matrix)
{
    Matrix4 result{};
    for (int col = 0; col < 4; ++col)
    {
        float column[4];
        ozz::math::StorePtrU(matrix.cols[col], column);
        for (int row = 0; row < 4; ++row)
            result[static_cast<size_t>(row)][static_cast<size_t>(col)] = column[row];
    }
    return result;
}

inline Fmatrix ToFmatrix(const Matrix4& matrix)
{
    Fmatrix result;
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            result.m[col][row] = matrix[static_cast<size_t>(row)][static_cast<size_t>(col)];
    return result;
}

inline std::array<float, 3> ApplyBasis(const Matrix4& basis, const std::array<float, 3>& vector)
{
    std::array<float, 3> result{};
    for (int row = 0; row < 3; ++row)
    {
        result[static_cast<size_t>(row)] = basis[static_cast<size_t>(row)][0] * vector[0] +
            basis[static_cast<size_t>(row)][1] * vector[1] +
            basis[static_cast<size_t>(row)][2] * vector[2];
    }
    return result;
}
} // namespace detail

inline Fmatrix ConvertOzzMatrixToXRay(const ozz::math::Float4x4& matrix)
{
    const auto ozz = detail::LoadOzzMatrix(matrix);
    const auto xray = detail::ChangeBasis(ozz, detail::kOzzToXray, detail::kXrayToOzz);
    return detail::ToFmatrix(xray);
}
} // namespace Animation
} // namespace XRay
