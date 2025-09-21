#pragma once

#include "xrCore/_matrix.h"

#include <ozz/base/maths/simd_math.h>

namespace XRay
{
namespace Animation
{
inline Fvector3 ConvertOzzVectorToXRay(float x, float y, float z)
{
    Fvector3 result;
    result.x = x;
    result.y = y;
    result.z = -z;
    return result;
}

inline Fmatrix ConvertOzzMatrixToXRay(const ozz::math::Float4x4& matrix)
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
} // namespace Animation
} // namespace XRay
