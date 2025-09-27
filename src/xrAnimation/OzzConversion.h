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

inline Fmatrix CopyOzzMatrixToFMatrix(const ozz::math::Float4x4& matrix)
{
    float column[4];

    Fmatrix out;
    out.identity();

    ozz::math::StorePtrU(matrix.cols[0], column);
    out.i.set(column[0], column[1], column[2]);
    out._14 = column[3];

    ozz::math::StorePtrU(matrix.cols[1], column);
    out.j.set(column[0], column[1], column[2]);
    out._24 = column[3];

    ozz::math::StorePtrU(matrix.cols[2], column);
    out.k.set(column[0], column[1], column[2]);
    out._34 = column[3];

    ozz::math::StorePtrU(matrix.cols[3], column);
    out.c.set(column[0], column[1], column[2]);
    out._44 = column[3];

    return out;
}
inline Fmatrix ConvertOzzMatrixToXRay(const ozz::math::Float4x4& matrix)
{
    return CopyOzzMatrixToFMatrix(matrix);
}
} // namespace Animation
} // namespace XRay
