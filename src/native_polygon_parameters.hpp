#pragma once
#include "native_builder_polyface.hpp"

namespace p3d::swept_detail {
struct NativePolygonParameterMapping {
    std::array<Point2, 2> distance_range{}, parameter_range{};
    Matrix4 transform{};
    bool transform_applied = false;
};
// Native pseudo-distance range mapping. Modes 0/1/2 select independent unit,
// common unit, and distance ranges; other values retain the input coordinates.
// A false transform result still carries the native output ranges.
NativePolygonParameterMapping map_native_polygon_parameter_range(const std::array<Point2, 2> &,
                                                                 std::uint32_t mode,
                                                                 Point2 distance_factors);

struct NativePolygonParameters {
    std::vector<Point2> projected, parameters;
    std::optional<NativePolygonParameterMapping> mapping;
    // The original caller updates only this member of the current face record.
    // Empty/singleton input reads an unwritten native range; do not invent one.
    std::optional<std::array<Point2, 2>> face_distance_range;
    bool complete = false;
    Json report;
};
// AddTriangulation UV route on its final point array, including source slots
// unused by facets and appended intersection points. No pooling or index
// remapping occurs here. The frame is supplied by the original caller.
NativePolygonParameters prepare_native_polygon_parameters(const std::vector<Point3> &,
                                                          const Matrix4 &world_to_local,
                                                          std::uint32_t mode, TubeBudget &);
} // namespace p3d::swept_detail
