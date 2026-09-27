#pragma once
#include "native_tube.hpp"

namespace p3d::swept_detail {
struct NativeBuilderCoordinateBatch {
    std::vector<Point3> points, normals;
    std::vector<Point2> parameters;
    double parameter_scope = 0;
    bool normalize_normals = true, reverse_normals = false;
};
struct NativeBuilderCoordinateOptions {
    double point_absolute_tolerance = 1e-14, point_relative_tolerance = 1e-12;
    double parameter_absolute_tolerance = 1e-14, parameter_relative_tolerance = 1e-12;
};
struct NativeBuilderCoordinateRemap {
    std::vector<std::size_t> points, normals, parameters;
};
struct NativeBuilderCoordinates {
    std::vector<Point3> points, normals;
    std::vector<Point2> parameters;
    std::vector<NativeBuilderCoordinateRemap> batches;
    Json report;
};
// Explicit native derived-mesh coordinate insertion, in batch/source order.
// Inputs must already have the builder's placement applied. Normals may be
// reversed and normalized as by FindOrAddNormal; disabling normalization is
// the lower-level coordinate-map entry, which stores the supplied vectors.
// Parameter scope is the native map's third key coordinate, not stored UV.
// This does not add facets, synthesize attributes or assemble an entire solid.
NativeBuilderCoordinates
map_native_builder_coordinates(const std::vector<NativeBuilderCoordinateBatch> &,
                               const NativeBuilderCoordinateOptions &, TubeBudget &);
} // namespace p3d::swept_detail
