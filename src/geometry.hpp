#pragma once
#include "internal.hpp"
namespace p3d {
Matrix4 identity();
Matrix4 multiply(const Matrix4 &, const Matrix4 &);
Point3 transform(const Matrix4 &, const Point3 &);
Point3 vector_transform(const Matrix4 &, const Point3 &);
double determinant(const Matrix4 &);
bool reverses_winding(const Matrix4 &);
Matrix4 instance_transform(const Bytes &);
Geometry reconstruct(const Json &, const Tessellation &);
Geometry reconstruct_native(const Json &, const Tessellation &);
void merge_geometry(Geometry &, const Geometry &, const Matrix4 &, bool text_parent = true);
void merge_mesh_channels(Geometry &, const Geometry &, std::size_t first_face);
Json geometry_json(const Geometry &);
NativeScene build_native_scene(const Document &, const Tessellation &, unsigned);
} // namespace p3d
