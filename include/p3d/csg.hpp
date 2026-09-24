#pragma once
#include <p3d/reader.hpp>

namespace p3d {
// Decodes the persisted CSG archive used by command 58 and graphics entry type 10.
// Nodes are a flat array with explicit left/right indices; no Boolean evaluation
// or inference that a cached geometry is current is performed. Source bytes and
// archive-relative ranges remain available for unsupported data.
// Invalid outer lengths throw; invalid/unsupported node or geometry payloads are
// reported locally so that independent archive members remain accessible.
// The node budget is shared by nested archives; archive depth starts at zero.
Json decode_csg_bytes(const Bytes &bytes, std::size_t max_nodes = 100000,
                      std::size_t max_archive_depth = 32);
// Source selection for a restored node's getGeometriesFinal/getTransform
// accessors, before update or Boolean evaluation. Geometry objects themselves
// need not be constructible merely because their source indices are selected.
Json csg_node_geometry_input(const Json &archive, std::size_t node_index);
// Inputs of the native root-to-mesh conversion for the stored state. This does
// not update the tree, traverse operands as a mesh, or validate cached results.
Json csg_mesh_input(const Json &archive);
} // namespace p3d
