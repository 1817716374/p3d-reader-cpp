#pragma once
#include <p3d/reader.hpp>

namespace p3d {
// Decodes the persisted CSG archive used by command 58 and graphics entry type 10.
// Nodes are a flat array with explicit left/right indices; no Boolean evaluation
// or inference that a cached geometry is current is performed. Source bytes and
// archive-relative ranges remain available for unsupported data.
// Invalid outer lengths throw; invalid/unsupported node or geometry payloads are
// reported locally so that independent archive members remain accessible.
Json decode_csg_bytes(const Bytes &bytes, std::size_t max_nodes = 100000);
} // namespace p3d
