#pragma once
#include <p3d/reader.hpp>
#include <set>
namespace p3d {
Json append_tree_entity_references(const Json &resources, const std::string &node,
                                   const std::set<std::string> &live, const Json &binding_index,
                                   Json &references);
}
