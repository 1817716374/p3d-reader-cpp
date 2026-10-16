#pragma once
#include <p3d/material_assignment_table.hpp>
namespace p3d::detail {
Json resolve_selected_assignment_rule(const Json &, const Json &,
                                      const NativeAssignmentMaterialContext &);
}
