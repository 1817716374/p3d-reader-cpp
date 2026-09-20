#pragma once
#include <p3d/reference_search.hpp>
namespace p3d::detail {
std::int32_t reference_object_kind(const ReferenceSearchModel &model);
bool reference_object_present(const ReferenceSearchModel &model);
Json search_reference_descendants_impl(const ReferenceSearchQuery &query,
                                       const std::vector<ReferenceSearchModel> &models,
                                       bool start_known, std::optional<std::size_t> start);
} // namespace p3d::detail
