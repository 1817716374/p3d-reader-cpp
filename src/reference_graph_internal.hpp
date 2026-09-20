#pragma once
#include <p3d/reference_search.hpp>
namespace p3d::detail {
std::int32_t reference_object_kind(const ReferenceSearchModel &model);
bool reference_object_present(const ReferenceSearchModel &model);
std::optional<std::size_t>
reference_connected_root_impl(const std::vector<ReferenceSearchModel> &models,
                              std::optional<std::size_t> index);
Json match_reference_model_impl(const ReferenceSearchQuery &query,
                                const std::vector<ReferenceSearchModel> &models,
                                std::optional<std::size_t> index);
Json search_reference_descendants_impl(const ReferenceSearchQuery &query,
                                       const std::vector<ReferenceSearchModel> &models,
                                       bool start_known, std::optional<std::size_t> start);
} // namespace p3d::detail
