#pragma once
#include <p3d/view_sequence.hpp>
namespace p3d::detail {
struct NativeReferenceSortStats {
    std::size_t comparisons = 0, partitions = 0, ninthers = 0, heap_ranges = 0;
    std::size_t rotate_up = 0, rotate_down = 0;
};
// The optional ideal argument is the native internal depth budget. The public
// loading entry always initializes it to the full link count.
std::vector<std::size_t>
native_reference_sort_indices(const std::vector<NativeViewLinkSortEntry> &links,
                              NativeReferenceSortStats *stats = nullptr,
                              std::optional<std::ptrdiff_t> ideal = {});
} // namespace p3d::detail
