#include "internal.hpp"
#include "native_reference_sort.hpp"
#include <numeric>

namespace p3d {
namespace {
class Sort {
    const std::vector<NativeViewLinkSortEntry> &links;
    detail::NativeReferenceSortStats &stats;
    std::vector<std::size_t> order;
    bool less(std::size_t a, std::size_t b) {
        ++stats.comparisons;
        const auto &x = links[a], &y = links[b];
        require(x.present && y.present, "null_link_in_native_sort_comparison");
        require(x.native_kind && y.native_kind, "native_link_kind_required");
        if (*x.native_kind != *y.native_kind)
            return *x.native_kind < *y.native_kind;
        if (*x.native_kind != 2)
            return false;
        require(x.secondary_key && y.secondary_key, "native_link_secondary_key_required");
        return *x.secondary_key < *y.secondary_key;
    }
    bool at_less(std::size_t a, std::size_t b) {
        return less(order[a], order[b]);
    }
    void median3(std::size_t a, std::size_t b, std::size_t c) {
        if (at_less(b, a))
            std::swap(order[a], order[b]);
        if (at_less(c, b)) {
            std::swap(order[b], order[c]);
            if (at_less(b, a))
                std::swap(order[a], order[b]);
        }
    }
    std::pair<std::size_t, std::size_t> partition(std::size_t first, std::size_t last) {
        ++stats.partitions;
        const auto mid = first + (last - first) / 2, end = last - 1;
        if (end - first > 40) {
            ++stats.ninthers;
            const auto step = (last - first) / 8;
            median3(first, first + step, first + 2 * step);
            median3(mid - step, mid, mid + step);
            median3(end - 2 * step, end - step, end);
            median3(first + step, mid, end - step);
        } else
            median3(first, mid, end);
        auto pfirst = mid, plast = mid + 1;
        while (first < pfirst && !at_less(pfirst - 1, pfirst) && !at_less(pfirst, pfirst - 1))
            --pfirst;
        while (plast < last && !at_less(plast, pfirst) && !at_less(pfirst, plast))
            ++plast;
        auto gfirst = plast, glast = pfirst;
        for (;;) {
            for (; gfirst < last; ++gfirst) {
                if (at_less(pfirst, gfirst))
                    continue;
                if (at_less(gfirst, pfirst))
                    break;
                if (plast != gfirst)
                    std::swap(order[plast], order[gfirst]);
                ++plast;
            }
            for (; first < glast; --glast) {
                if (at_less(glast - 1, pfirst))
                    continue;
                if (at_less(pfirst, glast - 1))
                    break;
                --pfirst;
                if (pfirst != glast - 1)
                    std::swap(order[pfirst], order[glast - 1]);
            }
            if (glast == first && gfirst == last)
                return {pfirst, plast};
            if (glast == first) {
                ++stats.rotate_up;
                if (plast != gfirst)
                    std::swap(order[pfirst], order[plast]);
                ++plast;
                std::swap(order[pfirst++], order[gfirst++]);
            } else if (gfirst == last) {
                ++stats.rotate_down;
                --glast;
                --pfirst;
                if (glast != pfirst)
                    std::swap(order[glast], order[pfirst]);
                std::swap(order[pfirst], order[--plast]);
            } else
                std::swap(order[gfirst++], order[--glast]);
        }
    }
    void insertion(std::size_t first, std::size_t last) {
        for (auto next = first + 1; next < last; ++next) {
            const auto value = order[next];
            auto hole = next;
            if (less(value, order[first])) {
                while (hole > first) {
                    order[hole] = order[hole - 1];
                    --hole;
                }
            } else {
                while (hole > first && less(value, order[hole - 1])) {
                    order[hole] = order[hole - 1];
                    --hole;
                }
            }
            order[hole] = value;
        }
    }
    void adjust_heap(std::size_t first, std::size_t top, std::size_t count, std::size_t value) {
        auto hole = top;
        const auto bottom = (count - 1) / 2;
        while (hole < bottom) {
            auto child = 2 * hole + 2;
            if (at_less(first + child, first + child - 1))
                --child;
            order[first + hole] = order[first + child];
            hole = child;
        }
        if (hole == bottom && count % 2 == 0) {
            order[first + hole] = order[first + count - 1];
            hole = count - 1;
        }
        while (hole > top) {
            const auto parent = (hole - 1) / 2;
            if (!less(order[first + parent], value))
                break;
            order[first + hole] = order[first + parent];
            hole = parent;
        }
        order[first + hole] = value;
    }
    void heap(std::size_t first, std::size_t last) {
        ++stats.heap_ranges;
        auto count = last - first;
        for (auto parent = count / 2; parent > 0;) {
            --parent;
            adjust_heap(first, parent, count, order[first + parent]);
        }
        while (count >= 2) {
            --count;
            const auto value = order[first + count];
            order[first + count] = order[first];
            adjust_heap(first, 0, count, value);
        }
    }
    void sort(std::size_t first, std::size_t last, std::ptrdiff_t ideal) {
        while (last - first > 32) {
            if (ideal <= 0) {
                heap(first, last);
                return;
            }
            const auto middle = partition(first, last);
            ideal = (ideal / 2) + (ideal / 4);
            if (middle.first - first < last - middle.second) {
                sort(first, middle.first, ideal);
                first = middle.second;
            } else {
                sort(middle.second, last, ideal);
                last = middle.first;
            }
        }
        insertion(first, last);
    }

  public:
    Sort(const std::vector<NativeViewLinkSortEntry> &input, detail::NativeReferenceSortStats &s)
        : links(input), stats(s), order(input.size()) {
        std::iota(order.begin(), order.end(), 0);
    }
    std::vector<std::size_t> run(std::ptrdiff_t ideal) {
        sort(0, order.size(), ideal);
        return std::move(order);
    }
};
} // namespace
std::vector<std::size_t>
detail::native_reference_sort_indices(const std::vector<NativeViewLinkSortEntry> &links,
                                      NativeReferenceSortStats *stats,
                                      std::optional<std::ptrdiff_t> ideal) {
    require(links.size() <= static_cast<std::size_t>(PTRDIFF_MAX), "native_sort_list_too_large");
    NativeReferenceSortStats local;
    if (stats)
        *stats = {};
    Sort sorter(links, stats ? *stats : local);
    return sorter.run(ideal.value_or(static_cast<std::ptrdiff_t>(links.size())));
}

Json order_native_view_links(const std::vector<NativeViewLinkSortEntry> &links) {
    Json out = {{"status", "unresolved"}, {"scope", "ordinary_model_native_link_sort"}};
    try {
        auto order = detail::native_reference_sort_indices(links);
        out.update({{"status", "ordered"}, {"link_indices", std::move(order)}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json prepare_native_view_sequence(const Json &saved, const ViewSequenceContext &c,
                                  const std::vector<NativeViewLinkSortEntry> &inputs) {
    Json out = {{"status", "unresolved"}, {"scope", "reconcile_sort_then_default_view_sequence"}};
    try {
        require(c.links_complete, "complete_native_link_list_required");
        require(c.links.size() == inputs.size(), "native_link_sort_input_count_mismatch");
        for (std::size_t i = 0; i < inputs.size(); ++i)
            require(c.links[i].present == inputs[i].present, "native_link_sort_presence_mismatch");
        auto context = c;
        context.initialize_default = false;
        auto sequence = resolve_view_link_sequence(saved, context);
        out["reconciled_sequence"] = sequence;
        require(sequence.at("status") == "resolved",
                sequence.value("reason", std::string("view_reconciliation_unresolved")));
        auto sorted = order_native_view_links(inputs);
        out["link_sort"] = sorted;
        require(sorted.at("status") == "ordered",
                sorted.value("reason", std::string("native_link_sort_unresolved")));
        const auto order = sorted.at("link_indices").get<std::vector<std::size_t>>();
        if (!sequence.at("sequence_allocated").get<bool>() && !c.links.empty()) {
            for (std::size_t i = 0; i < order.size(); ++i)
                context.links[i] = c.links[order[i]];
            context.initialize_default = true;
            sequence = resolve_view_link_sequence(saved, context);
            require(
                sequence.at("status") == "resolved",
                sequence.value("reason", std::string("view_default_initialization_unresolved")));
            for (auto &entry : sequence["entries"])
                if (entry.at("source") == "model_link")
                    entry["source_index"] = order.at(entry.at("source_index").get<std::size_t>());
        }
        out.update({{"status", "resolved"}, {"sequence", std::move(sequence)}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
