#pragma once
#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

namespace p3d {
// The native attribute reader sorts pointers. Sorting source ordinals preserves
// the same exchanges without rearranging or duplicating the attribute payloads.
template <class Less> class NativeAttributeSort {
    std::vector<std::size_t> &order;
    Less less;

    void median_three(std::size_t a, std::size_t b, std::size_t c) {
        if (less(order[b], order[a]))
            std::swap(order[a], order[b]);
        if (less(order[c], order[b])) {
            std::swap(order[b], order[c]);
            if (less(order[b], order[a]))
                std::swap(order[a], order[b]);
        }
    }
    std::pair<std::size_t, std::size_t> partition(std::size_t first, std::size_t last) {
        auto low = first + (last - first) / 2;
        const auto end = last - 1;
        if (end - first > 40) {
            const auto step = (end - first + 1) / 8;
            median_three(first, first + step, first + 2 * step);
            median_three(low - step, low, low + step);
            median_three(end - 2 * step, end - step, end);
            median_three(first + step, low, end - step);
        } else {
            median_three(first, low, end);
        }
        auto high = low + 1;
        auto equal = [&](std::size_t a, std::size_t b) {
            return !less(order[a], order[b]) && !less(order[b], order[a]);
        };
        while (low > first && equal(low - 1, low))
            --low;
        while (high < last && equal(high, low))
            ++high;
        auto left = low, right = high;
        for (;;) {
            while (right < last) {
                if (less(order[low], order[right])) {
                    ++right;
                    continue;
                }
                if (less(order[right], order[low]))
                    break;
                if (high != right)
                    std::swap(order[high], order[right]);
                ++high;
                ++right;
            }
            while (left > first) {
                const auto previous = left - 1;
                if (less(order[previous], order[low])) {
                    --left;
                    continue;
                }
                if (less(order[low], order[previous]))
                    break;
                --low;
                if (low != previous)
                    std::swap(order[low], order[previous]);
                --left;
            }
            if (left == first && right == last)
                return {low, high};
            if (left == first) {
                if (high != right)
                    std::swap(order[low], order[high]);
                ++high;
                std::swap(order[low], order[right]);
                ++low;
                ++right;
            } else if (right == last) {
                --left;
                --low;
                if (left != low)
                    std::swap(order[left], order[low]);
                --high;
                std::swap(order[low], order[high]);
            } else {
                --left;
                std::swap(order[left], order[right]);
                ++right;
            }
        }
    }
    void sift(std::size_t first, std::size_t hole, std::size_t count, std::size_t value) {
        const auto top = hole;
        const auto half = (count - 1) / 2;
        while (hole < half) {
            auto child = 2 * hole + 2;
            // Equality chooses the right child, which affects duplicate order.
            if (less(order[first + child], order[first + child - 1]))
                --child;
            order[first + hole] = order[first + child];
            hole = child;
        }
        if (hole == half && count % 2 == 0) {
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

  public:
    std::size_t partition_count = 0, heap_count = 0;
    NativeAttributeSort(std::vector<std::size_t> &indices, Less compare)
        : order(indices), less(std::move(compare)) {}
    void sort(std::size_t first, std::size_t last, std::size_t ideal) {
        while (last - first > 32) {
            if (ideal == 0) {
                ++heap_count;
                auto count = last - first;
                for (auto parent = count / 2; parent > 0;) {
                    --parent;
                    sift(first, parent, count, order[first + parent]);
                }
                while (count >= 2) {
                    --count;
                    const auto value = order[first + count];
                    order[first + count] = order[first];
                    sift(first, 0, count, value);
                }
                return;
            }
            ++partition_count;
            const auto equal_range = partition(first, last);
            ideal = ideal / 2 + ideal / 4;
            if (equal_range.first - first < last - equal_range.second) {
                sort(first, equal_range.first, ideal);
                first = equal_range.second;
            } else {
                sort(equal_range.second, last, ideal);
                last = equal_range.first;
            }
        }
        for (auto next = first; next < last; ++next) {
            const auto value = order[next];
            auto hole = next;
            while (hole > first && less(value, order[hole - 1])) {
                order[hole] = order[hole - 1];
                --hole;
            }
            order[hole] = value;
        }
    }
};
} // namespace p3d
