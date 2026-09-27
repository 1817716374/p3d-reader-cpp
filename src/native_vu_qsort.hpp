#pragma once
#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>
namespace p3d::swept_detail {
// Pointer-array exchanges used by the native VU lexical sort. The 8-element
// selection branch and equal-key partition are significant for source order.
// Compare returns a negative, zero or positive value, and owns work charging.
template <class Compare> void native_vu_qsort(std::vector<std::size_t> &values, Compare compare) {
    using Index = std::ptrdiff_t;
    if (values.size() < 2)
        return;
    auto cmp = [&](Index a, Index b) { return compare(values[a], values[b]); };
    auto swap = [&](Index a, Index b) { std::swap(values[a], values[b]); };
    std::vector<std::pair<Index, Index>> pending;
    Index lo = 0, hi = static_cast<Index>(values.size()) - 1;
    for (;;) {
        if (hi - lo + 1 <= 8) {
            for (auto end = hi; end > lo; --end) {
                auto largest = lo;
                for (auto p = lo + 1; p <= end; ++p)
                    if (cmp(p, largest) > 0)
                        largest = p;
                swap(largest, end);
            }
        } else {
            auto mid = lo + (hi - lo + 1) / 2;
            if (cmp(lo, mid) > 0)
                swap(lo, mid);
            if (cmp(lo, hi) > 0)
                swap(lo, hi);
            if (cmp(mid, hi) > 0)
                swap(mid, hi);
            auto left = lo, right = hi;
            for (;;) {
                if (mid > left) {
                    do {
                        ++left;
                    } while (left < mid && cmp(left, mid) <= 0);
                }
                if (mid <= left) {
                    do {
                        ++left;
                    } while (left <= hi && cmp(left, mid) <= 0);
                }
                do {
                    --right;
                } while (right > mid && cmp(right, mid) > 0);
                if (right < left)
                    break;
                swap(left, right);
                if (mid == right)
                    mid = left;
            }
            ++right;
            if (mid < right) {
                do {
                    --right;
                } while (right > mid && cmp(right, mid) == 0);
            }
            if (mid >= right) {
                do {
                    --right;
                } while (right > lo && cmp(right, mid) == 0);
            }
            // Process the smaller partition first, retaining the native tie.
            if (right - lo >= hi - left) {
                if (lo < right)
                    pending.emplace_back(lo, right);
                if (left < hi) {
                    lo = left;
                    continue;
                }
            } else {
                if (left < hi)
                    pending.emplace_back(left, hi);
                if (lo < right) {
                    hi = right;
                    continue;
                }
            }
        }
        if (pending.empty())
            break;
        lo = pending.back().first;
        hi = pending.back().second;
        pending.pop_back();
    }
}
} // namespace p3d::swept_detail
