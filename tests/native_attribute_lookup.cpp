#include "internal.hpp"
#include "native_attribute_sort.hpp"
#include <numeric>
#include <random>

unsigned native_attribute_lookup_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto attribute = [](unsigned group, unsigned key, std::uint32_t index = 0) {
        return Json{{"group", group}, {"key", key}, {"index", index}};
    };
    auto rows = Json::array();
    for (unsigned n : {0u, 1u, 5u, 6u, 32u, 33u, 41u, 42u, 127u}) {
        rows = Json::array();
        for (unsigned i = 0; i < n; ++i)
            rows.push_back(attribute(0, 20014));
        auto result = native_attribute_lookup(rows);
        check(result["status"] == "resolved" && result["sorted_source_ordinals"].size() == n,
              "empty and equal-key collections remain valid across algorithm thresholds");
        if (n) {
            check(result["keys"][0]["selected_source_ordinal"] == (n <= 5 ? n - 1 : 0),
                  "reverse small scan and lower-bound large scan choose different equal-key ends");
        }
    }
    rows = Json::array({attribute(0xffff, 0), attribute(0, 1), attribute(1, 0),
                        attribute(0, 0xffff), attribute(0, 1, 0xffffffff), attribute(0, 1, 1)});
    auto result = native_attribute_lookup(rows);
    check(result["sorted_source_ordinals"] == Json::array({2, 0, 1, 5, 4, 3}),
          "combined key sorts unsigned key before group and index remains unsigned");
    rows = Json::array();
    rows.push_back(attribute(0, 1));
    for (unsigned i = 1; i < 33; ++i)
        rows.push_back(attribute(0, 0));
    result = native_attribute_lookup(rows);
    check(result["keys"][0]["selected_source_ordinal"] == 16 &&
              result["sorted_source_ordinals"][16] == 32,
          "33-entry median exchanges change duplicate identity before lower-bound lookup");
    rows.erase(rows.end() - 1);
    check(native_attribute_lookup(rows)["keys"][0]["selected_source_ordinal"] == 1,
          "32-entry insertion sort retains first matching source identity");
    for (unsigned n : {41u, 42u}) {
        rows = Json::array({attribute(0, 1)});
        for (unsigned i = 1; i < n; ++i)
            rows.push_back(attribute(0, 0));
        check(native_attribute_lookup(rows)["keys"][0]["selected_source_ordinal"] ==
                  (n == 41 ? 20 : 5),
              "pivot switches from three entries to nine samples above 41 entries");
    }
    rows = Json::array();
    for (unsigned i = 0; i < 33; ++i)
        rows.push_back(attribute(0, i % 2));
    result = native_attribute_lookup(rows);
    Json zero_order = Json::array({0, 18, 2, 20, 4, 22, 6, 24, 8, 26, 10, 28, 12, 30, 14, 32, 16});
    check(result["keys"][0]["matching_source_ordinals"] == zero_order,
          "partition rotation preserves the native duplicate exchange sequence");
    rows[0]["group"] = 65536;
    check(native_attribute_lookup(rows)["status"] == "unresolved",
          "out-of-width fields are rejected rather than silently truncated");

    // Check permutation and unsigned ordering separately from lookup grouping,
    // including direct exercise of the introsort heap fallback.
    std::mt19937 rng(793180);
    for (std::size_t n : {1u, 2u, 31u, 32u, 33u, 41u, 42u, 63u, 64u, 65u, 127u, 257u, 4096u}) {
        for (unsigned pattern = 0; pattern < 5; ++pattern) {
            std::vector<std::uint32_t> values(n);
            for (std::size_t i = 0; i < n; ++i) {
                if (pattern == 0)
                    values[i] = rng() % 11;
                if (pattern == 1)
                    values[i] = static_cast<std::uint32_t>(n - i);
                if (pattern == 2)
                    values[i] = static_cast<std::uint32_t>(std::min(i, n - i));
                if (pattern == 3)
                    values[i] = static_cast<std::uint32_t>(i % 2);
                if (pattern == 4)
                    values[i] = 0xffffffffu;
            }
            for (auto ideal : {n, std::size_t(0)}) {
                std::vector<std::size_t> order(n);
                std::iota(order.begin(), order.end(), 0);
                auto less = [&](std::size_t a, std::size_t b) { return values[a] < values[b]; };
                NativeAttributeSort<decltype(less)> sorter(order, less);
                sorter.sort(0, n, ideal);
                check(std::is_sorted(order.begin(), order.end(), less),
                      "native introsort and heap fallback produce unsigned key order");
                auto permutation = order;
                std::sort(permutation.begin(), permutation.end());
                bool complete = true;
                for (std::size_t i = 0; i < n; ++i)
                    complete &= permutation[i] == i;
                check(complete, "native sort never loses or duplicates an attribute identity");
                if (n > 32 && ideal == 0)
                    check(sorter.heap_count == 1, "heap fallback is exercised");
                if (n > 32 && ideal == n)
                    check(sorter.partition_count > 0, "partition path is exercised");
            }
        }
    }
    Json member = {{"input_occurrence_index", 20}};
    Json input = {{"status", "resolved"}, {"attachments", Json::array()}};
    check(native_material_attribute(member, input)["status"] == "absent",
          "fully loaded member without collection has no material attribute");
    input["status"] = "partial";
    check(native_material_attribute(member, input)["status"] == "unresolved",
          "incomplete input cannot prove absent material attribute");
    rows = Json::array({attribute(0, 20014), attribute(0, 20014)});
    input["attachments"].push_back(
        {{"target", {{"input_occurrence_index", 20}}}, {"lookup", native_attribute_lookup(rows)}});
    check(native_material_attribute(member, input) ==
              Json({{"status", "selected"}, {"attachment_index", 0}, {"source_ordinal", 1}}),
          "material member uses exact native duplicate selection on attached collection");
    rows.push_back(attribute(0, 20014, 1));
    input["attachments"][0]["lookup"] = native_attribute_lookup(rows);
    check(native_material_attribute(member, input)["source_ordinal"] == 1,
          "same material key with a different index is not selected");
    rows[0]["key"] = 42;
    rows[1]["key"] = 42;
    input["attachments"][0]["lookup"] = native_attribute_lookup(rows);
    check(native_material_attribute(member, input)["reason"] == "material_attribute_key_not_found",
          "material exact index zero is required");
    return checks;
}
