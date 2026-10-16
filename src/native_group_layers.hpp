#pragma once
#include "internal.hpp"
namespace p3d {
struct NativeGroupLayerQueries {
    Json data;
    std::map<std::string, std::size_t> by_source;
    NativeGroupLayerQueries(const Json &records, const Json &containers,
                            const std::vector<std::size_t> &targets);
};
} // namespace p3d
