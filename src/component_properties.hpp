#pragma once
#include "internal.hpp"
namespace p3d {
Json decode_component_properties(
    const Bytes &, const std::vector<NarrowStringDecoder> & = {});
Json decode_data_unit(const Bytes &, const std::vector<NarrowStringDecoder> & = {});
} // namespace p3d
