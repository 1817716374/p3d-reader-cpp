#pragma once
#include <cstdint>
namespace p3d {
inline std::uint32_t native_line_style_local_kind(std::uint32_t resource) {
    switch (resource) {
    case 0:
        return 6;
    case 2001:
        return 2;
    case 2002:
        return 3;
    case 2003:
        return 4;
    case 2004:
    case 2005:
        return 1;
    default:
        return resource;
    }
}
} // namespace p3d
