#pragma once
#include "internal.hpp"
namespace p3d {
Json application_blob(const std::string &, const Bytes &, const std::string &);
Json complex_blob(const std::string &, const Bytes &);
Json binary_json(const Bytes &);
Json binary_xml(const Bytes &);
Json profile(const Bytes &, unsigned = 0);
Json boolean_record(const Bytes &);
} // namespace p3d
