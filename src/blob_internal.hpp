#pragma once
#include "internal.hpp"
namespace p3d {
Json application_blob(const std::string &, const Bytes &, const std::string &);
Json complex_blob(const std::string &, const Bytes &);
Json decode_bfa_formula_expression(const Bytes &encoded);
void bind_bfa_entity_sources(Json &binary_fields, const Json &objects);
void bind_bfa_definition_sources(Json &binary_fields, const Json &objects);
Json binary_json(const Bytes &);
Json binary_xml(const Bytes &);
Json profile(const Bytes &, unsigned = 0);
Json boolean_record(const Bytes &);
} // namespace p3d
