#pragma once
#include <p3d/reader.hpp>

namespace p3d {
// One complete native model-header record, including its four-byte stream prefix.
// Projects initial main-file acquisition only: version 8 is copied; version 7
// is supported with positive finite scales and an exact identity orientation.
// Other legacy repair paths return not_evaluated without a partial loaded header.
// Does not load entities, resolve host callbacks, or change the source record.
Json initial_native_model_header_input(const Bytes &source_record);
} // namespace p3d
