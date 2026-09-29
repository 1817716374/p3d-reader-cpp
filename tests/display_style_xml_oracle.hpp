#pragma once
#include <cstdint>

// Recorded by tools/probe_style_xml.py against unmodified R1.18 p3dlibxml2.dll.
// SHA256: ca24cc124168dd90958f79e72d2af9dbca1dd937338ce6dd07fb0500cf21270a
// Synthetic inputs and observed outputs; no proprietary file contents.
struct StyleXmlOracleCase {
    const char *source;
    unsigned bool_status; bool boolean;
    unsigned u32_status; std::uint32_t u32;
    unsigned u64_status; std::uint64_t u64;
};
inline constexpr StyleXmlOracleCase style_xml_oracle[] = {
    {nullptr, 4100, false, 4100, 0u, 4100, 0ull},
    {"", 4101, false, 4101, 0u, 4101, 0ull},
    {"true", 0, true, 4101, 0u, 4101, 0ull},
    {"false", 0, false, 4101, 0u, 4101, 0ull},
    {"True", 0, true, 4101, 0u, 4101, 0ull},
    {"TRUE", 0, true, 4101, 0u, 4101, 0ull},
    {"False", 0, false, 4101, 0u, 4101, 0ull},
    {"1", 4101, false, 0, 1u, 0, 1ull},
    {"0", 4101, false, 0, 0u, 0, 0ull},
    {" true", 4101, false, 4101, 0u, 4101, 0ull},
    {"true ", 4101, false, 4101, 0u, 4101, 0ull},
    {"yes", 4101, false, 4101, 0u, 4101, 0ull},
    {"-1", 4101, false, 0, 4294967295u, 0, 18446744073709551615ull},
    {"+2", 4101, false, 0, 2u, 0, 2ull},
    {"010", 4101, false, 0, 10u, 0, 10ull},
    {"0x10", 4101, false, 0, 0u, 0, 0ull},
    {"42tail", 4101, false, 0, 42u, 0, 42ull},
    {" 42 ", 4101, false, 0, 42u, 0, 42ull},
    {"4294967296", 4101, false, 0, 0u, 0, 4294967296ull},
    {"3.5", 4101, false, 0, 3u, 0, 3ull},
    {"nan", 4101, false, 4101, 0u, 4101, 0ull},
    {"INF", 4101, false, 4101, 0u, 4101, 0ull},
    {"1e3", 4101, false, 0, 1u, 0, 1ull},
    {"1,5", 4101, false, 0, 1u, 0, 1ull},
    {"-4294967296", 4101, false, 0, 0u, 0, 18446744069414584320ull},
    {"4294967297", 4101, false, 0, 1u, 0, 4294967297ull},
    {"18446744073709551615", 4101, false, 0, 4294967295u, 0, 18446744073709551615ull},
    {"18446744073709551616", 4101, false, 0, 4294967295u, 0, 18446744073709551615ull},
    {"18446744073709551617", 4101, false, 0, 4294967295u, 0, 18446744073709551615ull},
    {"-18446744073709551616", 4101, false, 0, 4294967295u, 0, 18446744073709551615ull},
    {"9999999999999999999999999999999999999", 4101, false, 0, 4294967295u, 0, 18446744073709551615ull},
    {"1e", 4101, false, 0, 1u, 0, 1ull},
    {"1e+", 4101, false, 0, 1u, 0, 1ull},
    {"0x", 4101, false, 0, 0u, 0, 0ull},
    {"0x1p", 4101, false, 0, 0u, 0, 0ull},
    {"0x1p+", 4101, false, 0, 0u, 0, 0ull},
    {"+", 4101, false, 4101, 0u, 4101, 0ull},
    {"-", 4101, false, 4101, 0u, 4101, 0ull},
    {"--1", 4101, false, 4101, 0u, 4101, 0ull},
    {"1e999", 4101, false, 0, 1u, 0, 1ull},
    {"1e-999", 4101, false, 0, 1u, 0, 1ull},
    {"nan(foo)", 4101, false, 4101, 0u, 4101, 0ull},
    {"infinity", 4101, false, 4101, 0u, 4101, 0ull},
    {"infinite", 4101, false, 4101, 0u, 4101, 0ull},
    {"FALSE", 0, false, 4101, 0u, 4101, 0ull},
    {"TrUe", 0, true, 4101, 0u, 4101, 0ull},
    {"\ttrue", 4101, false, 4101, 0u, 4101, 0ull},
    {"true\n", 4101, false, 4101, 0u, 4101, 0ull},
};
