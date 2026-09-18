#include "internal.hpp"

unsigned reference_target_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool condition, const char *message) {
        ++checks;
        require(condition, message);
    };
    auto put = [](Bytes &b, std::size_t at, std::uint64_t value, unsigned n) {
        for (unsigned i = 0; i < n; ++i)
            b.at(at + i) = std::uint8_t(value >> (8 * i));
    };
    auto link = [&](unsigned app, Bytes payload) {
        if (payload.size() % 2)
            payload.push_back(0);
        auto words = (payload.size() + 4) / 2;
        unsigned header = 0x1000;
        if (words <= 256)
            header |= unsigned(words - 1);
        else {
            unsigned shift = 1;
            while ((words + (std::size_t(1) << shift) - 1) >> shift > 255)
                ++shift;
            const auto mantissa = (words + (std::size_t(1) << shift) - 1) >> shift;
            words = mantissa << shift;
            payload.resize(words * 2 - 4);
            header |= 0x4000 | (shift << 8) | unsigned(mantissa);
        }
        return Json{
            {"header", header}, {"app", app}, {"offset", 372}, {"payload", rawbytes(payload)}};
    };
    auto text_link = [&](unsigned key, Bytes text) {
        Bytes p(8 + text.size());
        put(p, 0, key, 2);
        put(p, 4, text.size(), 4);
        std::copy(text.begin(), text.end(), p.begin() + 8);
        return link(0x56d2, p);
    };
    auto id_link = [&](unsigned version, std::uint32_t id) {
        Bytes p(8);
        put(p, 0, version, 2);
        put(p, 2, 0x1234, 2);
        put(p, 4, id, 4);
        return link(0x5717, p);
    };
    auto input = [](std::uint32_t flags) {
        return Json{{"status", "decoded"}, {"origin_inputs", {{"secondary_flags", flags}}}};
    };
    auto query = [&](std::uint32_t flags, Json links) {
        return native_reference_target(input(flags), links);
    };
    auto name = text_link(21, Bytes{'M', 'o', 'd', 'e', 'l'});
    auto alternate = text_link(36, Bytes{'A', 'l', 't'});
    auto q = query(0, Json::array({name, alternate}));
    check(q["model_selection"]["kind"] == "model_name" && q["model_selection"]["name"] == "Model",
          "primary persisted model name selection");
    check(query(0x20000, Json::array({name, alternate}))["model_selection"]["name"] == "Alt",
          "secondary flag selects alternate model-name slot");
    q = query(0x20000, Json::array({name}));
    check(q["model_selection"]["kind"] == "file_default_model" &&
              q["model_selection"]["source"] == "empty_model_name" &&
              q["model_selection"]["clear_requested_load_mask"] == false,
          "absent alternate name does not fall back to primary name");
    q = query(0, Json::array({name, id_link(0, 0xfffffffeu)}));
    check(q["model_selection"]["kind"] == "model_id" &&
              q["model_selection"]["model_id"] == 0xfffffffeu,
          "explicit model ID precedes name and keeps its complete uint32 bits");
    q = query(0x28000, Json::array({name, alternate, id_link(0, 9)}));
    check(q["model_selection"]["kind"] == "file_default_model" &&
              q["model_selection"]["clear_requested_load_mask"] == true,
          "default-model flag precedes explicit ID and clears requested load mask");
    q = query(0, Json::array({id_link(1, 99), id_link(0, 7), id_link(0, 8)}));
    check(q["model_selection"]["model_id"] == 7 &&
              q["model_id_link"]["selected_linkage_index"] == 1 &&
              q["model_id_link"]["skipped_versions"].size() == 1 &&
              q["model_id_link"]["reserved_word"] == 0x1234,
          "ID reader skips unsupported versions but selects first version zero");
    auto short_id = link(0x5717, Bytes(2));
    q = query(0, Json::array({short_id, id_link(0, 8)}));
    check(q["model_selection"]["status"] == "not_evaluated" &&
              q["model_id_link"]["selected_linkage_index"] == 0,
          "truncated selected ID does not fall through to later version-zero link");
    check(query(0x8000, Json::array({short_id}))["model_selection"]["status"] == "resolved",
          "default flag makes unknown explicit-ID value irrelevant to model selection");
    auto ignored_id = id_link(0, 99);
    ignored_id["header"] = 0;
    check(query(0, Json::array({ignored_id, name}))["model_selection"]["name"] == "Model",
          "non-user linkage cannot enable explicit model-ID override");
    q = query(0, Json::array({name, text_link(21, Bytes{'X'})}));
    check(q["model_selection"]["name"] == "Model" && q["strings"][2]["selected_linkage_index"] == 0,
          "string reader retains first match rather than last dictionary value");
    auto bad_name = name;
    auto p = bytesof(bad_name["payload"]);
    put(p, 4, 9999, 4);
    bad_name["payload"] = rawbytes(p);
    check(query(0, Json::array({bad_name, name}))["model_selection"]["status"] == "not_evaluated",
          "bad selected string length does not use shadowed valid name");
    check(query(0, Json::array({bad_name, id_link(0, 12)}))["model_selection"]["model_id"] == 12,
          "selected model ID does not require unused name string to decode");
    q = query(0, Json::array({text_link(21, Bytes{'A', 0, 'B'})}));
    check(q["model_selection"]["name"] == "A", "embedded null terminates the native wide result");
    q = query(0, Json::array({text_link(21, Bytes{0xc4, 0xe3})}));
    check(q["model_selection"]["utf16_code_units"] == Json::array({0xc4, 0xe3}),
          "unmarked bytes use code-page-1200 byte widening rather than inferred GBK");
    for (auto marker : {std::uint8_t(253), std::uint8_t(254)}) {
        q = query(0, Json::array({text_link(21, Bytes{255, marker, 0x2d, 0x4e, 0, 0})}));
        check(q["model_selection"]["name"] == "中", "both accepted wide markers use UTF16LE");
    }
    q = query(0, Json::array({text_link(21, Bytes{255, 254, 1, 0, 0xe9, 0})}));
    check(q["model_selection"]["utf16_code_units"] == Json::array({0xe9}),
          "marked narrow payload is widened after its four-byte marker");
    q = query(0, Json::array({text_link(21, Bytes{254, 255, 0, 65})}));
    check(q["strings"][2]["native_conversion_status"] == 2 &&
              q["model_selection"]["kind"] == "file_default_model",
          "rejected byte order leaves cleared destination and caller sees empty name");
    for (std::size_t n : {510, 511, 512, 513}) {
        q = query(0, Json::array({text_link(21, Bytes(n, 'x'))}));
        check(q["model_selection"]["utf16_code_units"].size() == (n <= 511 ? 510u : 511u),
              "native string termination preserves capacity-minus-one boundary behavior");
    }
    q = query(0, Json::array({text_link(4, Bytes(120, 'x')), text_link(2, Bytes(256, 'y'))}));
    check(q["strings"][0]["utf16_code_units"].size() == 95 &&
              q["strings"][1]["utf16_code_units"].size() == 255,
          "independent slot capacity and post-read truncation limits");
    q = query(0, Json::array({text_link(21, Bytes{255, 253, 65, 0, 99})}));
    check(q["model_selection"]["name"] == "A" && q["strings"][2]["discarded_odd_tail_byte"] == true,
          "wide copy uses whole code units and preserves odd-tail diagnostic");
    q = query(0, Json::array({text_link(21, Bytes{255, 254})}));
    check(q["model_selection"]["kind"] == "file_default_model",
          "BOM-only string uses appended zero marker lookahead");
    q = query(0, Json::array({text_link(21, Bytes{255, 253, 0, 0xd8})}));
    check(q["strings"][2]["text_status"] == "invalid_utf16" &&
              !q["model_selection"].contains("name") &&
              q["model_selection"]["utf16_code_units"] == Json::array({0xd800}) &&
              Json::parse(q.dump()) == q,
          "invalid surrogate retains exact native wide units without invalid UTF8 JSON");
    auto file = [&](Json links) { return query(0, links).at("file_specification"); };
    auto string_link = [&](unsigned key, const std::string &s) {
        return text_link(key, Bytes(s.begin(), s.end()));
    };
    auto file_link = string_link(3, "models/source.p3d");
    auto override_link = string_link(31, "other/lookup.p3d");
    q = file(Json::array({file_link, override_link, string_link(64, "context-token")}));
    check(q["status"] == "decoded" &&
              q["default_service_reference"]["stored_reference"] == "models/source.p3d" &&
              q["default_service_reference"]["lookup_reference"] == "other/lookup.p3d" &&
              q["service_parameter"] == "context-token" &&
              q["service_option"] == "enabled_by_nonempty_parameter" &&
              q["file_location"] == "not_evaluated",
          "file specification separates stored and lookup strings from actual file location");
    q = file(Json::array({file_link}));
    check(q["default_service_reference"]["lookup_reference"] == "models/source.p3d" &&
              q["service_option"] == "runtime_service_state_required",
          "empty alternate uses primary reference without inventing runtime service option");
    check(file(Json::array({override_link}))["resource_service_called"] == false &&
              file(Json::array({string_link(3, ""), override_link}))["status"] ==
                  "host_file_specification_required",
          "alternate cannot create a persisted file specification without nonempty primary");
    q = file(Json::array({text_link(3, Bytes{254, 255, 0, 65}), file_link}));
    check(q["status"] == "host_file_specification_required" &&
              q["strings"][0]["getter_status"] == "failure" &&
              q["strings"][0]["selected_linkage_index"] == 0,
          "rejected first file string triggers host fallback rather than later duplicate");
    q = file(Json::array({file_link, text_link(31, Bytes{254, 255, 0, 65})}));
    check(q["default_service_reference"]["lookup_reference"] == "models/source.p3d",
          "failed optional dynamic string getter leaves initialized empty alternate");
    q = file(Json::array({text_link(3, Bytes(4096, 'x'))}));
    check(q["strings"][0]["utf16_code_units"].size() == 4096 &&
              q["default_service_reference"]["lookup_reference"].get<std::string>().size() == 4096,
          "file string getter is linkage-sized and does not inherit 512-unit model-name limit");
    auto malformed_file = file_link;
    p = bytesof(malformed_file["payload"]);
    put(p, 4, 9999, 4);
    malformed_file["payload"] = rawbytes(p);
    check(file(Json::array({malformed_file, file_link}))["status"] == "not_evaluated",
          "out-of-linkage file string cannot be converted into a known absent reference");
    q = file(Json::array({text_link(3, Bytes{255, 253, 0, 0xd8})}));
    check(q["status"] == "not_evaluated" &&
              q["strings"][0]["utf16_code_units"] == Json::array({0xd800}) &&
              Json::parse(q.dump()) == q,
          "file reference with invalid Unicode preserves source units without invented text");
    struct ResourceCase {
        const char *primary, *alternate, *stored, *lookup;
    };
    for (const auto &c : {
             ResourceCase{"a<2>x", "", "a", "a<2>x"},
             ResourceCase{"a<2>x", "ordinary", "a", "a<2>x"},
             ResourceCase{"ordinary", "b<3>y", "", "b<3>y"},
             ResourceCase{"a<2>x", "b<3>y", "a", "b<3>y"},
             ResourceCase{"a<2>x", "b<bad>y", "a", "b<2>y"},
             ResourceCase{"a<2>x", "<bad>y", "a", "a<2>y"},
             ResourceCase{"a<bad>x", "b<3>y", "a", "b<3>y"},
             ResourceCase{"a<bad>x", "b<bad>y", "a<bad>x", "b<bad>y"},
             ResourceCase{"a<7>x", "b<0>y", "a", "b"},
             ResourceCase{"a<7>x", "b<-9>y", "a", "b<>y"},
             ResourceCase{"<2>x", "", "", ""},
             ResourceCase{"http:source<2>x", "", "http:source", "http:source<2>x"},
         }) {
        q = native_file_resource_reference(c.primary, c.alternate);
        check(q["status"] == "decoded" && q["stored_reference"] == c.stored &&
                  q["lookup_reference"] == c.lookup,
              "default file resource service preserves ordered marker-scanner side effects");
    }
    check(material_resource_reference("http:source<2>x")["value"] == "http:source",
          "material wrapper keeps its separate http prefix behavior");
    check(native_file_resource_reference("a<2147483648>", "b<1>")["status"] == "not_evaluated",
          "undefined primary integer overflow is not masked by a valid alternate");
    auto state_link = [&](std::uint64_t value, unsigned count = 1) {
        Bytes p(16);
        put(p, 0, 19, 2);
        put(p, 2, 0x3456, 2);
        put(p, 4, count, 4);
        put(p, 8, value, 8);
        return link(0x56d5, p);
    };
    auto no_state = query(0, Json::array());
    check(no_state["file_query_state"]["source"] == "initial_zero_state" &&
              no_state["file_query_state"]["blocks_file_query_here"] == false,
          "absent numeric state uses initial reference zero state");
    for (unsigned bits = 0; bits < 16; ++bits) {
        const auto value = 0xfedcba9876543210ull | bits;
        const auto target = query(0, Json::array({state_link(value)}));
        const auto &state = target.at("file_query_state");
        check(state["state_pair_0"] == (bits & 3) && state["state_pair_1"] == (bits >> 2) &&
                  state["blocks_file_query_here"] == (bits == 13) &&
                  state["applied_links"][0]["source_value"] == value &&
                  state["applied_links"][0]["reserved_word"] == 0x3456,
              "all sixteen persisted gate states retain full source value");
    }
    auto blocked = query(0, Json::array({state_link(13)}));
    q = query(0, Json::array({state_link(13), state_link(0)}));
    check(q["file_query_state"]["blocks_file_query_here"] == false &&
              q["file_query_state"]["selected_linkage_index"] == 1 &&
              q["file_query_state"]["applied_links"].size() == 2,
          "later accepted numeric state replaces earlier blocked state");
    q = query(0, Json::array({state_link(0), state_link(13), state_link(0, 0)}));
    check(q["file_query_state"]["blocks_file_query_here"] == true &&
              q["file_query_state"]["selected_linkage_index"] == 1 &&
              q["file_query_state"]["skipped_links"].size() == 1,
          "zero-count numeric link leaves preceding state unchanged");
    auto many = state_link(13, 99);
    auto many_bytes = bytesof(many["payload"]);
    many_bytes.insert(many_bytes.end(), {0xaa, 0xbb});
    many = link(0x56d5, many_bytes);
    q = query(0, Json::array({many}));
    check(q["file_query_state"]["blocks_file_query_here"] == true &&
              q["file_query_state"]["applied_links"][0]["declared_count"] == 99 &&
              bytesof(q["file_query_state"]["applied_links"][0]["trailing_storage"]) ==
                  Bytes({0xaa, 0xbb}),
          "native numeric gate consumes one value for every positive declared count");
    auto non_user = state_link(13);
    non_user["header"] = non_user["header"].get<unsigned>() & ~0x1000;
    auto other_app = state_link(13);
    other_app["app"] = 0x56d4;
    Bytes other_key(2);
    put(other_key, 0, 20, 2);
    q = query(0, Json::array({non_user, other_app, link(0x56d5, other_key)}));
    check(q["file_query_state"]["status"] == "decoded" &&
              q["file_query_state"]["blocks_file_query_here"] == false &&
              q["file_query_state"]["applied_links"].empty(),
          "gate ignores other applications, keys, and non-user links");
    Json unknown;
    for (const auto size : {0u, 2u, 6u, 8u, 14u}) {
        auto p = bytesof(state_link(13)["payload"]);
        p.resize(size);
        unknown = query(0, Json::array({link(0x56d5, p), state_link(0)}));
        check(unknown["status"] == "partial" &&
                  unknown["file_query_state"]["status"] == "not_evaluated" &&
                  !unknown["file_query_state"].contains("blocks_file_query_here"),
              "truncated state cannot be replaced by a guessed zero or later link");
    }
    q = initial_reference_file_query_gate({no_state, blocked, unknown}, false);
    check(q["status"] == "blocked" && q["blocked"] == true && q["blocking_reference_index"] == 1 &&
              q["examined_references"] == 2,
          "known host block short circuits without requiring the remaining chain");
    q = initial_reference_file_query_gate({blocked}, false);
    check(q["status"] == "blocked" && q["blocking_reference_index"] == 0,
          "current reference can block file query without host context");
    q = initial_reference_file_query_gate({no_state, no_state}, true);
    check(q["status"] == "allowed" && q["blocked"] == false && q["examined_references"] == 2,
          "only an entirely known complete selected chain permits initial file query");
    for (const auto &chain :
         {std::vector<Json>{}, std::vector<Json>{no_state}, std::vector<Json>{unknown, blocked},
          std::vector<Json>{Json::object()}}) {
        q = initial_reference_file_query_gate(chain, false);
        check(q["status"] == "not_evaluated" && !q.contains("blocked"),
              "missing, incomplete, or unknown initial gate inputs stay unresolved");
    }
    auto invalid_gate = no_state;
    invalid_gate["file_query_state"]["blocks_file_query_here"] = 0;
    q = initial_reference_file_query_gate({invalid_gate}, true);
    check(q["status"] == "not_evaluated" && !q.contains("blocked"),
          "a non-boolean supplied gate state cannot authorize a file query");
    Bytes raw(372, 0);
    put(raw, 4, 13, 2);
    put(raw, 12, 184, 4);
    put(raw, 160, 1, 4);
    for (unsigned i = 0; i < 3; ++i) {
        std::uint64_t bits;
        double value = 1;
        std::memcpy(&bits, &value, 8);
        put(raw, 220 + 32 * i, bits, 8);
        put(raw, 292, bits, 8);
    }
    const auto payload_bytes = bytesof(name.at("payload"));
    const auto at = raw.size();
    raw.resize(at + 4 + payload_bytes.size());
    put(raw, at, name.at("header").get<unsigned>(), 2);
    put(raw, at + 2, 0x56d2, 2);
    std::copy(payload_bytes.begin(), payload_bytes.end(), raw.begin() + at + 4);
    const auto file_payload = bytesof(file_link.at("payload"));
    const auto file_at = raw.size();
    raw.resize(file_at + 4 + file_payload.size());
    put(raw, file_at, file_link.at("header").get<unsigned>(), 2);
    put(raw, file_at + 2, 0x56d2, 2);
    std::copy(file_payload.begin(), file_payload.end(), raw.begin() + file_at + 4);
    const auto numeric = state_link(13);
    const auto numeric_payload = bytesof(numeric.at("payload"));
    const auto numeric_at = raw.size();
    raw.resize(numeric_at + 4 + numeric_payload.size());
    put(raw, numeric_at, numeric.at("header").get<unsigned>(), 2);
    put(raw, numeric_at + 2, 0x56d5, 2);
    std::copy(numeric_payload.begin(), numeric_payload.end(), raw.begin() + numeric_at + 4);
    put(raw, 8, (raw.size() - 4) / 2, 4);
    const auto records = parse_native(raw);
    check(records.size() == 1 &&
              records[0]["reference_target"]["model_selection"]["name"] == "Model" &&
              records[0]["reference_target"]["strings"][2]["source_offset"] == at,
          "native reference parsing automatically exposes linkage-derived model selector");
    check(records[0]["reference_target"]["file_specification"]["default_service_reference"]
                 ["lookup_reference"] == "models/source.p3d" &&
              records[0]["reference_target"]["file_specification"]["strings"][0]["source_offset"] ==
                  file_at,
          "native record parsing integrates file specification with source linkage location");
    check(records[0]["reference_target"]["file_query_state"]["blocks_file_query_here"] == true &&
              records[0]["reference_target"]["file_query_state"]["applied_links"][0]
                     ["source_offset"] == numeric_at,
          "native record parsing integrates persisted file-query state and source location");
    Bytes control(32), header(0x610);
    put(header, 0x124, 0xf1234567, 4);
    put(header, 0x128, 0x1122334455667788ull, 8);
    std::uint8_t previous = 0x26;
    for (std::size_t i = 0; i < 20; ++i) {
        control[12 + i] = previous;
        previous = control[12 + i];
    }
    q = native_file_header(control, header);
    check(q["initial_probe"]["default_model_id"] == 0xf1234567u &&
              q["initial_probe"]["id_counter"] == 0x1122334455667788ull,
          "default-model uint32 field is adjacent to but distinct from uint64 ID counter");
    previous = 0x26;
    for (std::size_t i = 0; i < 20; ++i) {
        control[12 + i] = std::uint8_t((i == 0 ? 2 : 0) ^ previous);
        previous = control[12 + i];
    }
    q = native_file_header(control, header);
    check(q["source_default_model_id"] == 0xf1234567u &&
              q["initial_probe"]["default_model_id"] == 0,
          "blank header initialization does not consume saved default-model ID");
    return checks;
}
