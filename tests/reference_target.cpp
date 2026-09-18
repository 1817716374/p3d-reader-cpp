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
    put(raw, 8, (raw.size() - 4) / 2, 4);
    const auto records = parse_native(raw);
    check(records.size() == 1 &&
              records[0]["reference_target"]["model_selection"]["name"] == "Model" &&
              records[0]["reference_target"]["strings"][2]["source_offset"] == at,
          "native reference parsing automatically exposes linkage-derived model selector");
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
