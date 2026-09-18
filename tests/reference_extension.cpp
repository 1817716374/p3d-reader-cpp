#include "internal.hpp"
unsigned reference_extension_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool v, const char *message) {
        ++checks;
        require(v, message);
    };
    auto put = [](Bytes &b, std::size_t at, std::uint64_t v, unsigned size) {
        for (unsigned i = 0; i < size; ++i)
            b.at(at + i) = std::uint8_t(v >> (8 * i));
    };
    auto main_payload = [&](std::uint32_t id) {
        Bytes b(64, 0xa5);
        put(b, 60, id, 4);
        return b;
    };
    auto attr = [](unsigned group, unsigned key, unsigned ix, const Bytes &b) {
        return Json{{"group", group}, {"key", key}, {"index", ix}, {"payload", rawbytes(b)}};
    };
    for (unsigned group : {0u, 20117u})
        for (unsigned size : {0u, 11u, 12u, 63u, 64u, 65u}) {
            const auto d = decode_attribute(group, 20081, Bytes(size, 0), 0);
            check(d["encoding"] == "native_reference_extension" && d["status"] == "partial" &&
                      d["loader_status"] ==
                          (size == (group ? 12u : 64u) ? "accepted_length" : "ignored_length"),
                  "reference extension loader uses exact length without claiming complete "
                  "parameter semantics");
        }
    check(decode_reference_extension(1, 20081, {}, 0).is_null() &&
              decode_reference_extension(0, 20080, {}, 0).is_null(),
          "unrelated attribute keys do not become reference extensions");
    auto d = decode_attribute(0, 20081, main_payload(0xabcd1234), 0);
    check(d["scale_provider"]["id"] == 0xabcd1234u && d["scale_provider"]["enabled"] == true &&
              d["ignored_word"]["value"] == 0xa5a5a5a5u && d["unresolved_ranges"].size() == 2,
          "provider comes from offset60 while nonzero ignored padding is accepted and unknown "
          "ranges remain explicit");
    for (unsigned id : {0u, 65535u, 65536u, 0xffffffffu}) {
        d = decode_attribute(0, 20081, main_payload(id), 0);
        check(d["scale_provider"]["id"] == id && d["scale_provider"]["enabled"] == bool(id >> 16),
              "provider high word controls whether the native query is enabled");
    }
    Bytes aux(12, 0x81);
    put(aux, 8, 0x87654321u, 4);
    d = decode_attribute(20117, 20081, aux, 0);
    check(d["auxiliary_state_word"]["value"] == 0x87654321u && d["ignored_prefix"]["size"] == 8 &&
              d["auxiliary_state_word"]["meaning"] == "unresolved",
          "auxiliary reader ignores the eight-byte writer prefix and retains the unnamed word");
    auto input = reference_extension_input(Json::array());
    check(input["status"] == "decoded" && input["scale_provider"]["id"] == 0 &&
              input["auxiliary_state_word"] == 0xfffffffeu &&
              input["parameter_block_loaded"] == false,
          "known empty initial collection uses constructor defaults");
    check(reference_extension_input(Json())["status"] == "not_evaluated",
          "missing collection is not a known empty array");
    Json oversized = Json::array();
    oversized.get_ref<Json::array_t &>().resize(65536);
    check(reference_extension_input(oversized)["reason"] ==
              "initial_attribute_count_exceeds_native_read_width",
          "full source count above native u16 read width cannot be treated as a loaded collection");
    Json attrs =
        Json::array({attr(0, 20081, 0, main_payload(0x10000)), attr(20117, 20081, 0, aux)});
    input = reference_extension_input(attrs);
    check(input["scale_provider"]["id"] == 0x10000u &&
              input["auxiliary_state_word"] == 0x87654321u &&
              input["parameter_block_loaded"] == true &&
              input["selections"][1]["source_ordinal"] == 1,
          "each extension has an independent exact key and index selection");
    auto duplicate = attrs;
    duplicate.push_back(attr(0, 20081, 0, main_payload(0x20000)));
    input = reference_extension_input(duplicate);
    check(input["scale_provider"]["id"] == 0x20000u &&
              input["selections"][0]["source_ordinal"] == 2,
          "small collection native reverse lookup chooses the last duplicate");
    duplicate.push_back(attr(0, 20081, 0, Bytes(63, 0)));
    input = reference_extension_input(duplicate);
    check(
        input["scale_provider"]["id"] == 0 &&
            input["selections"][0]["status"] == "ignored_length" &&
            input["auxiliary_state_word"] == 0x87654321u,
        "wrong length selected entry leaves default rather than falling back to a valid duplicate");
    Json many = Json::array(
        {attr(0, 20081, 0, main_payload(0x10000)), attr(0, 20081, 0, main_payload(0x20000))});
    for (unsigned i = 0; i < 4; ++i)
        many.push_back(attr(0, 20082 + i, 0, {}));
    input = reference_extension_input(many);
    check(input["scale_provider"]["id"] == 0x10000u &&
              input["lookup"]["lookup_method"] == "lower_bound",
          "larger collection follows native lower bound duplicate selection");
    attrs[0]["index"] = 1;
    input = reference_extension_input(attrs);
    check(input["scale_provider"]["id"] == 0 && input["selections"][0]["status"] == "absent" &&
              decode_attribute(0, 20081, main_payload(7), 1)["lookup_index_matches"] == false,
          "nonzero attribute index is preserved but not read by the reference extension loader");
    attrs[0]["index"] = 0;
    attrs[0]["payload"] = nullptr;
    check(reference_extension_input(attrs)["status"] == "not_evaluated",
          "unreadable selected payload does not default successfully");
    attrs = Json::array({attr(0, 20081, 0, main_payload(0x10000)), attr(20117, 20081, 0, aux)});
    Bytes graphics(28, 0);
    put(graphics, 0, 0xa11b, 4);
    put(graphics, 16, 42, 8);
    put(graphics, 24, 2, 4);
    for (const auto &a : attrs) {
        const auto raw = bytesof(a["payload"]);
        const auto at = graphics.size();
        graphics.resize(at + 16 + raw.size());
        put(graphics, at, a["group"].get<unsigned>(), 2);
        put(graphics, at + 2, 20081, 2);
        put(graphics, at + 8, raw.size(), 4);
        std::copy(raw.begin(), raw.end(), graphics.begin() + at + 16);
    }
    graphics.resize(graphics.size() + 4, 0);
    put(graphics, 4, graphics.size() - 28, 4);
    const auto records = parse_graphics(graphics);
    check(records.size() == 1 &&
              records[0]["reference_extension_input"]["scale_provider"]["id"] == 0x10000u &&
              records[0]["attributes"][0]["decoded"]["scale_provider"]["id"] == 0x10000u &&
              bytesof(records[0]["attributes"][0]["payload"]) == main_payload(0x10000),
          "graphics parsing exposes the conditional initial extension state and preserves every "
          "payload byte");
    check(Json::parse(records.dump()) == records,
          "reference extension fields round trip through JSON");
    return checks;
}
