#include "internal.hpp"
#include <p3d/proxy_cache.hpp>

unsigned cache_hash_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto set = [](Bytes &b, std::size_t offset, std::uint32_t value) {
        for (unsigned i = 0; i < 4; ++i)
            b[offset + i] = std::uint8_t(value >> (8 * i));
    };
    auto hash = [&](std::uint32_t mode, std::uint32_t size) {
        Bytes out(32);
        out[0] = 1;
        set(out, 4, mode);
        set(out, 8, size);
        for (unsigned i = 0; i < 20; ++i)
            out[12 + i] = std::uint8_t(i + 1);
        return out;
    };
    for (std::uint32_t mode : {0u, 1u, 2u, UINT32_MAX}) {
        const auto bytes = hash(mode, mode == 0 ? 16 : 20);
        const auto decoded = decode_native_cache_hash(bytes);
        check(decoded.at("status") == "decoded" && decoded.at("mode") == mode,
              "hash mode retains exact unsigned source value");
        check(decoded.at("algorithm") == (mode == 0 ? "md5" : "sha1") &&
                  decoded.at("algorithm_id") == (mode == 0 ? 0x8003 : 0x8004),
              "native zero-versus-nonzero algorithm selection");
        check(decoded.at("hash_generation_status") == "not_evaluated" &&
                  decoded.at("size_matches_algorithm") == true,
              "conventional size and exported marker do not prove runtime hash success");
        const auto same = compare_native_cache_hash(bytes, bytes);
        check(same.at("status") == "resolved" && same.at("match") == true &&
                  same.at("cache_validity_status") == "not_evaluated",
              "equality alone never validates a whole cache");
    }
    const auto md5 = decode_native_cache_hash(hash(0, 16));
    check(md5.at("digest_hex") == "0102030405060708090a0b0c0d0e0f10" &&
              bytesof(md5.at("unused_digest_storage")) == Bytes({17, 18, 19, 20}),
          "digest bytes keep their original byte order and separate unused tail");
    check(compare_native_cache_hash(hash(1, 20), hash(2, 20)).at("reason") == "mode_mismatch",
          "two modes using SHA1 are still distinct native keys");
    // Native comparison accepts any equal length up to 20, including zero.
    // Test every relevant byte against that rule, not only standard hashes.
    for (unsigned size = 0; size <= 20; ++size) {
        const auto saved = hash(0, size);
        const auto decoded = decode_native_cache_hash(saved);
        check(bytesof(decoded.at("digest")).size() == size &&
                  decoded.at("size_matches_algorithm") == (size == 16),
              "nonstandard digest sizes remain visible instead of being normalized");
        for (unsigned byte = 0; byte < 32; ++byte) {
            auto current = saved;
            current[byte] ^= 0x80;
            const auto compared = compare_native_cache_hash(saved, current);
            const bool expected = byte < 4 || byte >= 12 + size;
            check(compared.at("status") == "resolved" && compared.at("match") == expected,
                  "native comparison ignores marker, padding and unused digest bytes only");
        }
    }
    for (std::uint32_t size : {21u, UINT32_MAX}) {
        const auto bytes = hash(0, size);
        const auto decoded = decode_native_cache_hash(bytes);
        check(decoded.at("status") == "decoded" && decoded.at("digest").is_null() &&
                  decoded.at("digest_status") == "declared_size_exceeds_storage" &&
                  bytesof(decoded.at("digest_storage")).size() == 20,
              "oversized digest declaration preserves header and all stored bytes");
        check(compare_native_cache_hash(bytes, bytes).at("match") == false,
              "identical overlong declarations do not match natively");
    }
    for (unsigned size = 0; size <= 33; ++size) {
        if (size == 32)
            continue;
        check(decode_native_cache_hash(Bytes(size)).at("status") == "not_evaluated",
              "exact hash storage framing required");
        check(compare_native_cache_hash(Bytes(size), hash(0, 16)).at("status") == "not_evaluated" &&
                  compare_native_cache_hash(hash(0, 16), Bytes(size)).at("status") ==
                      "not_evaluated",
              "bad framing on either input is unresolved rather than a mismatch");
    }
    return checks;
}
