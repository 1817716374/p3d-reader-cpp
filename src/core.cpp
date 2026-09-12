#include "geometry.hpp"
#include "internal.hpp"
#include <atomic>
#include <fstream>
#include <lz4/lz4.h>
#include <mutex>
#include <pugixml/pugixml.hpp>
#include <thread>
#ifdef _WIN32
#include <windows.h>
#else
#include <iconv.h>
#endif
namespace p3d {
Bytes slice(const Bytes &b, std::size_t p, std::size_t n) {
    Reader r(b, p);
    return r.take(n);
}
std::string hex(const Bytes &b) {
    static const char *h = "0123456789abcdef";
    std::string s;
    s.reserve(b.size() * 2);
    for (auto c : b) {
        s += h[c >> 4];
        s += h[c & 15];
    }
    return s;
}
std::string base64(const Bytes &b) {
    static const char *a = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string s;
    s.reserve((b.size() + 2) / 3 * 4);
    for (std::size_t i = 0; i < b.size(); i += 3) {
        unsigned v = unsigned(b[i]) << 16;
        if (i + 1 < b.size())
            v |= unsigned(b[i + 1]) << 8;
        if (i + 2 < b.size())
            v |= b[i + 2];
        s += a[(v >> 18) & 63];
        s += a[(v >> 12) & 63];
        s += i + 1 < b.size() ? a[(v >> 6) & 63] : '=';
        s += i + 2 < b.size() ? a[v & 63] : '=';
    }
    return s;
}
Bytes unbase64(const std::string &s) {
    require(s.size() % 4 == 0, "base64 length");
    Bytes b;
    unsigned v = 0, bits = 0;
    for (char c : s) {
        if (c == '=')
            break;
        unsigned x = c >= 'A' && c <= 'Z'   ? c - 'A'
                     : c >= 'a' && c <= 'z' ? c - 'a' + 26
                     : c >= '0' && c <= '9' ? c - '0' + 52
                     : c == '+'             ? 62
                     : c == '/'             ? 63
                                            : 255;
        require(x < 64, "base64 character");
        v = (v << 6) | x;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            b.push_back((v >> bits) & 255);
        }
    }
    return b;
}
static void codepoint(std::string &s, std::uint32_t c) {
    require(c <= 0x10ffff && !(c >= 0xd800 && c <= 0xdfff), "invalid Unicode scalar");
    if (c < 128)
        s += char(c);
    else if (c < 2048) {
        s += char(192 | (c >> 6));
        s += char(128 | (c & 63));
    } else if (c < 65536) {
        s += char(224 | (c >> 12));
        s += char(128 | ((c >> 6) & 63));
        s += char(128 | (c & 63));
    } else {
        s += char(240 | (c >> 18));
        s += char(128 | ((c >> 12) & 63));
        s += char(128 | ((c >> 6) & 63));
        s += char(128 | (c & 63));
    }
}
std::string utf16(const Bytes &b) {
    require(b.size() % 2 == 0, "odd UTF16 length");
    std::string s;
    Reader r(b);
    while (r.left()) {
        std::uint32_t c = r.u16();
        if (c >= 0xd800 && c <= 0xdbff) {
            auto d = r.u16();
            require(d >= 0xdc00 && d <= 0xdfff, "UTF16 surrogate");
            c = 0x10000 + ((c - 0xd800) << 10) + d - 0xdc00;
        }
        codepoint(s, c);
    }
    return s;
}
std::string latin1(const Bytes &b) {
    std::string s;
    for (auto c : b)
        codepoint(s, c);
    return s;
}
std::string utf8(const Bytes &b) {
    std::string s(b.begin(), b.end());
    Json(s).dump();
    return s;
}
std::string gb18030(const Bytes &b) {
#ifdef _WIN32
    int n =
        MultiByteToWideChar(54936, MB_ERR_INVALID_CHARS, reinterpret_cast<const char *>(b.data()),
                            int(b.size()), nullptr, 0);
    require(n > 0 || b.empty(), "invalid GB18030");
    Bytes w(n * 2);
    if (n)
        require(MultiByteToWideChar(54936, MB_ERR_INVALID_CHARS,
                                    reinterpret_cast<const char *>(b.data()), int(b.size()),
                                    reinterpret_cast<wchar_t *>(w.data()), n) == n,
                "GB18030 conversion");
    return utf16(w);
#else
    iconv_t c = iconv_open("UTF-8", "GB18030");
    require(c != (iconv_t)-1, "GB18030 converter unavailable");
    std::string result(b.size() * 4 + 4, '\0');
    char *in = (char *)b.data();
    char *out = result.data();
    std::size_t ni = b.size(), no = result.size();
    auto rc = iconv(c, &in, &ni, &out, &no);
    iconv_close(c);
    require(rc != (std::size_t)-1 && ni == 0, "invalid GB18030");
    result.resize(result.size() - no);
    return result;
#endif
}
std::string Reader::string(char fmt) {
    auto n = fmt == 'I' ? u32() : u64();
    require(n <= left() / 2, "UTF16 string length");
    return utf16(take(n * 2));
}
std::string Reader::zstring() {
    auto q = p;
    while (left() >= 2) {
        if (u16() == 0)
            return utf16(slice(b, q, p - q - 2));
    }
    throw std::runtime_error("unterminated UTF16 string");
}
std::string native_string(const Bytes &b) {
    std::string s;
    if (b.size() >= 4 && hex(slice(b, 0, 4)) == "fffe0100")
        s = latin1(slice(b, 4, b.size() - 4));
    else if (b.size() >= 2 && b[0] == 255 && b[1] == 254)
        s = utf16(slice(b, 2, b.size() - 2));
    else if (b.size() >= 2 && b[0] == 255 && b[1] == 253) {
        Reader r(b, 2);
        Bytes raw;
        while (r.left()) {
            auto c = r.u16();
            if (c >= 256)
                raw.push_back(c >> 8);
            raw.push_back(c & 255);
        }
        s = gb18030(raw);
    } else
        throw std::runtime_error("unsupported native string encoding");
    while (!s.empty() && s.back() == 0)
        s.pop_back();
    return s;
}
static std::string qualified(const pugi::xml_node &n) {
    std::string name = n.name();
    auto p = name.find(':');
    std::string prefix = p == std::string::npos ? "" : name.substr(0, p),
                local = p == std::string::npos ? name : name.substr(p + 1);
    std::string key = prefix.empty() ? "xmlns" : "xmlns:" + prefix;
    for (auto t = n; t; t = t.parent()) {
        auto a = t.attribute(key.c_str());
        if (a)
            return "{" + std::string(a.value()) + "}" + local;
    }
    return name;
}
static Json xml_node(const pugi::xml_node &n) {
    Json a = Json::object(), children = Json::array();
    for (auto x : n.attributes()) {
        std::string k = x.name();
        if (k == "xmlns" || k.rfind("xmlns:", 0) == 0)
            continue;
        auto p = k.find(':');
        if (p != std::string::npos) {
            std::string pre = k.substr(0, p), key = "xmlns:" + pre;
            for (auto t = n; t; t = t.parent()) {
                auto ns = t.attribute(key.c_str());
                if (ns) {
                    k = "{" + std::string(ns.value()) + "}" + k.substr(p + 1);
                    break;
                }
            }
        }
        a[k] = x.value();
    }
    Json text = nullptr, tail = nullptr;
    for (auto c = n.first_child(); c; c = c.next_sibling()) {
        if (c.type() == pugi::node_element)
            children.push_back(xml_node(c));
        else if (c.type() == pugi::node_pcdata || c.type() == pugi::node_cdata) {
            if (children.empty())
                text = text.is_null() ? Json(c.value()) : Json(text.get<std::string>() + c.value());
        }
    }
    for (auto t = n.next_sibling(); t && t.type() != pugi::node_element; t = t.next_sibling())
        if (t.type() == pugi::node_pcdata || t.type() == pugi::node_cdata)
            tail = tail.is_null() ? Json(t.value()) : Json(tail.get<std::string>() + t.value());
    return {{"tag", qualified(n)},
            {"attributes", a},
            {"text", text},
            {"tail", tail},
            {"children", children}};
}
Json xml_tree(const std::string &s) {
    pugi::xml_document d;
    auto r = d.load_buffer(s.data(), s.size(), pugi::parse_default | pugi::parse_ws_pcdata,
                           pugi::encoding_utf8);
    require(bool(r), std::string("XML: ") + r.description());
    require(bool(d.document_element()), "empty XML");
    return xml_node(d.document_element());
}
static Bytes lz(const Bytes &b, std::size_t n, std::size_t limit) {
    require(n > 0 && n <= limit && n <= INT32_MAX && b.size() <= INT32_MAX, "LZ4 size limit");
    Bytes out(n);
    auto got =
        LZ4_decompress_safe((const char *)b.data(), (char *)out.data(), int(b.size()), int(n));
    require(got == int(n), "LZ4 size mismatch");
    return out;
}
class Cfb {
    Bytes file, mini;
    std::size_t sector = 0;
    std::vector<std::uint32_t> fat, mfat;
    struct Entry {
        std::string name;
        unsigned type;
        std::uint32_t left, right, child, start;
        std::uint64_t size;
    };
    std::vector<Entry> dirs;
    Bytes sec(std::uint32_t i) const {
        require(i < (file.size() / sector) - 1, "CFB sector range");
        return slice(file, (std::size_t(i) + 1) * sector, sector);
    }
    Bytes chain(std::uint32_t start, const std::vector<std::uint32_t> &table, std::size_t unit,
                const Bytes *storage = nullptr, std::uint64_t length = UINT64_MAX) const {
        Bytes out;
        std::set<std::uint32_t> visited;
        auto p = start;
        while (p != 0xfffffffe) {
            require(p < table.size() && visited.insert(p).second, "CFB invalid/cyclic chain");
            Bytes b = storage ? slice(*storage, std::size_t(p) * unit, unit) : sec(p);
            std::size_t n = length == UINT64_MAX
                                ? unit
                                : std::size_t(std::min<std::uint64_t>(unit, length - out.size()));
            out.insert(out.end(), b.begin(), b.begin() + n);
            p = table[p];
            if (out.size() == length) {
                require(p == 0xfffffffe, "CFB excess chain sectors");
                break;
            }
            require(visited.size() <= file.size() / 64, "CFB chain limit");
        }
        require(length == UINT64_MAX || out.size() == length, "CFB truncated chain");
        return out;
    }

  public:
    explicit Cfb(const std::filesystem::path &path) {
        std::ifstream f(path, std::ios::binary);
        require(bool(f), "cannot open P3D");
        f.seekg(0, std::ios::end);
        auto n = f.tellg();
        require(n >= 512, "short CFB");
        file.resize(std::size_t(n));
        f.seekg(0);
        require(bool(f.read((char *)file.data(), n)), "CFB read failure");
        Reader r(file);
        require(hex(r.take(8)) == "d0cf11e0a1b11ae1", "not CFB");
        require(r.at<std::uint16_t>(28) == 0xfffe, "CFB byte order");
        auto major = r.at<std::uint16_t>(26), shift = r.at<std::uint16_t>(30);
        require((major == 3 && shift == 9) || (major == 4 && shift == 12), "CFB version");
        sector = std::size_t(1) << shift;
        require(file.size() % sector == 0 && r.at<std::uint16_t>(32) == 6, "CFB sector size");
        std::vector<std::uint32_t> ids;
        for (std::size_t i = 76; i < 512; i += 4) {
            auto id = r.at<std::uint32_t>(i);
            if (id != 0xffffffff)
                ids.push_back(id);
        }
        auto next = r.at<std::uint32_t>(68);
        auto nd = r.at<std::uint32_t>(72);
        require(nd <= file.size() / sector, "CFB DIFAT count");
        std::set<std::uint32_t> seen;
        for (unsigned j = 0; j < nd; ++j) {
            require(seen.insert(next).second, "CFB DIFAT cycle");
            auto b = sec(next);
            Reader d(b);
            for (std::size_t i = 0; i < sector / 4 - 1; ++i) {
                auto id = d.u32();
                if (id != 0xffffffff)
                    ids.push_back(id);
            }
            next = d.u32();
        }
        require(ids.size() == r.at<std::uint32_t>(44), "CFB FAT count");
        for (auto id : ids) {
            auto b = sec(id);
            Reader d(b);
            while (d.left())
                fat.push_back(d.u32());
        }
        auto db = chain(r.at<std::uint32_t>(48), fat, sector);
        Reader dr(db);
        for (std::size_t p = 0; p + 128 <= db.size(); p += 128) {
            unsigned n = dr.at<std::uint16_t>(p + 64), t = db[p + 66];
            require(t == 0 || (n >= 2 && n <= 64 && n % 2 == 0), "CFB directory name");
            Entry e{t ? utf16(slice(db, p, n - 2)) : "", t,
                    dr.at<std::uint32_t>(p + 68),        dr.at<std::uint32_t>(p + 72),
                    dr.at<std::uint32_t>(p + 76),        dr.at<std::uint32_t>(p + 116),
                    dr.at<std::uint64_t>(p + 120)};
            if (major == 3)
                e.size &= 0xffffffff;
            dirs.push_back(e);
        }
        require(!dirs.empty() && dirs[0].type == 5, "CFB root missing");
        mini = chain(dirs[0].start, fat, sector, nullptr, dirs[0].size);
        auto mc = r.at<std::uint32_t>(64);
        if (mc) {
            auto mb =
                chain(r.at<std::uint32_t>(60), fat, sector, nullptr, std::uint64_t(mc) * sector);
            Reader m(mb);
            while (m.left())
                mfat.push_back(m.u32());
        }
        require(r.at<std::uint32_t>(56) == 4096, "CFB mini cutoff");
    }
    std::vector<Stream> streams() {
        std::vector<Stream> out;
        std::set<unsigned> seen;
        std::function<void(unsigned, StreamPath, unsigned)> walk = [&](unsigned i, StreamPath p,
                                                                       unsigned depth) {
            if (i == 0xffffffff)
                return;
            require(i < dirs.size() && seen.insert(i).second && depth < 512, "CFB directory graph");
            const auto &e = dirs[i];
            walk(e.left, p, depth + 1);
            auto here = p;
            here.push_back(e.name);
            if (e.type == 2) {
                Bytes b;
                if (e.size)
                    b = e.size < 4096 ? chain(e.start, mfat, 64, &mini, e.size)
                                      : chain(e.start, fat, sector, nullptr, e.size);
                Stream s;
                s.path = here;
                s.raw = std::make_shared<Bytes>(std::move(b));
                out.push_back(std::move(s));
            } else if (e.type == 1)
                walk(e.child, here, depth + 1);
            else
                require(false, "CFB unexpected directory type");
            walk(e.right, p, depth + 1);
        };
        walk(dirs[0].child, {}, 0);
        std::sort(out.begin(), out.end(), [](auto &a, auto &b) { return a.path < b.path; });
        return out;
    }
};
struct Document::Impl {
    std::vector<Stream> streams;
    Json index = Json::object(), model_ids = Json::object(), schema_ids = Json::object(),
         objects = Json::array(), native = Json::array(), graphics = Json::array(),
         bindings = Json::array(), relations = Json::array(), materials, schemas, models,
         colors = Json::array(), layers, file_header;
    Options options;
};
Document::Document(const std::filesystem::path &path, Options opt)
    : impl_(std::make_shared<Impl>()) {
    auto &d = *impl_;
    d.options = opt;
    d.streams = Cfb(path).streams();
    auto it = std::find_if(d.streams.begin(), d.streams.end(),
                           [](auto &s) { return s.path == StreamPath{"P3DSIndexes"}; });
    require(it != d.streams.end(), "missing P3DSIndexes");
    Reader ir(*it->raw, 32);
    auto n = ir.u32(), c = ir.u32();
    d.file_header = native_file_header(*it->raw, lz(ir.take(c), n, opt.max_stream_bytes));
    ir.skip(12);
    n = ir.u32();
    c = ir.u32();
    auto table = utf16(lz(ir.take(c), n, opt.max_stream_bytes));
    while (!table.empty() && table.back() == 0)
        table.pop_back();
    std::size_t p = 0;
    while (p < table.size()) {
        auto end = table.find(';', p);
        if (end == std::string::npos)
            end = table.size();
        auto entry = table.substr(p, end - p);
        if (!entry.empty()) {
            auto q = entry.find(':');
            require(q != std::string::npos, "P3D index entry");
            d.index[entry.substr(0, q)] = entry.substr(q + 1);
        }
        p = end + 1;
    }
    for (auto e = d.index.begin(); e != d.index.end(); ++e) {
        if (e.key()[0] == '#' && e.key().find('%') == std::string::npos)
            d.model_ids[e.value().get<std::string>()] = std::stoull(e.key().substr(1), nullptr, 16);
        if (e.key().rfind("P3D-SCN", 0) == 0)
            d.schema_ids[e.value().get<std::string>()] = std::stoull(e.key().substr(7));
    }
    struct Parsed {
        Json objects = Json::array(), native = Json::array(), graphics = Json::array(),
             bindings = Json::array(), relations = Json::array();
    };
    std::vector<Parsed> parsed(d.streams.size());
    std::atomic<std::size_t> work{0};
    std::exception_ptr error;
    std::mutex mu;
    auto worker = [&]() {
        while (true) {
            auto i = work.fetch_add(1);
            if (i >= d.streams.size())
                break;
            try {
                auto &s = d.streams[i];
                s.decoded = s.raw;
                for (std::size_t off : {0, 12, 16}) {
                    if (s.raw->size() < off + 9)
                        continue;
                    Reader r(*s.raw, off);
                    auto n = r.u32(), c = r.u32();
                    if (!n || n > opt.max_stream_bytes || !c || c > r.left())
                        continue;
                    try {
                        s.decoded = std::make_shared<Bytes>(lz(r.take(c), n, opt.max_stream_bytes));
                        s.compression_offset = off + 8;
                        break;
                    } catch (const std::exception &) {
                    }
                }
                auto &v = parsed[i];
                const auto &b = *s.decoded;
                auto canonical = [&](const char *name) {
                    return d.index.value(name, std::string());
                };
                if (s.path.size() >= 3 && d.schema_ids.contains(s.path[2])) {
                    const Bytes magic = {'P', '3', 'D', 'D', 'E', 'X', 0, 255, 10, 13};
                    std::size_t pos = 0;
                    while (pos < b.size()) {
                        auto found =
                            std::search(b.begin() + pos, b.end(), magic.begin(), magic.end());
                        if (found == b.end())
                            break;
                        pos = found - b.begin();
                        require(pos >= 30, "DEX missing prefix");
                        auto start = pos - 30;
                        Reader r(b, start + 8);
                        auto len = r.u32();
                        auto id = r.u64();
                        auto o = parse_dex(b, pos, opt.decode_binary_fields);
                        require(pos == start + 20 + len, "DEX record length");
                        o.update({{"class_id", d.schema_ids[s.path[2]]},
                                  {"object_id", id},
                                  {"stream", s.path},
                                  {"offset", start},
                                  {"length", pos - start},
                                  {"prefix", rawbytes(slice(b, start, 30))}});
                        v.objects.push_back(std::move(o));
                    }
                }
                bool nat = false;
                for (auto key : {"P3D-SMG", "P3D-SMC", "P3D-SSYS"})
                    if (std::find(s.path.begin(), s.path.end(), canonical(key)) != s.path.end())
                        nat = true;
                if (nat && !b.empty() && b.size() != 16) {
                    v.native = parse_native(b);
                    for (auto &x : v.native)
                        x["stream"] = s.path;
                }
                if (b.size() >= 4 && b[0] == 0x1b && b[1] == 0xa1 && b[2] == 0 && b[3] == 0) {
                    v.graphics = parse_graphics(b);
                    for (auto &x : v.graphics)
                        x["stream"] = s.path;
                }
                if (s.path ==
                    StreamPath{canonical("P3D-SD"), canonical("P3D-SMAP"), canonical("P3D-SB1")}) {
                    Reader r(b);
                    auto count = r.u32();
                    require(r.left() == std::uint64_t(count) * 28, "bindings size");
                    for (unsigned j = 0; j < count; ++j) {
                        auto f = r.u32(), e = r.u32(), m = r.u32();
                        auto o = r.u64(), sc = r.u64();
                        v.bindings.push_back({{"flags", f},
                                              {"element_id", e},
                                              {"model_id", m},
                                              {"object_id", o},
                                              {"schema_id", sc}});
                    }
                }
                if (s.path.size() == 4 && s.path[1] == canonical("P3D-SREP") &&
                    s.path.back() == canonical("P3D~MELE1") && b.size() >= 60) {
                    v.relations = parse_relationships(b);
                    for (auto &x : v.relations)
                        x["stream"] = s.path;
                }
            } catch (const std::exception &ex) {
                std::lock_guard<std::mutex> guard(mu);
                if (!error) {
                    std::string name;
                    for (auto &part : d.streams[i].path)
                        name += "/" + part;
                    error = std::make_exception_ptr(std::runtime_error(name + ": " + ex.what()));
                }
            }
        }
    };
    unsigned nt = std::min<unsigned>(std::min(64u, std::max(1u, opt.threads)),
                                     std::max(std::size_t(1), d.streams.size()));
    std::vector<std::thread> threads;
    for (unsigned i = 1; i < nt; ++i) {
        try {
            threads.emplace_back(worker);
        } catch (const std::system_error &) {
            break;
        }
    }
    worker();
    for (auto &t : threads)
        t.join();
    if (error)
        std::rethrow_exception(error);
    for (auto &v : parsed) {
        for (auto &x : v.objects)
            d.objects.push_back(std::move(x));
        for (auto &x : v.native)
            d.native.push_back(std::move(x));
        for (auto &x : v.graphics)
            d.graphics.push_back(std::move(x));
        for (auto &x : v.bindings)
            d.bindings.push_back(std::move(x));
        for (auto &x : v.relations)
            d.relations.push_back(std::move(x));
    }
    d.schemas = read_schemas(*this);
    d.models = read_models(*this);
    d.materials = read_materials(*this);
    d.layers = build_layer_tables(d.index, d.native, d.graphics, d.models);
    for (auto &g : d.graphics)
        for (auto &a : g["attributes"])
            if (a["group"] == 0 && a["key"] == 22902) {
                auto item = a["decoded"];
                item.update({{"stream", g["stream"]},
                             {"record_id", g["id"]},
                             {"attribute_index", a["index"]}});
                d.colors.push_back(std::move(item));
            }
}
const std::vector<Stream> &Document::streams() const {
    return impl_->streams;
}
#define ACCESSOR(name, member)                                                                     \
    const Json &Document::name() const {                                                           \
        return impl_->member;                                                                      \
    }
ACCESSOR(index, index)
ACCESSOR(file_header, file_header)
ACCESSOR(models, models)
ACCESSOR(layer_tables, layers)
ACCESSOR(objects, objects)
ACCESSOR(native_records, native)
ACCESSOR(graphics_records, graphics) ACCESSOR(bindings, bindings) ACCESSOR(relationships, relations)
    ACCESSOR(materials, materials) ACCESSOR(schemas, schemas)
        ACCESSOR(color_tables, colors) Json Document::object_graph() const {
    return build_object_graph(*this);
}
Json Document::scene(unsigned n) const {
    return build_scene(*this, n);
}
NativeScene Document::native_scene(Tessellation policy) const {
    return build_native_scene(*this, policy, impl_->options.threads);
}
Json Document::summary() const {
    std::size_t cmds = 0;
    for (auto &g : graphics_records())
        for (auto &a : g["attributes"])
            if (a["decoded"].contains("commands"))
                cmds += a["decoded"]["commands"].size();
    return {{"streams", streams().size()},
            {"objects", objects().size()},
            {"native_records", native_records().size()},
            {"graphics_records", graphics_records().size()},
            {"commands", cmds},
            {"bindings", bindings().size()},
            {"relationships", relationships().size()},
            {"materials", materials().at("definitions").size()},
            {"schemas", schemas().size()},
            {"models", models().size()}};
}
Json leaf_fields(const Json &n) {
    Json r = Json::object();
    if (n.contains("children"))
        for (auto &c : n["children"])
            r[c.at("name").get<std::string>()] = c.value("value", Json());
    return r;
}
} // namespace p3d
