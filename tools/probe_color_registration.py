#!/usr/bin/env python3
"""Observe original R1.18 HSV, palette search and extended-color registration.

Runs only SHA256-gated original DLLs on bounded synthetic inputs. Fake file
storage has BOTH native caches populated, avoiding file/provider loading. Slot
buffers never grow. Native tree, string and DOM allocations live until this
short process exits; never run cache/file destructors on these synthetic objects.
"""
import argparse
import ctypes as C
import hashlib
import itertools
import json
import os
from pathlib import Path
import random

HASHES = {'ROOT/P3DKJ.dll': '37fc96b97d230ba31619af4fba09ee11b2a38bfffca616621f11b240079f5901', 'ROOT/P3DKJJC.dll': 'd5735b0f6c706beafb5feb2a21933404c70eb0371d8d208bf48867a4c43a313c', 'ROOT/P3DDC.dll': '1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03', 'SHARE/p3dlibxml2.dll': 'ca24cc124168dd90958f79e72d2af9dbca1dd937338ce6dd07fb0500cf21270a'}


def probe(root):
    if os.name != 'nt' or C.sizeof(C.c_void_p) != 8:
        raise RuntimeError('Requires Windows x64 Python')
    for name, expected in HASHES.items():
        if hashlib.sha256((root / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Unsupported DLL version: {name}')
    directories = [os.add_dll_directory(str(root / d))
                   for d in ['ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM']]
    try:
        dll = C.WinDLL(str(root / 'ROOT/P3DKJ.dll'))
        hsv = C.CFUNCTYPE(None, C.c_void_p, C.c_void_p)(dll._handle + 0x1e2b40)
        nearest = C.CFUNCTYPE(C.c_int, C.c_void_p, C.c_void_p, C.c_void_p)(dll._handle + 0x1e4500)
        read = C.CFUNCTYPE(C.c_int, C.c_void_p, C.c_void_p, C.c_uint64)(dll._handle + 0x2665b0)
        register = C.CFUNCTYPE(C.c_uint32, C.c_void_p, C.c_void_p, C.c_void_p,
                              C.c_void_p, C.c_bool)(dll._handle + 0x1e41c0)
        rng = random.Random(0x1e4500)
        colors = set(itertools.product(range(0, 256, 17), repeat=3))
        colors.update((i, i, i) for i in range(256))
        for i in range(256):
            for d in [-3, -2, -1, 1, 2, 3]:
                if 0 <= i+d <= 255:
                    colors.update(itertools.permutations((i, i, i+d)))
        colors.update(tuple(rng.randrange(256) for _ in range(3)) for _ in range(512))
        hsv_rows = []
        for rgb in sorted(colors):
            source = (C.c_ubyte * 3)(*rgb)
            result = (C.c_int * 5)(0x12345678, 0, 0, 0, 0x12345678)
            hsv(C.byref(result, 4), source)
            assert result[0] == result[4] == 0x12345678 and list(source) == list(rgb)
            hsv_rows.append({'rgb': rgb, 'hsv': list(result)[1:4]})
        palettes = [[[i, i, i] for i in range(256)], [[255-i]*3 for i in range(256)]]
        special = [[255]*3 for _ in range(256)]
        special[0], special[1], special[255] = [0]*3, [20]*3, [255, 0, 0]
        palettes.append(special)
        palettes.append([[(i*47) % 256, (i*73) % 256, (i*137) % 256] for i in range(256)])
        # Ties, exact match at excluded index 255, and hue wrap.
        palettes.append([[255, 0, 1], [255, 1, 0]] + [[0, 255, 0]]*253 + [[255, 0, 0]])
        palettes.extend([[list(rng.choices(range(256), k=3)) for _ in range(256)] for _ in range(3)])
        queries = [[0,0,0], [1,1,1], [2,2,2], [255,0,0], [254,255,255], [250,255,255],
                   [248,255,255], [255,254,255], [255,255,254], [255,0,1], [255,1,0]]
        queries += [list(rng.choices(range(256), k=3)) for _ in range(128)]
        palette_buffers, palette_rows = [], []
        for palette in palettes:
            data = C.create_string_buffer(0x410)
            for i, rgb in enumerate(palette):
                for c, value in enumerate(rgb): data[16 + 4*i + c] = value
                data[16 + 4*i + 3] = 0xa5  # ignored fourth byte
            before = data.raw
            cases = []
            for rgb in queries:
                source = (C.c_ubyte * 3)(*rgb)
                index = nearest(data, source, None)
                assert data.raw == before and list(source) == rgb and 0 <= index < 255
                cases.append({'rgb': rgb, 'index': index})
            palette_buffers.append(data)
            palette_rows.append({'palette': palette, 'queries': cases})

        def string(address):
            size = C.c_uint64.from_address(address + 16).value
            capacity = C.c_uint64.from_address(address + 24).value
            if size > 100000: raise RuntimeError('Unexpected native string length')
            pointer = C.c_void_p.from_address(address).value if capacity >= 8 else address
            return C.string_at(pointer, size * 2).decode('utf-16le')

        def snapshot(cache, slots):
            count = (C.c_void_p.from_buffer(cache, 16).value - C.addressof(slots)) // 16
            assert 0 <= count <= len(slots)//16
            entries = []
            for i in range(count):
                pair = C.c_void_p.from_buffer(slots, 16*i + 8).value
                entries.append({'rgb': list(slots.raw[16*i:16*i+3]),
                    'book_name': {'book': string(pair), 'name': string(pair + 32)} if pair else None})
            return {'entries': entries, 'dirty': bool(cache.raw[56])}

        queries = [
            ([1,2,3], None, None, False), ([1,2,3], '', 'Name', False),
            ([1,2,3], 'Book', 'Name', False), ([1,2,3], 'Other', 'Other', True),
            ([4,5,6], None, 'Name', True), ([4,5,6], 'Book', None, True),
            ([4,5,6], 'Book', 'Name', False),
            ([7,8,9], None, None, False), ([7,8,9], '', '', True),
            ([7,8,9], 'Replacement', 'Replacement', False),
            ([10,11,12], None, '', True), ([10,11,12], '', 'Name', True),
            ([10,11,12], '长书名测试abcdefghi', '长颜色名称测试abcdefghi', True),
            ([13,14,15], 'B\0ignored', 'N\0ignored', True),
            ([16,17,18], '\0ignored', 'N', True),
            ([16,17,18], 'Replacement', 'Replacement', True),
            ([255,0,0], 'B', 'N', False), ([255,0,0], 'B', 'N', True)]
        xmls = [None, '<Colors/>', '<Colors><Entry Color="(1,2,3)"/><Entry Color="(1,2,3)"/></Colors>',
                '<Colors><Entry Color="(1,2,3)" Book="Existing" Name="Existing"/>'
                '<Entry Color="(1,2,3)"/><Entry Color="(4,5,6)" Book="Keep" Name="Keep"/></Colors>',
                '<Colors><Other/><Entry Color="(1,2,3)"/><Group><Entry Color="(4,5,6)"/></Group></Colors>']
        registrations = []
        for xml in xmls:
            cache, slots, head, file = (C.create_string_buffer(n) for n in [64, 16*128, 40, 0x1000])
            h = C.addressof(head)
            for offset in [0,8,16]: C.c_void_p.from_buffer(head, offset).value = h
            C.c_uint16.from_buffer(head, 24).value = 0x101
            for offset, value in [(8,C.addressof(slots)), (16,C.addressof(slots)),
                                  (24,C.addressof(slots)+len(slots)), (32,h)]:
                C.c_void_p.from_buffer(cache, offset).value = value
            if xml is not None:
                text = C.create_unicode_buffer(xml)
                assert read(cache, text, len(xml.encode('utf-16le'))) == 0
            C.c_void_p.from_buffer(file, 0xf20).value = C.addressof(palette_buffers[3])
            C.c_void_p.from_buffer(file, 0xf28).value = C.addressof(cache)
            initial = snapshot(cache, slots)
            rows = []
            for rgb, book, name, allow in queries:
                source = (C.c_ubyte * 3)(*rgb)
                b = C.create_unicode_buffer(book) if book is not None else None
                n = C.create_unicode_buffer(name) if name is not None else None
                result = register(source, b, n, file, allow)
                assert list(source) == rgb
                rows.append({'rgb': rgb, 'book': book, 'name': name, 'allow_append': allow,
                             'color_id': result, 'native_index': result >> 8, **snapshot(cache, slots)})
            registrations.append({'xml': xml, 'initial': initial, 'palette': 3, 'queries': rows})
        return {'hsv': hsv_rows, 'palettes': palette_rows, 'registrations': registrations}
    finally:
        for directory in directories: directory.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--fixture', type=Path)
    args = parser.parse_args()
    rows = probe(args.dll_root.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'dll_sha256': HASHES, **rows}, ensure_ascii=True, indent=2), encoding='utf8')
    if args.fixture:
        body = json.dumps(rows, ensure_ascii=True, separators=(',', ':'))
        chunks = ['R"P3D(' + body[i:i+8192] + ')P3D"' for i in range(0, len(body), 8192)]
        args.fixture.write_text('#pragma once\n// Original R1.18 DLL observations; synthetic data only.\n'
            + 'inline constexpr const char *color_registration_oracle_parts[] = {\n'
            + ',\n'.join(chunks) + '\n};\n', encoding='utf8')
    print(f'Recorded {len(rows["hsv"])} HSV values, '
          f'{sum(len(p["queries"]) for p in rows["palettes"])} palette queries, '
          f'{sum(len(r["queries"]) for r in rows["registrations"])} registrations')


if __name__ == '__main__':
    main()
