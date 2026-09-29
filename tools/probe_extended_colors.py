#!/usr/bin/env python3
"""Execute original R1.18 extended-color XML import on synthetic empty caches.

Preallocated slot storage and an empty MSVC tree sentinel replace the file
cache. The original importer allocates map/string nodes; this bounded probe
relies on process exit for reclamation and never destroys the synthetic cache.
No file context, resource registration or background application is invoked.
"""
import argparse
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
from xml.sax.saxutils import quoteattr

HASHES = {'ROOT/P3DKJ.dll': '37fc96b97d230ba31619af4fba09ee11b2a38bfffca616621f11b240079f5901', 'ROOT/P3DKJJC.dll': 'd5735b0f6c706beafb5feb2a21933404c70eb0371d8d208bf48867a4c43a313c', 'ROOT/P3DDC.dll': '1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03', 'SHARE/p3dlibxml2.dll': 'ca24cc124168dd90958f79e72d2af9dbca1dd937338ce6dd07fb0500cf21270a'}


def probe(root, xmls):
    if os.name != 'nt' or C.sizeof(C.c_void_p) != 8:
        raise RuntimeError('Requires Windows x64 Python')
    for name, expected in HASHES.items():
        if hashlib.sha256((root / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Unsupported DLL version: {name}')
    directories = [os.add_dll_directory(str(root / d))
                   for d in ['ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM']]
    try:
        dll = C.WinDLL(str(root / 'ROOT/P3DKJ.dll'))
        read = C.CFUNCTYPE(C.c_int, C.c_void_p, C.c_void_p, C.c_uint64)(dll._handle + 0x2665b0)
        def string(address):
            size = C.c_uint64.from_address(address + 16).value
            capacity = C.c_uint64.from_address(address + 24).value
            if size > 100000: raise RuntimeError('Unexpected native string length')
            data = C.c_void_p.from_address(address).value if capacity >= 8 else address
            return C.string_at(data, size * 2).decode('utf-16le')
        rows = []
        for xml in xmls:
            capacity = xml.count('<') + 1
            if capacity > 10000 or len(xml) > 2000000:
                raise ValueError('Probe input exceeds bounded capacity')
            cache, slots, head = (C.create_string_buffer(n) for n in [64, 16 * capacity, 40])
            h = C.addressof(head)
            for offset in [0, 8, 16]: C.c_void_p.from_buffer(head, offset).value = h
            C.c_uint16.from_buffer(head, 24).value = 0x101
            for offset, value in [(8, C.addressof(slots)), (16, C.addressof(slots)),
                                  (24, C.addressof(slots) + len(slots)), (32, h)]:
                C.c_void_p.from_buffer(cache, offset).value = value
            data = C.create_unicode_buffer(xml)
            status = read(cache, data, len(xml.encode('utf-16le')))
            count = (C.c_void_p.from_buffer(cache, 16).value - C.addressof(slots)) // 16
            if not 0 <= count <= capacity: raise RuntimeError('Unexpected native slot count')
            entries = []
            for i in range(count):
                pair = C.c_void_p.from_buffer(slots, 16 * i + 8).value
                entries.append({'rgb': list(slots.raw[16*i:16*i+3]),
                    'book_name': {'book': string(pair), 'name': string(pair + 32)} if pair else None})
            pending = [C.c_void_p.from_buffer(head, 8).value]; mapping = []
            while pending:
                node = pending.pop()
                if node == h: continue
                if len(mapping) >= count: raise RuntimeError('Unexpected native tree topology')
                mapping.append({'rgb_key': C.c_uint32.from_address(node + 28).value,
                                'native_index': C.c_uint32.from_address(node + 32).value})
                pending.extend([C.c_void_p.from_address(node).value,
                                C.c_void_p.from_address(node + 16).value])
            rows.append({'xml': xml, 'native_status': status, 'entries': entries,
                         'rgb_lookup': sorted(mapping, key=lambda item: item['rgb_key'])})
        return rows
    finally:
        for directory in directories: directory.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--fixture', type=Path)
    parser.add_argument('--corpus-report', type=Path)
    args = parser.parse_args()
    values = ['bad', '(1)', '(1,2)', '(256,-2,999)', ' (1,2,3)', '( 1, 2, 3)',
              '(1 ,2,3)', '(1,2,3)tail', '(1,2,3', '(1,2,3,4)', '(0x10,2,3)',
              '(+1,-2,003)', '(2147483648,4294967296,-4294967297)',
              '(9223372036854775808,18446744073709551616,-9223372036854775809)',
              '(18446744073709551615,-18446744073709551615,9999999999999999999999999999999)']
    xmls = ['<Colors><Entry Color=' + quoteattr(v) + '/></Colors>' for v in values]
    xmls += ['<Entry Color="(1,2,3)"/>',
             '<ExtendedColors><Other Color="(1,2,3)"/><Entry/><Entry Color=""/><Group><Entry Color="(4,5,6)"/></Group></ExtendedColors>',
             '<ExtendedColors xmlns="x"><Entry Color="(1,2,3)"/><Entry xmlns="" Color="(4,5,6)"/></ExtendedColors>',
             '<Colors><Entry Color="(1,2,3)" Book="B" Name="N"/><Entry Color="(1,2,3)" Book="B2"/><Entry Color="(4,5,6)" Name="N2"/></Colors>',
             '<x:Colors xmlns:x="x"><x:Entry Color="(1,2,3)"/><Entry Color="(7,8,9)" Book="长书名" Name="长颜色名称"/></x:Colors>']
    synthetic_count = len(xmls)
    if args.corpus_report:
        for file in json.loads(args.corpus_report.read_text(encoding='utf8'))['files']:
            doc = json.loads((Path(file['output']) / 'document.json').read_text(encoding='utf8'))
            for record in doc['graphics_records']:
                for attr in record['attributes']:
                    if (attr['group'], attr['key'], attr['index']) == (0, 22902, 0):
                        xmls.append(attr['decoded']['xml'])
    rows = probe(args.dll_root.resolve(), xmls)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'dll_sha256': HASHES, 'synthetic_count': synthetic_count,
                                      'cases': rows}, ensure_ascii=False, indent=2), encoding='utf8')
    if args.fixture:
        body = json.dumps(rows[:synthetic_count], ensure_ascii=True, separators=(',', ':'))
        args.fixture.write_text('#pragma once\n// Original 2665b0 observations, synthetic XML only.\n'
            + 'inline constexpr const char *extended_colors_oracle = R"P3D(' + body + ')P3D";\n', encoding='utf8')
    print(f'Recorded {len(rows)} XML imports, {sum(len(r["entries"]) for r in rows)} entries')


if __name__ == '__main__':
    main()
