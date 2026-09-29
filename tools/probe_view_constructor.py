#!/usr/bin/env python3
"""Observe original R1.18 view construction on bounded synthetic input.

The input entity has no edit overlay or linkages. Its model ID is -1, and
the parent is a resident synthetic file system context with the original
vtable. Optional attributes are all consumed by the original reader, so no
attribute-copy allocation or host/model callback is reached. No synthetic
object destructor is called. This does not select an active GUI view.
"""
import argparse
import base64
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import struct

HASHES = {
    'ROOT/P3DKJ.dll': '37fc96b97d230ba31619af4fba09ee11b2a38bfffca616621f11b240079f5901',
    'ROOT/P3DKJJC.dll': 'd5735b0f6c706beafb5feb2a21933404c70eb0371d8d208bf48867a4c43a313c',
    'ROOT/P3DDC.dll': '1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03',
    'SHARE/p3dlibxml2.dll': 'ca24cc124168dd90958f79e72d2af9dbca1dd937338ce6dd07fb0500cf21270a',
}


def probe(root, corpus_report=None):
    if os.name != 'nt' or C.sizeof(C.c_void_p) != 8:
        raise RuntimeError('Requires Windows x64')
    for name, expected in HASHES.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Unsupported DLL: {name}')
    directories = [os.add_dll_directory(str(root/d)) for d in
                   ['ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM']]
    try:
        dll = C.WinDLL(str(root/'ROOT/P3DKJ.dll'))
        ctor = C.CFUNCTYPE(C.c_void_p, C.c_void_p, C.c_void_p)(dll._handle+0x4b46b0)
        file = C.create_string_buffer(0x1000)
        context = C.addressof(file)+0x6c8
        C.c_void_p.from_buffer(file, 0x6c8).value = dll._handle+0x535e10
        C.c_void_p.from_buffer(file, 0x768).value = C.addressof(file)

        def run(rgb, first, second, attributes):
            # Zero preheader: entity-20 kind != 1, hence no editable overlay.
            entity = C.create_string_buffer(0x20+0x48+0x120)
            address = C.addressof(entity)+0x20
            body = bytearray(0x120)
            struct.pack_into('<III', body, 0, 11, 0x90, 0x90)  # empty linkage span
            struct.pack_into('<I', body, 0xc, 1)
            struct.pack_into('<II', body, 0xf8, first, second)
            struct.pack_into('<i', body, 0x10c, -1)
            body[0x110:0x113] = bytes(rgb)
            body[0x118:0x11b] = bytes([61, 113, 197])
            body[0x11c:0x11f] = bytes([199, 127, 59])
            C.memmove(address+0x48, bytes(body), len(body))
            attrs, payloads = [], []
            for key, index, data in attributes:
                attr, payload = C.create_string_buffer(24), C.create_string_buffer(data)
                struct.pack_into('<III', attr, 0, key, index, len(data))
                C.c_void_p.from_buffer(attr, 16).value = C.addressof(payload)
                attrs.append(attr)
                payloads.append(payload)
            pointers = (C.c_void_p*len(attrs))(*(C.addressof(a) for a in attrs))
            collection, record = C.create_string_buffer(32), C.create_string_buffer(48)
            for offset, n in [(8, 0), (16, len(attrs)), (24, len(attrs))]:
                C.c_void_p.from_buffer(collection, offset).value = C.addressof(pointers)+8*n
            vtable = (C.c_void_p*8)()
            vtable[7] = dll._handle+0x6570  # original kind-test constant one
            C.c_void_p.from_buffer(record).value = C.addressof(vtable)
            C.c_void_p.from_buffer(record, 40).value = C.addressof(collection)
            if attributes:
                C.c_void_p.from_address(address+0x28).value = C.addressof(record)
            wrapper, output = C.create_string_buffer(48), C.create_string_buffer(0x190)
            C.c_void_p.from_buffer(wrapper, 0x10).value = context
            C.c_void_p.from_buffer(wrapper, 0x18).value = address
            assert ctor(output, wrapper) == C.addressof(output)
            assert C.c_void_p.from_buffer(output, 0xe0).value == C.addressof(file)
            assert C.c_int32.from_buffer(output, 0xe8).value == -1
            assert not C.c_void_p.from_buffer(output, 0x160).value
            return {'source_rgb': rgb, 'source_words': [first, second],
                    'attributes': [{'key': k, 'index': i, 'hex': b.hex()} for k, i, b in attributes],
                    'constructor_rgb': list(output.raw[0xec:0xef]),
                    'adjacent_rgb_bytes': list(output.raw[0xef:0xf5]),
                    'constructor_words': list(struct.unpack_from('<II', output, 0x10)),
                    'display_style_index': C.c_int32.from_buffer(output, 0x148).value}

        rows = [run([i, 255-i, i*73 % 256], ((i % 64) << 23) | 0x60402135,
                    (i*0x01010101) & 0xffffffff, []) for i in range(256)]
        key = 0x4e700000
        four = lambda value: struct.pack('<i', value)
        cases = [[(key, 0, four(7))], [(key, 0, four(-1))], [(key, 0, b'bad')],
                 [(key, 0, four(7)), (key, 0, b'bad')],
                 [(key, 0, b'bad'), (key, 0, four(9))],
                 [(0x4e520000, 0, bytes(range(36))), (key, 0, four(19)),
                  (0x59640000, 0, four(101)), (0x4e720000, 1, four(37))]]
        rows.extend(run([17, 29, 43], 0xffffffff, 0xffffffff, attrs) for attrs in cases)
        for row in rows:
            assert row['constructor_rgb'] == row['source_rgb']
            assert row['adjacent_rgb_bytes'] == [61, 113, 197, 199, 127, 59]
        assert [r['display_style_index'] for r in rows[-6:]] == [7, -1, -1, -1, 9, 19]
        corpus_cases = []
        if corpus_report:
            report = json.loads(corpus_report.read_text(encoding='utf8'))
            for file_row in report['files']:
                if file_row['execution_status'] != 'completed':
                    raise ValueError('Incomplete corpus audit')
                document = json.loads((Path(file_row['output'])/'document.json').read_text(encoding='utf8'))
                for table in document['initial_display_style_tables']:
                    for ref in table['reference_inputs']['references']:
                        record = document['native_records'][ref['native_record_index']]
                        data = base64.b64decode(record['data']['base64'], validate=True)
                        if record['element_type'] != 11 or len(data) < 0x123 or struct.unpack_from('<I', data, 16)[0] != 1:
                            raise ValueError('Unsupported view input')
                        # Only the observed RGB/flag fields enter the bounded
                        # synthetic wrapper; real model lookup is not exercised.
                        row = run(list(data[0x114:0x117]), *struct.unpack_from('<II', data, 0xfc), [])
                        row.update({'source_file_sha256': file_row['sha256'],
                                    'native_record_index': ref['native_record_index'],
                                    'input_occurrence_index': ref['input_occurrence_index'],
                                    'source_model_id': struct.unpack_from('<i', data, 0x110)[0],
                                    'synthetic_model_id': -1})
                        corpus_cases.append(row)
        return {'scope': 'original_type11_constructor_no_linkages_system_model_no_host',
                'dll_sha256': HASHES, 'constructor_rva': '0x4b46b0',
                'body_rgb_offset': 0x110, 'stream_rgb_offset': 0x114,
                'active_view_selection': 'not_evaluated', 'cases': rows,
                'corpus_field_copy_cases': corpus_cases}
    finally:
        for directory in directories:
            directory.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fixture', type=Path)
    parser.add_argument('--corpus-report', type=Path)
    args = parser.parse_args()
    result = probe(args.dll_root.resolve(), args.corpus_report)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf8')
    if args.fixture:
        # Numeric observations and synthetic inputs only; no proprietary bytes.
        fixture = {k: v for k, v in result.items() if k != 'corpus_field_copy_cases'}
        args.fixture.write_text('#pragma once\n// Original R1.18 numeric constructor observations.\n'
                                'inline constexpr const char* view_constructor_oracle = R"oracle('
                                +json.dumps(fixture, separators=(',', ':'))+')oracle";\n', encoding='utf8')
    print(f"Verified {len(result['cases'])} synthetic and {len(result['corpus_field_copy_cases'])} corpus-field constructors: {args.output}")


if __name__ == '__main__':
    main()
