#!/usr/bin/env python3
"""Probe the original R1.18 exact 20080/index-0 composite attribute query.

Synthetic read-only record entity and preallocated attribute pointer storage;
no host, editable overlay, complete object constructor or rendering is invoked.
The record's kind-test slot points to the original constant-one function.
"""
import argparse
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import struct

HASHES = {
    "ROOT/P3DKJ.dll": "37fc96b97d230ba31619af4fba09ee11b2a38bfffca616621f11b240079f5901",
    "ROOT/P3DKJJC.dll": "d5735b0f6c706beafb5feb2a21933404c70eb0371d8d208bf48867a4c43a313c",
    "ROOT/P3DDC.dll": "1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03",
    "SHARE/p3dlibxml2.dll": "ca24cc124168dd90958f79e72d2af9dbca1dd937338ce6dd07fb0500cf21270a",
}


def probe(root):
    if os.name != 'nt' or C.sizeof(C.c_void_p) != 8:
        raise RuntimeError('Requires Windows x64 Python')
    for name, expected in HASHES.items():
        if hashlib.sha256((root / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Unsupported DLL version: {name}')
    directories = [os.add_dll_directory(str(root / name))
                   for name in ['ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM']]
    try:
        dll = C.WinDLL(str(root / 'ROOT/P3DKJ.dll'))
        query = C.CFUNCTYPE(C.c_void_p, C.c_void_p, C.c_void_p, C.c_uint32,
                           C.c_uint32)(dll._handle + 0x3fe9f0)

        def run(spec):
            attributes, payloads = [], []
            for key, index, data in spec:
                attribute, payload = C.create_string_buffer(24), C.create_string_buffer(data)
                C.c_uint32.from_buffer(attribute, 0).value = key
                C.c_uint32.from_buffer(attribute, 4).value = index
                C.c_uint32.from_buffer(attribute, 8).value = len(data)
                C.c_void_p.from_buffer(attribute, 16).value = C.addressof(payload)
                attributes.append(attribute)
                payloads.append(payload)
            addresses = [C.addressof(a) for a in attributes]
            pointers = (C.c_void_p * len(attributes))(*addresses)
            collection = C.create_string_buffer(32)
            for offset, count in [(8, 0), (16, len(attributes)), (24, len(attributes))]:
                C.c_void_p.from_buffer(collection, offset).value = C.addressof(pointers) + 8 * count
            vtable = (C.c_void_p * 8)()
            vtable[7] = dll._handle + 0x6570
            record, handle, output = (C.create_string_buffer(n) for n in [48, 48, 64])
            C.c_void_p.from_buffer(record, 0).value = C.addressof(vtable)
            C.c_void_p.from_buffer(record, 40).value = C.addressof(collection)
            C.c_void_p.from_buffer(handle, 0).value = C.addressof(record)
            query(output, handle, 0x4e700000, 0)
            state = C.c_uint32.from_buffer(output, 56).value
            selected = None
            if state == 2:
                iterator = C.c_void_p.from_buffer(output, 24).value
                if iterator:
                    selected = addresses.index(C.c_void_p.from_address(iterator).value)
            return {'attributes': [{'key': k, 'index': i, 'hex': b.hex()} for k, i, b in spec],
                    'state': state, 'selected_source_ordinal': selected,
                    'sorted_source_ordinals': [addresses.index(p) for p in pointers]}

        key = 0x4e700000
        def four(value):
            return struct.pack('<i', value)
        cases = [[], [(key, 0, four(9))],
                 [(key, 0, four(9)), (key, 0, b'bad')],
                 [(key, 0, b'bad'), (key, 0, four(-1))],
                 [(key, 0, four(9)), (key, 1, four(10))],
                 [(key, 0, b'bad'), (key, 0, four(9))] +
                 [(0x4e720000, i, b'x') for i in range(4)]]
        rows = [run(case) for case in cases]
        expected = [(3, None), (2, 0), (2, 1), (2, 1), (2, 0), (2, 0)]
        for row, pair in zip(rows, expected):
            if (row['state'], row['selected_source_ordinal']) != pair:
                raise RuntimeError(f'Unexpected original query result: {row}')
        return {'scope': 'original_query_with_synthetic_readonly_record_no_edit_overlay',
                'dll_sha256': HASHES, 'query_rva': '0x3fe9f0',
                'whole_reader_execution': 'not_evaluated', 'cases': rows}
    finally:
        for directory in directories:
            directory.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = probe(args.dll_root.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(f"Verified {len(result['cases'])} original attribute queries: {args.output}")


if __name__ == '__main__':
    main()
