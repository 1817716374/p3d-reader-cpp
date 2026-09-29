#!/usr/bin/env python3
"""Probe original R1.18 system-context slot-zero selection, without host loading.

Nonempty synthetic lists only: the empty-list allocator path is intentionally
not invoked. This verifies 19aad0/1a6850 selection, not file loading or XML import.
"""
import argparse
import ctypes as C
import hashlib
import json
import os
from pathlib import Path

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
        select = C.CFUNCTYPE(C.c_void_p, C.c_void_p, C.c_uint32, C.c_bool)(dll._handle + 0x19aad0)
        rows = []
        for split in [False, True]:
            for cached in ['none', 'first', 'last']:
                for flags in [0, 8, 0xffffffff]:
                    for first_null in [False, True]:
                        file, table = C.create_string_buffer(0x1000), C.create_string_buffer(0x38)
                        entries = [C.create_string_buffer(0x60) for _ in range(4)]
                        C.c_uint32.from_buffer(entries[0], 16).value = flags
                        addresses = [C.addressof(e) for e in entries]
                        if first_null: addresses[0] = 0
                        blocks = [C.create_string_buffer(0x48) for _ in range(2 if split else 1)]
                        slots = []
                        for i, block in enumerate(blocks):
                            values = addresses[2*i:2*i+2] if split else addresses
                            array = (C.c_void_p * len(values))(*values); slots.append(array)
                            for offset, value in [(0,C.addressof(table)),
                                (8,C.addressof(blocks[i+1]) if i+1 < len(blocks) else 0),
                                (16,C.addressof(blocks[i-1]) if i else 0),
                                (24,C.addressof(array)), (32,C.addressof(array)+C.sizeof(array))]:
                                C.c_void_p.from_buffer(block, offset).value = value
                            C.c_uint32.from_buffer(block, 60).value = i*2 if split else 0
                        for offset, value in [(24,C.addressof(blocks[0])), (32,C.addressof(blocks[-1])),
                            (40,0 if cached == 'none' else C.addressof(blocks[0 if cached == 'first' else -1]))]:
                            C.c_void_p.from_buffer(table, offset).value = value
                        C.c_void_p.from_buffer(file, 0x800).value = C.addressof(table)
                        before = [e.raw for e in entries]
                        value = select(C.addressof(file)+0x6c8, 0, False) or 0
                        assert value == addresses[0] and before == [e.raw for e in entries]
                        rows.append({'split_blocks': split, 'cached_block': cached, 'first_flags': flags,
                                     'first_null': first_null, 'selected_slot': None if not value else 0})
        return rows
    finally:
        for directory in directories: directory.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    rows = probe(args.dll_root.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'dll_sha256': HASHES, 'cases': rows}, indent=2), encoding='utf8')
    print(f'Confirmed {len(rows)} nonempty system-list queries')


if __name__ == '__main__':
    main()
