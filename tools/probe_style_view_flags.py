#!/usr/bin/env python3
"""Observe original R1.18 daa40 flag writes using synthetic fixed buffers.

This leaf function reads only style DWORDs +48/+50 and QWORD +78 and writes
two destination DWORDs. No host, XML importer or resource resolution is run.
"""
import argparse
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import random

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
        apply = C.CFUNCTYPE(None, C.c_void_p, C.c_void_p)(dll._handle + 0xdaa40)
        cases = []
        for initial in [0, 0xffffffff]:
            for bit in range(32):
                cases.append([initial, initial, 1 << bit, 0, 0])
                cases.append([initial, initial, 0, 1 << bit, 0])
        for material in [0, 1, 0x100000000, 0xffffffffffffffff]:
            cases.append([0xdeadbeef, 0xffffffff, 0, 0x40, material])
        for bit in range(32):
            cases.append([1 << bit, 1 << bit, 0, 0, 0])
        randomizer = random.Random(0xdaa40)
        for _ in range(128):
            cases.append([randomizer.getrandbits(32) for _ in range(4)] +
                         [randomizer.choice([0, randomizer.getrandbits(64)])])
        rows = []
        for first, second, flags48, flags50, material in cases:
            style = C.create_string_buffer(0x80)
            C.c_uint32.from_buffer(style, 0x48).value = flags48
            C.c_uint32.from_buffer(style, 0x50).value = flags50
            C.c_uint64.from_buffer(style, 0x78).value = material
            # Guard words establish that the original function only updates
            # the documented eight-byte destination range for these inputs.
            destination = (C.c_uint32 * 4)(0x11223344, first, second, 0xaabbccdd)
            before = bytes(style)
            apply(style, C.byref(destination, 4))
            if bytes(style) != before or destination[0] != 0x11223344 or destination[3] != 0xaabbccdd:
                raise RuntimeError('Original function changed source or destination guards')
            rows.append([first, second, flags48, flags50, material, destination[1], destination[2]])
        return {'scope': 'original_daa40_pure_flag_application', 'dll_sha256': HASHES,
                'columns': ['first', 'second', 'flags48', 'flags50', 'material', 'out_first', 'out_second'],
                'cases': rows}
    finally:
        for directory in directories:
            directory.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--fixture', type=Path, help='Optional C++ synthetic oracle header')
    args = parser.parse_args()
    result = probe(args.dll_root.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    if args.fixture:
        lines = ['#pragma once', '#include <cstdint>', '',
                 '// Original R1.18 daa40 observations; synthetic inputs only.',
                 '// first, second, flags48, flags50, material, out_first, out_second',
                 'inline constexpr std::uint64_t style_view_flag_oracle[][7] = {']
        lines += ['    {' + ', '.join(f'0x{value:x}ull' for value in row) + '},' for row in result['cases']]
        lines += ['};', '']
        args.fixture.write_text('\n'.join(lines), encoding='utf-8')
    print(f"Recorded {len(result['cases'])} original flag applications: {args.output}")


if __name__ == '__main__':
    main()
