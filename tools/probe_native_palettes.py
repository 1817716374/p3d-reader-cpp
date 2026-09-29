#!/usr/bin/env python3
"""Observe R1.18 palette values, constructor order and header selector priority.

Execute only original hash-gated DLLs. General 1e4dd0 selectors are NOT called:
those branches require host TLS. 1e3e50 observes the pre-host palette only.
A handful of native palette objects are reclaimed by this short process exit.
"""
import argparse
import ctypes as C
import hashlib
import itertools
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
        choose = C.CFUNCTYPE(C.c_uint32, C.c_void_p)(dll._handle + 0x1313b0)
        builtin = C.CFUNCTYPE(C.c_void_p, C.c_void_p, C.c_uint32)(dll._handle + 0x1e4dd0)
        pre_host = C.CFUNCTYPE(C.c_void_p, C.c_void_p)(dll._handle + 0x1e3e50)
        construct = C.CFUNCTYPE(C.c_void_p, C.c_void_p, C.c_void_p)(dll._handle + 0x1e4ba0)
        def read(function, *args):
            out = C.c_void_p()
            address = function(C.byref(out), *args)
            assert address == C.addressof(out) and out.value
            # Native palette +0x10 contains 256 packed RGBA DWORDs.
            assert all(C.c_ubyte.from_address(out.value+19+4*i).value == 0 for i in range(256))
            return [list(C.string_at(out.value+16+4*i, 3)) for i in range(256)]
        native_default = read(pre_host)
        observations = [read(builtin, selector) for selector in [3,4,3,4]]
        assert all(rgb == observations[0] for rgb in observations)
        source = (C.c_ubyte * 768)(*[(i*73+19) % 256 for i in range(768)])
        copied = read(construct, source)
        assert copied == [list(source[3*((i+1)%256):3*((i+1)%256)+3]) for i in range(256)]
        rows = []
        values = [0,1,2,3,4,5,0x7fffffff,0x80000000,0xffffffff]
        for base, override in itertools.product(values, repeat=2):
            file = C.create_string_buffer(0x380)
            C.c_uint32.from_buffer(file, 0x184).value = base
            C.c_uint32.from_buffer(file, 0x37c).value = override
            before = file.raw
            selector = choose(file)
            assert file.raw == before
            rows.append({'base': base, 'override': override, 'selector': selector})
        return {'pre_host_rgb': native_default, 'builtin_rgb': observations[0], 'selectors': rows,
                'constructor_rotates_source_left_one': True, 'repeated_builtin_values_equal': True}
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
    args.output.write_text(json.dumps({'dll_sha256': HASHES, **rows}, indent=2), encoding='utf8')
    if args.fixture:
        body = json.dumps(rows, separators=(',', ':'))
        chunks = ['R"P3D(' + body[i:i+8192] + ')P3D"' for i in range(0,len(body),8192)]
        args.fixture.write_text('#pragma once\n// Original R1.18 numeric palette observations.\n'
            + 'inline constexpr const char *native_palette_oracle_parts[] = {\n'
            + ',\n'.join(chunks) + '\n};\n', encoding='utf8')
    print(f'Recorded two 256-color palettes and {len(rows["selectors"])} selector queries')


if __name__ == '__main__':
    main()
