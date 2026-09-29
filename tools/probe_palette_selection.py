#!/usr/bin/env python3
"""Observe original R1.18 palette selection with the original default service.

A synthetic TLS host contains only the verified +0x90 service. No GUI, file
constructor or model loading is run. All palette objects are native allocations;
only inspected RGB storage and synthetic file fields are supplied by the probe.
"""
import argparse
import ctypes as C
import hashlib
import itertools
import json
import os
from pathlib import Path

HASHES = {
    'ROOT/P3DKJ.dll': '37fc96b97d230ba31619af4fba09ee11b2a38bfffca616621f11b240079f5901',
    'ROOT/P3DKJJC.dll': 'd5735b0f6c706beafb5feb2a21933404c70eb0371d8d208bf48867a4c43a313c',
    'ROOT/P3DDC.dll': '1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03',
    'SHARE/p3dlibxml2.dll': 'ca24cc124168dd90958f79e72d2af9dbca1dd937338ce6dd07fb0500cf21270a',
}


def probe(root):
    if os.name != 'nt' or C.sizeof(C.c_void_p) != 8:
        raise RuntimeError('Requires Windows x64 Python')
    for name, expected in HASHES.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Unsupported DLL version: {name}')
    directories = [os.add_dll_directory(str(root/d))
                   for d in ['ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM']]
    try:
        dll = C.WinDLL(str(root/'ROOT/P3DKJ.dll'))
        base = dll._handle
        def fn(rva, result, *args): return C.CFUNCTYPE(result, *args)(base+rva)
        query = fn(0x12e680, C.c_void_p, C.c_void_p)
        construct = fn(0x1e4cd0, C.c_void_p, C.c_void_p, C.c_void_p)
        prehost = fn(0x1e3e50, C.c_void_p, C.c_void_p)
        builtin = fn(0x1e4dd0, C.c_void_p, C.c_void_p, C.c_uint32)
        clear_global = fn(0x1e4170, None)
        def release(ptr):
            vt = C.c_void_p.from_address(ptr).value
            C.CFUNCTYPE(None, C.c_void_p)(C.c_void_p.from_address(vt+8).value)(ptr)
        def rgb(ptr): return [list(C.string_at(ptr+16+4*i, 3)) for i in range(256)]
        def create(function, *args):
            out = C.c_void_p()
            assert function(C.byref(out), *args) == C.addressof(out) and out.value
            return out.value
        global_slot = C.c_void_p.from_address(base+0x644708)
        if global_slot.value: raise RuntimeError('Requires a fresh process/global palette cache')
        service = fn(0x164050, C.c_void_p, C.c_void_p)(None)
        assert C.c_void_p.from_address(service).value == base+0x530a00
        host = C.create_string_buffer(0x110)
        C.c_void_p.from_buffer(host, 0x90).value = service
        get_host = C.CFUNCTYPE(C.c_void_p, C.c_void_p)(C.c_void_p.from_address(base+0x51e788).value)
        set_host = C.CFUNCTYPE(None, C.c_void_p, C.c_void_p)(C.c_void_p.from_address(base+0x51e5b0).value)
        key = base+0x643db8
        previous = get_host(key)
        palettes = {}
        for name, function, args in [('pre_host', prehost, ()), ('builtin', builtin, (3,))]:
            ptr = create(function, *args)
            palettes[name] = rgb(ptr)
            release(ptr)
        palettes['global_custom'] = [[(i*17+3)%256, (i*31+7)%256, (i*47+11)%256] for i in range(256)]
        palettes['file_custom'] = [[(i*53+13)%256, (i*71+19)%256, (i*89+23)%256] for i in range(256)]
        def packed(colors): return (C.c_ubyte*1024)(*[v for color in colors for v in color+[0]])
        rows = []
        try:
            set_host(key, C.addressof(host))
            assert get_host(key) == C.addressof(host)
            values = [0,1,2,3,4,5,0x7fffffff,0x80000000,0xffffffff]
            for mode, a, b in itertools.product(['fresh', 'global_custom', 'file_custom'], values, values):
                clear_global()
                file = C.create_string_buffer(0xf28)
                file_slot = C.c_void_p.from_buffer(file, 0xf20)
                C.c_uint32.from_buffer(file, 0x184).value = a
                C.c_uint32.from_buffer(file, 0x37c).value = b
                if mode == 'global_custom':
                    # Establish native ownership before changing its RGB values.
                    temp = create(builtin, 2)
                    release(temp)
                    C.memmove(global_slot.value+16, packed(palettes[mode]), 1024)
                elif mode == 'file_custom':
                    file_slot.value = create(construct, packed(palettes[mode]))
                global_before = global_slot.value
                source = ('file_custom' if mode == 'file_custom' else
                          'builtin' if (b or a) in (3,4) else
                          'global_custom' if mode == 'global_custom' else 'pre_host')
                ptr = query(file)
                assert ptr == file_slot.value and rgb(ptr) == palettes[source]
                global_after = global_slot.value
                global_source = 'global_custom' if mode == 'global_custom' else 'pre_host' if global_after else None
                if global_after: assert rgb(global_after) == palettes[global_source]
                assert ptr != global_after  # File owns a copy, not the global object.
                # Header mutation must not bypass a populated file cache.
                C.c_uint32.from_buffer(file, 0x184).value = 4 if source != 'builtin' else 2
                C.c_uint32.from_buffer(file, 0x37c).value = 0
                assert query(file) == ptr and rgb(ptr) == palettes[source]
                if global_after:
                    old = rgb(global_after)
                    C.c_ubyte.from_address(ptr+16).value ^= 0xff
                    assert rgb(global_after) == old
                rows.append({'mode': mode, 'base': a, 'override': b, 'palette': source,
                             'global_palette': global_source,
                             'global_created': not global_before and bool(global_after),
                             'file_created': mode != 'file_custom'})
                release(ptr)
                file_slot.value = None
            clear_global()
        finally:
            clear_global()
            set_host(key, previous)
            fn(0xa8a60, C.c_void_p, C.c_void_p, C.c_uint32)(service, 1)
        return {'scope': 'original_default_palette_service_and_native_caches',
                'gui_initialized': False, 'palettes': palettes, 'queries': rows,
                'file_cache_survives_header_mutation': True, 'global_copy_independent': True}
    finally:
        for directory in directories: directory.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--fixture', type=Path)
    args = parser.parse_args()
    result = probe(args.dll_root.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'dll_sha256': HASHES, **result}, indent=2), encoding='utf8')
    if args.fixture:
        body = json.dumps(result, separators=(',', ':'))
        chunks = ['R"P3D('+body[i:i+8192]+')P3D"' for i in range(0,len(body),8192)]
        args.fixture.write_text('#pragma once\n// Original R1.18 palette/cache observations.\n'
            +'inline constexpr const char *palette_selection_oracle_parts[] = {\n'
            +',\n'.join(chunks)+'\n};\n', encoding='utf8')
    print(f'Recorded {len(result["queries"])} native palette/cache queries')


if __name__ == '__main__': main()
