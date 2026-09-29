#!/usr/bin/env python3
"""Observe R1.18 Lite list lookup/replacement using original style objects.

The list storage is synthetic and has spare capacity; the vtable and executed
lookup, replacement, clone and reference-count functions are original. Both
file contexts are null, so cross-file resource remapping is not exercised.
This does NOT execute dbaa0's host/file writeback callbacks.
"""
import argparse
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
from xml.sax.saxutils import quoteattr

HASHES = {
    "ROOT/P3DKJ.dll": "37fc96b97d230ba31619af4fba09ee11b2a38bfffca616621f11b240079f5901",
    "ROOT/P3DKJJC.dll": "d5735b0f6c706beafb5feb2a21933404c70eb0371d8d208bf48867a4c43a313c",
    "ROOT/P3DDC.dll": "1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03",
    "SHARE/p3dlibxml2.dll": "ca24cc124168dd90958f79e72d2af9dbca1dd937338ce6dd07fb0500cf21270a",
}


def probe(root):
    for name, expected in HASHES.items():
        if hashlib.sha256((root / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Unsupported DLL version: {name}')
    directories = [os.add_dll_directory(str(root / name))
                   for name in ['ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM']]
    try:
        dll = C.WinDLL(str(root / 'ROOT/P3DKJ.dll'))
        base = dll._handle

        def function(rva, result, *types):
            return C.CFUNCTYPE(result, *types)(base + rva)

        importer = function(0xd90a0, C.c_void_p, C.c_void_p, C.c_void_p, C.c_void_p)
        release = function(0x75a0, C.c_uint32, C.c_void_p)
        by_index = function(0x60000, C.c_void_p, C.c_void_p, C.c_void_p, C.c_uint64)
        by_name = function(0x60050, C.c_void_p, C.c_void_p, C.c_void_p, C.c_void_p)
        replace = function(0x601f0, None, C.c_void_p, C.c_void_p, C.c_bool)

        def create(name, index, mode=6, state=0):
            xml = C.create_unicode_buffer(f'<ShowStyle Name={quoteattr(name)}><Flags/>'
                                          f'<Overrides DisplayMode="{mode}"/></ShowStyle>')
            out = C.c_void_p()
            importer(C.byref(out), xml, None)
            if not out.value:
                raise RuntimeError('native style import failed')
            C.c_int32.from_address(out.value + 0x10).value = index
            C.c_uint32.from_address(out.value + 0x38).value = state
            return out.value

        def describe(ptr):
            if not ptr:
                return None
            size = C.c_uint64.from_address(ptr + 0x28).value
            capacity = C.c_uint64.from_address(ptr + 0x30).value
            address = C.c_void_p.from_address(ptr + 0x18).value if capacity >= 8 else ptr + 0x18
            return {'name': C.string_at(address, size * 2).decode('utf-16le'),
                    'stored_index': C.c_int32.from_address(ptr + 0x10).value,
                    'state': C.c_uint32.from_address(ptr + 0x38).value,
                    'flags48': C.c_uint32.from_address(ptr + 0x48).value}

        def run(spec, queries, replacements):
            array = (C.c_void_p * 16)()
            native_list = C.create_string_buffer(0x28)
            C.c_void_p.from_buffer(native_list).value = base + 0x529708
            # All supplied stored indices and maximum appended size are
            # bounded by this allocation. Never execute an out-of-bounds
            # native replacement to test rejection: native does not check it.
            for i, item in enumerate(spec):
                array[i] = create(**item) if item else None
            start = C.addressof(array)
            for offset, value in [(0x10, start), (0x18, start + 8 * len(spec)), (0x20, start + 128)]:
                C.c_void_p.from_buffer(native_list, offset).value = value

            def count():
                return (C.c_void_p.from_buffer(native_list, 0x18).value - start) // 8

            def snapshot():
                return [describe(array[i]) for i in range(count())]

            result = {'initial': snapshot(), 'queries': [], 'replacements': []}
            try:
                for query in queries:
                    out = C.c_void_p()
                    if isinstance(query, str):
                        name = C.create_unicode_buffer(query)
                        by_name(native_list, C.byref(out), name)
                    else:
                        by_index(native_list, C.byref(out), query)
                    try:
                        result['queries'].append({'query': query, 'result': describe(out.value)})
                    finally:
                        if out.value:
                            release(out)
                for name, allow in replacements:
                    new = create(name, 7, 31, 99)
                    try:
                        replace(native_list, new, allow)
                    finally:
                        release(new)
                    result['replacements'].append({'name': name, 'allow_append': allow, 'slots': snapshot()})
            finally:
                for i in range(count()):
                    if array[i]:
                        release(array[i])
            return result

        rows = [run([None, {'name': 'Alpha', 'index': 1, 'mode': 2, 'state': 7},
                     {'name': 'alpha', 'index': 2, 'mode': 4, 'state': 9},
                     {'name': 'Alpha', 'index': 3, 'mode': 5, 'state': 11}],
                    [0, 1, 3, 4, 0xffffffffffffffff, 'Alpha', 'alpha', 'ALPHA', 'missing'],
                    [('Alpha', False), ('New', False), ('New', True)]),
                run([{'name': '', 'index': 0}, {'name': '样式长名称测试A', 'index': 1},
                     {'name': 'Ä', 'index': 2}], ['', '样式长名称测试A', '样式长名称测试a', 'ä'], []),
                run([None, {'name': 'Wrong', 'index': 2, 'mode': 1}, {'name': 'Target', 'index': 2, 'mode': 2}],
                    [], [('Wrong', True)])]
        return {'dll_sha256': HASHES, 'file_context': None, 'host_initialized': False,
                'scope': 'original_list_methods_with_synthetic_storage', 'cases': rows}
    finally:
        for directory in directories:
            directory.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('distribution', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.name != 'nt' or C.sizeof(C.c_void_p) != 8:
        parser.error('requires 64-bit Python on Windows')
    result = probe(args.distribution.resolve(strict=True))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f"Recorded {len(result['cases'])} native list scenarios in {args.output}")


if __name__ == '__main__':
    main()
