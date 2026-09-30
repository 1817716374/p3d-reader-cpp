#!/usr/bin/env python3
"""Observe original R1.18 physical-reader range expansion in bounded memory.

The complete record is already buffered. No file refill, host callback, patch,
entity constructor or file writer is used. Only synthetic bytes are fixtures.
"""
import argparse
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import struct
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_view_frame import HASHES


def probe(root):
    assert os.name == 'nt' and C.sizeof(C.c_void_p) == 8
    for name, expected in HASHES.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == expected
    directories = [os.add_dll_directory(str(root / d))
                   for d in ('ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM')]
    try:
        dll = C.WinDLL(str(root / 'ROOT/P3DKJ.dll'))
        read = C.CFUNCTYPE(C.c_int, C.c_void_p)(dll._handle + 0x107060)
        values = [-(1 << 63), -(1 << 63) + 1, -100, -1, 0, 1, 100, (1 << 63) - 2, (1 << 63) - 1]
        rows = []
        for low in values:
            for delta in values:
                for extended in (False, True):
                    source = [low, delta, -1, delta, low, 1]
                    raw = bytearray(132)
                    struct.pack_into('<HHII', raw, 4, 127, 0x20 if extended else 0, 64, 64)
                    struct.pack_into('<6q', raw, 60, *source)
                    # A dummy prior record advances by 36 bytes to this input.
                    memory = C.create_string_buffer(36 + len(raw) + 32)
                    struct.pack_into('<I', memory, 8, 16)
                    C.memmove(C.addressof(memory) + 36, bytes(raw), len(raw))
                    context = C.create_string_buffer(0x40)
                    C.c_void_p.from_buffer(context, 0x30).value = C.addressof(memory)
                    C.c_void_p.from_buffer(context, 0x38).value = C.addressof(memory) + len(memory)
                    assert read(context) == 0
                    assert C.c_void_p.from_buffer(context, 0x30).value == C.addressof(memory) + 36
                    assert C.c_uint32.from_buffer(context, 0x28).value == 1
                    loaded = bytes(memory)[36:36 + len(raw)]
                    assert loaded[:84] == raw[:84] and loaded[108:] == raw[108:]
                    if not extended:
                        assert loaded == raw
                    rows.append(dict(extended=extended, source_values=source,
                                     reader_values=list(struct.unpack_from('<6q', loaded, 60))))
        return dict(dll_sha256=HASHES, cases=rows)
    finally:
        for directory in directories:
            directory.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fixture', type=Path)
    args = parser.parse_args()
    result = probe(args.dll_root.resolve())
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf8')
    if args.fixture:
        args.fixture.write_text('#pragma once\n// Original 107060; synthetic buffered records only.\n'
            'inline constexpr const char* record_bounds_input_oracle = R"oracle('
            + json.dumps(result, separators=(',', ':')) + ')oracle";\n', encoding='utf8')
    print(len(result['cases']), 'physical reader observations')


if __name__ == '__main__':
    main()
