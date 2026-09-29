#!/usr/bin/env python3
"""Record small synthetic Usages inputs with the original R1.18 native DLL.

Use the extracted distribution directory. Ordinals and object layouts are
version-specific, so the two directly inspected DLL hashes are checked first.
No large or nonterminating ranges are executed by this probe.
"""
import argparse
import ctypes as C
import hashlib
import json
import os
from pathlib import Path


HASHES = {"P3DKJJC.dll": "d5735b0f6c706beafb5feb2a21933404c70eb0371d8d208bf48867a4c43a313c",
          "P3DDC.dll": "1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03"}
CASES = ["", "0", "1", "0,2", "1-3", "3-1", "1,3,3", "  1, 3", "\t1,\t3", ",1",
         "1,,3", "1 3", "1\t3", "1\n3", "1,\n3", "\n1", "+1", "-1", "1-", "1--0",
         "1-+3", "1 - 3", "1- 3", "1;3", "1tail,3", "0x10,3", "1, +3,4", "1,\v3",
         "1,\u30003", "1\u30003", "1-2-3", "1\t-3", "1,3\t,5", "1,\r\n3", "0,15-17",
         "1-\u30003", "1-\t3", "1-\n3", "1-\u00a03", "1,\u00a03", "1,\u00853",
         "1,\u200b3", "1,\u20073", "1,\u202f3", "1,\ufeff3", "1, ,3", "1, 3,,5",
         "4294967296", "4294967297,2", "4294967299-4294967297", "1--4294967296"]


def probe(root):
    for name, expected in HASHES.items():
        actual = hashlib.sha256((root / "ROOT" / name).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError(f"Unsupported DLL version: {name} ({actual})")
    directories = [os.add_dll_directory(str(root / name))
                   for name in ["ROOT", "SHARE", "SHARE/vcredist/X64", "PLATFORM"]]
    try:
        dll = C.WinDLL(str(root / "ROOT/P3DKJJC.dll"))

        def function(ordinal, result, args):
            f = dll[ordinal]
            f.restype, f.argtypes = result, args
            return f

        create = function(170, C.c_void_p, [C.c_bool])
        destroy = function(203, None, [C.POINTER(C.c_void_p)])
        parse = function(280, None, [C.c_void_p, C.c_void_p, C.c_uint32, C.c_uint32])
        rows = []
        for text in CASES:
            data = text.encode("utf-16le")
            length = len(data) // 2
            source = C.create_string_buffer(32)
            buffer = C.create_string_buffer(data + b"\0\0")
            if length < 8:
                C.memmove(source, data, len(data))
            else:
                C.c_void_p.from_buffer(source).value = C.addressof(buffer)
            C.c_uint64.from_buffer(source, 16).value = length
            C.c_uint64.from_buffer(source, 24).value = max(7, length)
            obj = C.c_void_p(create(False))
            if not obj.value:
                raise RuntimeError("native bitset creation failed")
            try:
                parse(obj, source, 0, 0xffffffff)
                size = C.c_uint32.from_address(obj.value + 8).value
                if size > 1000:
                    raise RuntimeError("unexpected bitset extent for bounded probe input")
                pointer = C.c_void_p.from_address(obj.value).value
                words = ((C.c_uint16 * ((size + 15) // 16)).from_address(pointer) if pointer else [])
                bits = [i for i in range(size) if words[i // 16] & (1 << (i % 16))]
                rows.append({"source": text, "size": size, "bits": bits})
            finally:
                destroy(C.byref(obj))
        dc = C.WinDLL(str(root / "ROOT/P3DDC.dll"))
        # Verified import slot, not an arbitrary code address. The hash gate
        # above is necessary before using this R1.18 IAT offset.
        address = C.c_void_p.from_address(dc._handle + 0x277e8).value
        is_space = C.CFUNCTYPE(C.c_int, C.c_uint16)(address)
        spaces = [i for i in range(65536) if is_space(i)]
        return {"dll_sha256": HASHES, "cases": rows, "iswspace_utf16_units": spaces}
    finally:
        for directory in directories:
            directory.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("distribution", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if os.name != "nt" or C.sizeof(C.c_void_p) != 8:
        parser.error("requires 64-bit Python on Windows")
    result = probe(args.distribution.resolve(strict=True))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Recorded {len(result['cases'])} synthetic Usages inputs in {args.output}")


if __name__ == "__main__":
    main()
