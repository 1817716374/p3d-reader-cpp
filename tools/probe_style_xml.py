#!/usr/bin/env python3
"""Probe scalar XML getters in an external, unmodified R1.18 p3dlibxml2.dll.

Windows x64 only. The DLL stays outside the repository. This uses synthetic
xmlNode values, not a licensed model or application registry. Output records
the binary hash and actual getter return values; non-finite doubles are strings.
"""
import argparse
import ctypes as C
import hashlib
import json
import math
import os
from pathlib import Path


CASES = [None, "", "true", "false", "True", "TRUE", "False", "1", "0", " true",
         "true ", "yes", "-1", "+2", "010", "0x10", "42tail", " 42 ", "4294967296",
         "3.5", "nan", "INF", "1e3", "1,5", "-4294967296", "4294967297",
         "18446744073709551615", "18446744073709551616", "18446744073709551617",
         "-18446744073709551616", "9999999999999999999999999999999999999",
         "1e", "1e+", "0x", "0x1p", "0x1p+", "+", "-", "--1", "1e999", "1e-999",
         "nan(foo)", "infinity", "infinite", "FALSE", "TrUe", "\ttrue", "true\n"]


def probe(path, dependencies):
    directories = [os.add_dll_directory(str(p)) for p in [path.parent, *dependencies]]
    try:
        dll = C.WinDLL(str(path))

        def function(name, result, args):
            f = getattr(dll, name)
            f.restype, f.argtypes = result, args
            return f

        ptr = C.c_void_p
        new = function("xmlNewNode", ptr, [ptr, C.c_char_p])
        prop = function("xmlSetProp", ptr, [ptr, C.c_char_p, C.c_char_p])
        free = function("xmlFreeNode", None, [ptr])
        getters = []
        for name, suffix, ctype in [("bool", "AEA_N", C.c_bool), ("u32", "AEAI", C.c_uint32),
                                    ("u64", "AEA_K", C.c_uint64), ("double", "AEAN", C.c_double)]:
            getter = function(f"?getAttribute@P3DXmlNode@p3d@@QEAA?AW4P3DXmlStatus@2@{suffix}PEBD@Z",
                              C.c_int, [ptr, C.POINTER(ctype), C.c_char_p])
            getters.append((name, getter, ctype))
        rows = []
        for value in CASES:
            node = new(None, b"Flags")
            if not node:
                raise RuntimeError("xmlNewNode failed")
            try:
                if value is not None and not prop(node, b"value", value.encode("utf-8")):
                    raise RuntimeError("xmlSetProp failed")
                row = {"source": value}
                for name, getter, ctype in getters:
                    output = ctype(123)
                    status = getter(node, C.byref(output), b"value")
                    result = output.value
                    if isinstance(result, float) and not math.isfinite(result):
                        result = str(result)
                    row[name] = {"status": status, "value": result}
                rows.append(row)
            finally:
                free(node)
        return rows
    finally:
        for directory in directories:
            directory.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dll", type=Path)
    parser.add_argument("--dll-dir", type=Path, action="append", default=[],
                        help="additional DLL dependency directory (repeatable)")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if os.name != "nt" or C.sizeof(C.c_void_p) != 8:
        parser.error("requires 64-bit Python on Windows")
    path = args.dll.resolve(strict=True)
    result = {"dll": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
              "cases": probe(path, [p.resolve(strict=True) for p in args.dll_dir])}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                           encoding="utf-8")
    print(f"Recorded {len(result['cases'])} synthetic inputs in {args.output}")


if __name__ == "__main__":
    main()
