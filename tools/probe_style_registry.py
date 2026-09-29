#!/usr/bin/env python3
"""Probe the original R1.18 core registry and default missing-handler service.

Runs in a short-lived process. This initializes core handler registrations, not
the BIMBase GUI. A synthetic thread-local host contains only the original
service pointer consumed by the inspected lookup path. It does not simulate
application/plugin registrations or load a model. Never use patched DLLs.
"""
import argparse
import ctypes as C
import hashlib
import json
import os
from pathlib import Path


HASHES = {
    "P3DKJ.dll": "37fc96b97d230ba31619af4fba09ee11b2a38bfffca616621f11b240079f5901",
    "P3DDC.dll": "1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03",
}
CASES = [(0x006F0000, 0), (0x597E0000, 0), (0x58740000, 0), (0x58740001, 0),
         (0x58740000, 0), (0x58740001, 0xFFFFFFFF), (0xEC340000, 0), (0x5DC0000A, 0)]


def probe(root):
    for name, expected in HASHES.items():
        actual = hashlib.sha256((root / "ROOT" / name).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError(f"Unsupported DLL version: {name} ({actual})")
    directories = [os.add_dll_directory(str(root / d))
                   for d in ["ROOT", "SHARE", "SHARE/vcredist/X64", "PLATFORM"]]
    try:
        dll = C.WinDLL(str(root / "ROOT/P3DKJ.dll"))
        base = dll._handle

        def function(rva, result, *args):
            return C.CFUNCTYPE(result, *args)(base + rva)

        def vtable(ptr):
            return hex(C.c_uint64.from_address(ptr).value - base) if ptr else None

        # No arguments: initializes the built-in tables used by the host init
        # at ordinal 5271. Does not call that host initializer with a fake host.
        function(0x39F3C0, None)()
        lookup = function(0x394620, C.c_void_p, C.c_void_p)
        resolve = function(0x3946E0, C.c_void_p, C.c_void_p)
        create_service = dll[9982]  # 0x360dc0, original default-service factory
        create_service.restype, create_service.argtypes = C.c_void_p, [C.c_void_p]
        service = create_service(None)
        if not service or vtable(service) != "0x544390":
            raise RuntimeError("unexpected default handler service")
        host = C.create_string_buffer(0x110)
        C.c_void_p.from_buffer(host, 0x88).value = service
        # Verified IAT slots: P3DDC ordinals 239 (get) and 417 (set).
        get_host = C.CFUNCTYPE(C.c_void_p, C.c_void_p)(
            C.c_void_p.from_address(base + 0x51E788).value)
        set_host = C.CFUNCTYPE(None, C.c_void_p, C.c_void_p)(
            C.c_void_p.from_address(base + 0x51E5B0).value)
        host_key = base + 0x643DB8
        previous = get_host(host_key)
        rows = []
        try:
            set_host(host_key, C.addressof(host))
            if get_host(host_key) != C.addressof(host):
                raise RuntimeError("synthetic service context not installed")
            for registration, tail in CASES:
                marker = C.c_uint64(registration | (tail << 32))
                before = lookup(C.byref(marker))
                returned = resolve(C.byref(marker))
                after = lookup(C.byref(marker))
                rows.append({"registration_key": registration, "word_at_4": tail,
                             "before_vtable_rva": vtable(before),
                             "returned_vtable_rva": vtable(returned),
                             "after_vtable_rva": vtable(after)})
            fallback = C.c_void_p.from_address(base + 0x664AC0).value
            factories = C.c_void_p.from_address(base + 0x664EC0).value
            result = {"scope": "original_core_registry_with_original_default_service",
                      "host": "synthetic_service_context_only", "gui_initialized": False,
                      "dll_sha256": HASHES, "core_initializer_rva": "0x39f3c0",
                      "registry_lookup_rva": "0x394620", "service_lookup_rva": "0x3946e0",
                      "default_service_vtable_rva": vtable(service),
                      "type_92_default_vtable_rva": vtable(fallback),
                      "type_92_factory_list_present": bool(factories), "queries": rows}
        finally:
            set_host(host_key, previous)
            # Original default-service scalar deleting destructor, flag 1.
            function(0xA8A60, C.c_void_p, C.c_void_p, C.c_uint32)(service, 1)
        return result
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
    print(f"Recorded {len(result['queries'])} native registry queries in {args.output}")


if __name__ == "__main__":
    main()
