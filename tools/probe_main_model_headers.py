#!/usr/bin/env python3
"""Acquire real persisted models with the original R1.18 main-file provider.

Requires a source inventory produced by verify_main_model_headers --inventory.
Only audited version-8 headers and version-7 headers with positive scales and
an identity orientation are admitted. No entity lists, write methods or final
destructors are invoked. Native objects remain caller-held until process exit.
"""
import argparse
import ctypes as C
import json
import os
from pathlib import Path
import shutil
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_main_file_open import digest, observe
from probe_view_frame import HASHES


def preflight(row):
    ids = set()
    for header in row['headers']:
        data = bytes.fromhex(header['header_hex'])
        assert len(data) >= 496 and len(data) == 2 * struct.unpack_from('<I', data, 4)[0]
        assert struct.unpack_from('<H', data)[0] == 47
        assert struct.unpack_from('<I', data, 12)[0] == 32
        version = struct.unpack_from('<H', data, 32)[0]
        assert version == header['version'] and version in (7, 8)
        assert len(bytes.fromhex(header['prefix_hex'])) == 4096
        model_id = int(header['model_key'][1:], 16)
        assert model_id < 0x7fffffff and model_id not in ids
        ids.add(model_id)
        if version == 7:
            # 105020 normalizes old headers. This profile avoids its unsupported
            # default-unit and nonidentity-orientation repair branches.
            assert all(struct.unpack_from('<d', data, off)[0] > 0 for off in (0xe0, 0x168, 0x170))
            assert struct.unpack_from('<9d', data, 0x120) == (1., 0., 0., 0., 1., 0., 0., 0., 1.)
    return ids


def acquire(base, file, directory, expected_ids):
    V, I = C.c_void_p, C.c_int
    ptr = lambda a: V.from_address(a).value
    u32 = lambda a: C.c_uint32.from_address(a).value
    u64 = lambda a: C.c_uint64.from_address(a).value
    byte = lambda a: C.c_uint8.from_address(a).value
    fn = lambda rva, result, *args: C.CFUNCTYPE(result, *args)(base + rva)
    provider = ptr(file + 0xee0)
    assert {entry['model_id'] for entry in directory} == expected_ids
    rows = []
    for entry in directory:
        model_id = entry['model_id']
        # 1263f0 -> provider +68 -> 1c71b0: original construction, SMH
        # read/conversion and active-tree registration, without entity input.
        model = fn(0x1263f0, V, V, I)(file, model_id)
        assert model and ptr(model) == base + 0x5333c8
        assert ptr(model + 0xa0) == file and u32(model + 0x1b8) == model_id
        assert u32(model + 0x1c0) == 0
        assert fn(0x199330, I, V)(model) == 1
        header = ptr(model + 0x7e8)
        size = u32(header + 4) * 2
        assert 496 <= size <= 0x20000
        loaded = C.string_at(header, size).hex()
        assert byte(model + 0x7a0) == 1
        assert fn(0x1263f0, V, V, I)(file, model_id) == model
        assert ptr(model + 0x7e8) == header and u32(model + 0x1c0) == 1
        # Original read-tool factory opens this model's selected substorage.
        # 104b70(false) reads only the 4096-byte prefix into context +44.
        # Provider +118 (1c73d0) is a WRITER and must not be used here.
        reader = V()
        assert fn(0x1c6890, I, V, V, V)(provider, C.byref(reader), model) == 0
        assert ptr(reader.value) == base + 0x52e150
        rc = fn(0x104b70, I, V, V, C.c_bool, I, V, V)(
            reader, ptr(reader.value + 0x38), False, model_id, None, None)
        assert rc == 0
        prefix = C.string_at(reader.value + 0x44, 4096).hex()
        assert C.string_at(header, size).hex() == loaded
        controls = ptr(model + 0x138)
        assert byte(controls + 0x14) == byte(model + 0x154) == 0
        assert not ptr(controls + 0x18) and not ptr(controls + 0x20)
        assert not ptr(model + 0x158) and not ptr(model + 0x160)
        rows.append(dict(model_id=model_id, model_vtable='0x5333c8', header_hex=loaded,
            prefix_hex=prefix, spatial=bool(byte(model + 0x78)), readonly=True,
            model_reference_count=u32(model + 0x1c0), cached_model_reused=True,
            entity_lists_loaded=False, prefix_return_code=rc,
            active_model_count=u64(file + 0x6b0), held_model_count=u32(file + 0x684)))
    assert u64(file + 0x6b0) == u32(file + 0x684) == len(rows)
    return dict(models=rows, active_model_count=u64(file + 0x6b0),
                held_model_count=u32(file + 0x684), file_reference_count=u32(file + 0x680))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('dll-root', 'source-inventory', 'copy-dir', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    assert os.name == 'nt' and C.sizeof(C.c_void_p) == 8
    root = args.dll_root.resolve()
    for name, expected in HASHES.items():
        assert digest(root / name) == expected, name
    inventory = json.loads(args.source_inventory.read_text(encoding='utf-8-sig'))
    sources = [Path(row['source']['path']).resolve() for row in inventory]
    targets = [(args.copy_dir / str(i) / source.name).resolve() for i, source in enumerate(sources)]
    assert not set(sources) & set(targets)
    for row, source in zip(inventory, sources):
        assert digest(source) == row['source']['sha256'] and source.stat().st_size == row['source']['bytes']
        preflight(row)
    directories = [os.add_dll_directory(str(root / d))
                   for d in ('ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM')]
    try:
        dll = C.WinDLL(str(root / 'ROOT/P3DKJ.dll'))
        cases = []
        for row, source, target in zip(inventory, sources, targets):
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
            print(source.name, 'acquiring model headers', flush=True)
            result = observe(dll._handle, target,
                lambda base, file, models: acquire(base, file, models, preflight(row)))
            assert digest(source) == digest(target) == row['source']['sha256']
            result.update(source=row['source'], copy_path=str(target), source_and_copy_unchanged=True)
            cases.append(result)
            print(len(result['model_loading']['models']), 'model headers loaded', flush=True)
        args.output.write_text(json.dumps(dict(
            scope='original_readonly_main_file_model_acquisition_and_header_input',
            entity_loading='not_executed', dll_sha256=HASHES, cases=cases),
            ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    finally:
        for directory in directories:
            directory.close()


if __name__ == '__main__':
    main()
