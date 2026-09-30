#!/usr/bin/env python3
"""Read public P3D copies through the original R1.18 main-file provider.

Verifies the DLL/corpus hashes before execution. Original factories construct
the name objects, file and provider; 122580 selects the registered provider and
127700 reads the model directory. This excludes model/entity loading, fallback
directory reconstruction and final file destruction. Native allocations live
until process exit. No vendor binary or sample is embedded in this script.
"""
import argparse
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from native_input_context import input_context
from probe_view_frame import HASHES


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def observe(base, path, model_observer=None):
    V, I, B, W = C.c_void_p, C.c_int, C.c_bool, C.c_wchar_p
    ptr = lambda a: V.from_address(a).value
    u32 = lambda a: C.c_uint32.from_address(a).value
    u64 = lambda a: C.c_uint64.from_address(a).value
    fn = lambda rva, result, *args: C.CFUNCTYPE(result, *args)(base + rva)

    def text(address):
        size = u64(address + 16)
        assert size <= 32768
        return C.wstring_at(ptr(address) if u64(address + 24) >= 8 else address, size)

    def name_map(address):
        sentinel = ptr(address)
        result = {}
        def visit(node):
            if node == sentinel:
                return
            assert len(result) <= 100000
            visit(ptr(node))
            key = text(node + 0x20)
            assert key not in result
            result[key] = text(node + 0x40)
            visit(ptr(node + 0x10))
        visit(ptr(sentinel + 8))
        assert len(result) == u64(address + 8)
        return result

    with input_context(base, True, True, True):
        host = C.CFUNCTYPE(V, V)(ptr(base + 0x51e788))(base + 0x643db8)
        # Console host +08 -> BPPlatform 1151b8 -> KJ #9972 -> 16e6c0.
        service = fn(0x16e6c0, V)()
        assert ptr(service) == base + 0x530ff0
        V.from_address(host + 0x40).value = service
        model_service = fn(0x360e20, V)()
        assert ptr(model_service) == base + 0x544230
        V.from_address(host + 0x70).value = model_service
        # Concrete path specification and file wrapper, using original virtual
        # factories +30 and +10. Optional names and flags are explicitly empty.
        name, wrapper, holder = V(), V(), V()
        fn(0x1699e0, V, V, V, W, W, V, B, V, B, V, V)(
            service, C.byref(name), str(path), str(path), None, False, None, False, None, None)
        assert ptr(name.value) == base + 0x531338
        fn(0x169220, V, V, V, V)(service, C.byref(wrapper), name)
        assert ptr(wrapper.value) == base + 0x531248
        fn(0x360c10, V, V, V, V)(model_service, C.byref(holder), wrapper)
        file = holder.value
        assert ptr(file) == base + 0x52eef8 and u32(file + 0x680) == 1
        assert u32(file + 0x60) == 1 and not ptr(file + 0xee0)
        logical = C.create_string_buffer(32)
        fn(0x12b780, V, V, V)(file, logical)
        assert text(C.addressof(logical)) == str(path)
        # Original DLL static initialization registers the persistence factory.
        begin, end = ptr(base + 0x640798), ptr(base + 0x6407a0)
        assert begin and (end - begin) % 32 == 0
        factory_count = (end - begin) // 32
        status = I(-999)
        rc = fn(0x122580, I, V, V)(file, C.byref(status))
        assert rc == 0, hex(rc)
        provider = ptr(file + 0xee0)
        assert provider and ptr(provider) == base + 0x534a40
        assert C.c_bool.from_address(file + 0x678).value and u32(file + 0x688) == 2
        header = C.string_at(file + 0x68, 0x610)
        mapping = name_map(ptr(file + 0x58))
        # 127700 -> provider +00 -> 1c6750 -> 1082d0 reads persisted MMIx.
        # Only supported, complete MMIx inputs belong to this probe's scope.
        directory = fn(0x127700, V, V)(file)
        assert ptr(directory) == base + 0x533230 and ptr(directory + 0x28) == file
        begin, end = ptr(directory + 8) or 0, ptr(directory + 16) or 0
        assert end >= begin and (end - begin) % 0x88 == 0
        models = []
        for address in range(begin, end, 0x88):
            assert ptr(address) == directory
            bitmap = ptr(address + 0x70)
            mask = None
            if bitmap:
                count = u32(bitmap + 8)
                assert count <= 1048576 and count % 16 == 0
                mask = dict(effective_bit_count=count,
                            packed_words=list((C.c_uint16 * (count // 16)).from_address(ptr(bitmap))))
            # Preserve unknown field names; their business meanings are not inferred.
            models.append(dict(model_id=u32(address + 0x28), name=text(address + 8),
                secondary_string=text(address + 0x30), kind=u32(address + 0x50),
                flags=list(C.string_at(address + 0x54, 6)),
                unassigned_float64_bits=C.string_at(address + 0x60, 8).hex(),
                unassigned_u16=u32(address + 0x68), bitmap=mask,
                extension_count=u64(address + 0x80)))
        assert u64(file + 0x6b0) == 0 and u32(file + 0x684) == 0
        result = dict(return_code=rc, low_level_status=status.value, readonly=True,
            registered_factory_count=factory_count, provider_vtable='0x534a40',
            file_vtable='0x52eef8', name_spec_vtable='0x531338', name_wrapper_vtable='0x531248',
            provider_storage_present=bool(ptr(provider + 0x40)), header_hex=header.hex(),
            name_map=mapping, directory_version=u32(directory + 0x30), models=models,
            loaded_model_count=0, file_reference_count=u32(file + 0x680))
        # Optional follow-on experiment runs while the original TLS context lives.
        # The fields above retain the snapshot before any model acquisition.
        if model_observer is not None:
            result['model_loading'] = model_observer(base, file, models)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('dll-root', 'manifest', 'copy-dir', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    assert os.name == 'nt' and C.sizeof(C.c_void_p) == 8
    root = args.dll_root.resolve()
    for name, expected in HASHES.items():
        assert digest(root / name) == expected, name
    manifest = json.loads(args.manifest.read_text(encoding='utf-8-sig'))
    sources = [(Path(row['path']).resolve(), row) for row in manifest['files']]
    copy_dir = args.copy_dir.resolve()
    # Never overwrite any input, even if a caller supplies overlapping folders.
    targets = [(copy_dir / str(i) / source.name).resolve() for i, (source, _) in enumerate(sources)]
    assert not (set(targets) & {source for source, _ in sources})
    for source, row in sources:
        assert source.stat().st_size == row['bytes'] and digest(source) == row['sha256']
    directories = [os.add_dll_directory(str(root / d))
                   for d in ('ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM')]
    try:
        dll = C.WinDLL(str(root / 'ROOT/P3DKJ.dll'))
        rows = []
        for (source, row), target in zip(sources, targets):
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
            observed = observe(dll._handle, target)
            assert digest(source) == digest(target) == row['sha256']
            observed['source'] = row
            observed['copy_path'] = str(target)
            observed['source_and_copy_unchanged'] = True
            rows.append(observed)
            print(source.name, len(observed['models']), 'models in directory', flush=True)
        report = dict(scope='original_readonly_main_file_header_mapping_and_persisted_directory',
            model_entity_loading='not_executed', dll_sha256=HASHES, cases=rows)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    finally:
        for directory in directories:
            directory.close()


if __name__ == '__main__':
    main()
