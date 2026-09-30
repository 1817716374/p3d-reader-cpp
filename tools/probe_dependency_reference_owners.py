#!/usr/bin/env python3
"""Original type-13 reference construction, binding and dependency callbacks.

All file/model/reference objects, source input, reference-list registration
and target binding use original functions. The target models are synthetic
resident objects, not loaded from an external file. No virtual replacement,
binary patch, reference target pointer write, retry, flush or final unload.
"""
import argparse
import ctypes as C
import hashlib
import json
import os
import struct
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from native_input_context import input_context
from native_constructed_model import constructed_model
from native_main_entity_input import install_services
from probe_view_frame import HASHES
from probe_dependency_paths import payload as path_payload


def cases():
    programs = [(6, path, count) for path in ([41, 42], [41, 43, 42], [41, 41, 42],
                                             [41, 500, 42], [0, 42], [500, 42])
                for count in (1, 3)]
    programs += [(0, path, 1) for path in ([41, 42], [41, 43, 42], [41, 41, 42], [41, 500, 42])]
    programs += [(4, [41, 42], 1)]
    for format_, path, count in programs:
        for target_flags in (0, 8, 0x20000):
            for owner_flags, policy in ((0, False), (8, False), (8, True), (0x20000, False)):
                for disabled in (False, True):
                    raw = (struct.pack('<4H2Q', 999, 1, (4 << 10) | disabled, 1, *path).hex()
                           if format_ == 4 else path_payload(format_, path, count, disabled))
                    yield dict(format=format_, path=path, iterations=count, disabled=disabled,
                               payload_hex=raw, target_flags=target_flags, owner_flags=owner_flags,
                               owner_lookup_includes_deleted=policy)


def probe(root, inputs):
    assert os.name == 'nt' and C.sizeof(C.c_void_p) == 8
    for name, sha in HASHES.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
    directories = [os.add_dll_directory(str(root / d))
                   for d in ('ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM')]
    try:
        dll = C.WinDLL(str(root / 'ROOT/P3DKJ.dll'))
        base = dll._handle
        V, B, W = C.c_void_p, C.c_bool, C.c_wchar_p
        ptr = lambda a: V.from_address(a).value
        u32 = lambda a: C.c_uint32.from_address(a).value
        fn = lambda r, t, *args: C.CFUNCTYPE(t, *args)(base + r)
        for rva in (0x163a90, 0x39f3c0): fn(rva, None)()
        rows = []
        for case in inputs:
            keep, local, sources = [], [], []
            with input_context(base, True, True, True) as snapshot:
                install_services(base)
                host = C.CFUNCTYPE(V, V)(ptr(base + 0x51e788))(base + 0x643db8)
                service = fn(0x16e6c0, V)()
                assert ptr(service) == base + 0x530ff0
                V.from_address(host + 0x40).value = service
                name, wrapper = V(), V()
                # These factories construct a specification; no file is opened.
                logical_path = 'D:/codex/data/synthetic-reference.p3d'
                fn(0x1699e0, V, V, V, W, W, V, B, V, B, V, V)(
                    service, C.byref(name), logical_path, logical_path, None, False, None, False, None, None)
                fn(0x169220, V, V, V, V)(service, C.byref(wrapper), name)
                mb, fb, _, _ = constructed_model(base, 100, False, wrapper)
                model, file = C.addressof(mb), C.addressof(fb)
                assert ptr(file + 0xec8) == wrapper.value
                dependency = ptr(fn(0x1632e0, V, V, V)(host + 8, base + 0x643e18))
                C.c_uint8.from_address(dependency + 8).value = case['owner_lookup_includes_deleted']

                def allocate(n):
                    address = fn(0x4ef808, V, C.c_size_t)(n)
                    assert address
                    C.memset(address, 0, n)
                    return address

                def new_model(id_):
                    target = allocate(0x7f8)
                    assert fn(0x19d0f0, V, V, V, C.c_int)(target, file, id_) == target
                    assert fn(0x1223e0, C.c_int, V, V, B)(file, target, False) == 0
                    assert fn(0x199330, C.c_int, V)(target) == 1
                    return target

                def insert(owner, id_, type_=33, links=()):
                    size = 368 if type_ == 13 else 128
                    raw = bytearray(size)
                    struct.pack_into('<HHIIIQ', raw, 0, type_, 0x20, size // 2, size // 2, 0, id_)
                    if type_ == 13:
                        transform = case.get('source_transforms', {}).get(str(id_), {})
                        struct.pack_into('<3d', raw, 168, *transform.get('reference_point', [0, 0, 0]))
                        struct.pack_into('<3d', raw, 192, *transform.get('translation', [0, 0, 0]))
                        struct.pack_into('<9d', raw, 216, *transform.get('matrix', [1, 0, 0, 0, 1, 0, 0, 0, 1]))
                        struct.pack_into('<d', raw, 288, transform.get('scale', 1))
                    else: struct.pack_into('<6q', raw, 56, -1, -2, -3, 1, 2, 3)
                    for link in links:
                        data = bytes.fromhex(link)
                        raw += struct.pack('<HH', 0x1000 + (len(data) + 4) // 2 - 1, 0x56d0) + data
                    struct.pack_into('<I', raw, 4, len(raw) // 2)
                    buffer = C.create_string_buffer(0x68 + len(raw)); keep.append(buffer)
                    node = C.addressof(buffer) + 0x20
                    C.memmove(node + 0x48, bytes(raw), len(raw))
                    assert fn(0x199e40, C.c_int, V, V, V, C.c_double, B)(owner, node, owner + 0x140, 0., False) == 0
                    entity = ptr(node + 0x28)
                    assert entity and ptr(entity + 0x20) is None
                    assert C.c_uint64.from_address(ptr(entity + 0x40) + 16).value == id_
                    loaded = ptr(entity + 0x40)
                    assert u32(loaded + 4) * 2 == len(raw)
                    sources.append(dict(model_id=u32(owner + 0x1b8), id=id_, type=type_, header_hex=raw.hex(),
                                        loaded_header_hex=C.string_at(loaded, len(raw)).hex()))
                    return entity

                def reference(owner, source, target):
                    holder = C.create_string_buffer(0x28); keep.append(holder)
                    fn(0x391c90, V, V, V, B)(holder, source, False)
                    result = V()
                    assert fn(0x1400d0, C.c_int, V, V, V)(owner, C.byref(result), holder) == 0
                    ref = result.value
                    assert ref and ptr(ref) == base + 0x5301c8 and not ptr(ref + 0x88)
                    assert C.c_uint64.from_address(ref + 0x258).value == C.c_uint64.from_address(ptr(source + 0x40) + 16).value
                    listing = ptr(owner + 0x20)
                    assert listing and ref in [ptr(p) for p in range(ptr(listing + 8), ptr(listing + 16), 8)]
                    before = u32(target + 0x1c0)
                    fn(0x14ed50, None, V, V)(ref, target)
                    assert u32(target + 0x1c0) == before + 1 and ptr(ref + 0x88) == target
                    assert ptr(ptr(ref) + 0x28) == base + 0x1457b0
                    assert ptr(ptr(ref) + 0x58) == base + 0x9f030
                    assert fn(0x1457b0, V, V)(ref) == target and fn(0x9f030, V, V)(ref) == ref
                    return ref

                a, b = new_model(9), new_model(10)
                ae, be = insert(a, 41), insert(b, 41)
                nested_source = insert(a, 43, 13)
                for entity in (ae, be): C.c_uint32.from_address(entity + 0x10).value = case['target_flags']
                # First construct the nested reference under model 9. Its list
                # will be used only when the caller context is that model.
                nested_model_ref = reference(a, nested_source, model)
                file_entities = [[ae, nested_source], [be]]

                def reverse(entities):
                    result = []
                    for entity in entities:
                        node = fn(0x19d5b0, V, V)(entity); row, seen = [], set()
                        while node:
                            assert node not in seen; seen.add(node)
                            row.append(local.index(ptr(node + 8))); node = ptr(node)
                        result.append(row)
                    return result

                def observe():
                    state = snapshot(local)
                    assert state['callback_depth'] == state['transaction_status'] == 0
                    assert state['monitored_entities'] == state['scheduled_pairs'] == []
                    return dict(local_dependents=reverse(local),
                                file_dependents=[reverse(group) for group in file_entities],
                                pending_entities=state['pending_entities'])

                calls = []
                local.append(insert(model, 77, links=[case['payload_hex']]))
                calls.append(observe())
                source = insert(model, 42, 13); local.append(source)
                root_ref = reference(model, source, a)
                # A reference context has its own child list. Binding to a
                # model does not implicitly copy that model's list into it.
                nested_ref = reference(root_ref, nested_source, b)
                assert nested_ref != nested_model_ref
                C.c_uint32.from_address(source + 0x10).value = case['owner_flags']
                calls.append(observe())
                local.append(insert(model, 41)); calls.append(observe())
                local.append(insert(model, 78, links=[case['payload_hex']])); calls.append(observe())
                transitions = []
                for context, id_, expected in ((model, 42, root_ref), (root_ref, 43, nested_ref),
                                               (a, 43, nested_model_ref), (root_ref, 41, root_ref)):
                    got = fn(0x1023e0, V, V, C.c_uint64)(context, id_)
                    flags = case['owner_flags'] if context == model else case['target_flags'] if id_ == 41 else 0
                    rejected = flags & 8 and not case['owner_lookup_includes_deleted']
                    assert got == (None if rejected else expected), (case, id_, context == model, context == root_ref,
                                                                    got == root_ref, got == nested_ref, got == nested_model_ref, got is None)
                    transitions.append(dict(owner_id=id_, accepted=got is not None,
                                            result='null' if got is None else 'same_reference' if got == context else 'child_reference'))
                assert observe() == calls[-1]
                affine_queries = []
                if 'source_transforms' in case:
                    for ref_index, ref, stop_index, stop in ((0, root_ref, None, None),
                            (1, nested_model_ref, None, None), (2, nested_ref, None, None),
                            (2, nested_ref, 0, root_ref), (2, nested_ref, 2, nested_ref),
                            (0, root_ref, 2, nested_ref)):
                        out = (C.c_double * 12)()
                        fn(0x33af90, None, V, V, V)(out, ref, stop)
                        affine_queries.append(dict(reference_index=ref_index, stop_reference_index=stop_index,
                                                   native_matrix=list(out)))
                    assert observe() == calls[-1]
                row = dict(case, source_headers=sources, calls=calls, transitions=transitions,
                                 reference_ids=[42, 43, 43], reference_parent_contexts=['current', 'model_9', 'reference_42'],
                                 target_model_ids=[9, 7, 10])
                if affine_queries: row['affine_queries'] = affine_queries
                rows.append(row)
        return dict(scope='R1.18_original_bound_type13_reference_owner_callbacks', dll_sha256=HASHES, cases=rows)
    finally:
        for directory in directories: directory.close()


def fixture(result):
    return '#pragma once\n// Synthetic original bound type-13 owner callbacks; no vendor or corpus payloads.\ninline constexpr const char* dependency_reference_owners_oracle = R"oracle(' + json.dumps(result, separators=(',', ':')) + ')oracle";\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fixture', type=Path)
    args = parser.parse_args()
    result = probe(args.dll_root.resolve(), cases())
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf8')
    if args.fixture: args.fixture.write_text(fixture(result), encoding='utf8')
    print('Observed', len(result['cases']), 'bound reference owner cases')


if __name__ == '__main__':
    main()
