#!/usr/bin/env python3
"""Original local file callbacks with a populated, distinct system registry.

Uses real file/model constructors. System targets are synthetic prepared
records registered by 1a5cc0/1a6920/1a5da0, not a simulated SSYS file load.
Local roots execute complete 199e40 callbacks. Runtime target flags are
explicit experimental inputs. No callback replacement or binary patch.
No retry, outer flush or final unload; native allocations live until exit.
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
from probe_dependency_registration import payload
from probe_view_frame import HASHES


def cases():
    high, maximum = 1 << 63, (1 << 64) - 1
    for format_ in (0, 1):
        for system_flags in (0, 8, 0x20000, 0x20008, 0x100000):
            for local_flags in (0, 8):
                for local_first in (False, True):
                    for disabled in (False, True):
                        ids = [41, 77, 78] if local_first else [77, 41, 78]
                        links = []
                        for id_ in ids:
                            links.append([] if id_ == 41 else [
                                payload([41, 42, high, maximum, 0, 500, 41, 77, 78, 42], format_, int(disabled)),
                                payload([42, 41, 42], format_, int(disabled))])
                        yield dict(system_ids=[41, 42, high, maximum], ids=ids,
                                   system_flags=system_flags, local_flags=local_flags,
                                   dependency_payloads=links)


def probe(root, inputs):
    assert os.name == 'nt' and C.sizeof(C.c_void_p) == 8
    for name, sha in HASHES.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
    directories = [os.add_dll_directory(str(root / d))
                   for d in ('ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM')]
    try:
        dll = C.WinDLL(str(root / 'ROOT/P3DKJ.dll'))
        base = dll._handle
        V = C.c_void_p
        ptr = lambda a: V.from_address(a).value
        u32 = lambda a: C.c_uint32.from_address(a).value
        fn = lambda r, t, *args: C.CFUNCTYPE(t, *args)(base + r)
        fn(0x163a90, None)()
        rows = []
        for case in inputs:
            keep, system_entities, local_entities = [], [], []
            with input_context(base, True) as snapshot:
                model_buffer, file_buffer, _, metadata = constructed_model(base, 100, False)
                model, file = C.addressof(model_buffer), C.addressof(file_buffer)
                system = file + 0x6c8
                assert ptr(system) == base + 0x535e10
                assert ptr(system + 0x698) is None and ptr(model + 0x698) is None
                if case.get('plain_root_owner_profile'):
                    assert ptr(base + 0x664be0 + 33 * 8) is None
                    host = C.CFUNCTYPE(V, V)(ptr(base + 0x51e788))(base + 0x643db8)
                    service = ptr(fn(0x1632e0, V, V, V)(host + 8, base + 0x643e18))
                    assert C.c_uint8.from_address(service + 8).value == 0
                    C.c_uint8.from_address(service + 8).value = case.get('owner_lookup_includes_deleted', False)
                # Actual original ordinary-model getter disables file fallback.
                assert ptr(ptr(model) + 0xe8) == base + 0x8a90
                assert not fn(0x8a90, C.c_bool, V)(model)
                if case.get('standard_model_owner_transition_known_null'):
                    assert ptr(ptr(model) + 0x28) == base + 0x9f030
                    assert ptr(ptr(model) + 0x58) == base + 0x8a80
                    assert fn(0x9f030, V, V)(model) == model
                    assert fn(0x8a80, V, V)(model) is None

                def insert(target_model, id_, payloads):
                    data = bytearray(128)
                    struct.pack_into('<HHIIIQ', data, 0, 33, 0x20, 64, 64, 0, id_)
                    struct.pack_into('<6q', data, 56, -1, -2, -3, 1, 2, 3)
                    for encoded in payloads:
                        raw = bytes.fromhex(encoded)
                        assert len(raw) + 4 <= 512
                        data += struct.pack('<HH', 0x1000 + (len(raw) + 4) // 2 - 1, 0x56d0) + raw
                    struct.pack_into('<I', data, 4, len(data) // 2)
                    buffer = C.create_string_buffer(0x68 + len(data))
                    keep.append(buffer)
                    node = C.addressof(buffer) + 0x20
                    C.memmove(node + 0x48, bytes(data), len(data))
                    if target_model == system:
                        listing = ptr(system + 0x138)
                        assert listing and ptr(listing + 0x30) == system
                        if not ptr(listing + 0x20):
                            fn(0x1a6920, V, V, C.c_int, C.c_bool, C.c_int, C.c_bool)(listing, 16, True, 0, True)
                        assert fn(0x1a5cc0, C.c_int, V, V, C.c_bool, C.c_bool, C.c_bool)(listing, node, True, False, False) == 0
                        rc = fn(0x1a5da0, C.c_int, V, V, V, C.c_bool, C.c_bool, C.c_bool, C.c_double, C.c_bool)(listing, node, None, False, True, False, 0., False)
                    else:
                        rc = fn(0x199e40, C.c_int, V, V, V, C.c_double, C.c_bool)(target_model, node, target_model + 0x140, 0., False)
                    assert rc == 0
                    entity = ptr(node + 0x28)
                    assert entity and C.c_uint64.from_address(ptr(entity + 0x40) + 16).value == id_
                    assert fn(ptr(ptr(entity) + 0x20) - base, V, V)(entity) == target_model
                    if case.get('plain_root_owner_profile'):
                        assert ptr(entity + 0x20) is None
                        assert ptr(entity + 0x28) is None
                        assert C.c_uint16.from_address(ptr(entity + 0x40)).value == 33
                        assert u32(ptr(entity + 0x40) + 12) == 0
                    return entity, data.hex()

                def reverse(entities):
                    lists = []
                    for entity in entities:
                        node = fn(0x19d5b0, V, V)(entity)
                        row, seen = [], set()
                        while node:
                            assert node not in seen
                            seen.add(node)
                            row.append(local_entities.index(ptr(node + 8)))
                            node = ptr(node)
                        lists.append(row)
                    return lists

                for id_ in case['system_ids']:
                    entity, _ = insert(system, id_, [])
                    C.c_uint32.from_address(entity + 0x10).value = case['system_flags']
                    system_entities.append(entity)
                file_entities = []
                for extra in case.get('file_models', []):
                    extra_model = fn(0x4ef808, V, C.c_size_t)(0x7f8)
                    assert extra_model
                    C.memset(extra_model, 0, 0x7f8)
                    assert fn(0x19d0f0, V, V, V, C.c_int)(extra_model, file, extra['model_id']) == extra_model
                    assert fn(0x1223e0, C.c_int, V, V, C.c_bool)(file, extra_model, False) == 0
                    assert fn(0x199330, C.c_int, V)(extra_model) == 1
                    assert ptr(ptr(extra_model) + 0xe8) == base + 0x8a90
                    registered = []
                    for id_ in extra['ids']:
                        entity, _ = insert(extra_model, id_, [])
                        C.c_uint32.from_address(entity + 0x10).value = extra['flags']
                        registered.append(entity)
                    file_entities.append(registered)
                if 'file_models' in case:
                    assert ptr(ptr(system) + 0xe8) == base + 0x37770
                    assert fn(0x37770, C.c_bool, V)(system)
                    assert C.c_uint64.from_address(file + 0x6b0).value == len(file_entities) + 1
                calls, headers = [], []
                for id_, links in zip(case['ids'], case['dependency_payloads']):
                    entity, header = insert(model, id_, links)
                    local_entities.append(entity)
                    headers.append(header)
                    if id_ == 41:
                        C.c_uint32.from_address(entity + 0x10).value = case['local_flags']
                    assert all(u32(e + 0x10) == case['system_flags'] for e in system_entities)
                    state = snapshot(local_entities)
                    assert state['callback_depth'] == state['transaction_status'] == 0
                    assert state['monitored_entities'] == state['scheduled_pairs'] == []
                    calls.append(dict(dependents=reverse(local_entities), system_dependents=reverse(system_entities),
                                      pending_entities=state['pending_entities']))
                    if 'file_models' in case:
                        assert all(u32(e + 0x10) == extra['flags']
                                   for extra, group in zip(case['file_models'], file_entities) for e in group)
                        calls[-1]['file_dependents'] = [reverse(group) for group in file_entities]
                lookups = []
                for id_ in case['system_ids'] + [0, 77, 78, 500]:
                    target = fn(0x19a8f0, V, V, C.c_uint64)(model, id_)
                    output = (C.c_uint64 * 3)(11, 22, 33)
                    fn(0x1f12b0, None, V, V, C.c_uint64)(output, model, id_)
                    assert output[0] == id_ and output[1] == 0 and output[2] == (target or 0)
                    identity = None
                    if target in local_entities: identity = ['local', local_entities.index(target)]
                    elif target in system_entities: identity = ['system', system_entities.index(target)]
                    else: assert target is None
                    lookups.append(dict(id=id_, target=identity))
                native = metadata() if 'file_models' not in case else dict(
                    current_model_id=u32(model + 0x1b8),
                    resident_models=C.c_uint64.from_address(file + 0x6b0).value,
                    system_file_fallback=True, ordinary_file_fallback=False)
                if case.get('observe_path_readers'):
                    readers = []
                    for encoded in next(links for links in case['dependency_payloads'] if links):
                        raw = bytes.fromhex(encoded)
                        _, _, flags, count = struct.unpack_from('<4H', raw)
                        if flags & 1 or not count:
                            continue  # Outer input never calls a skipped reader.
                        format_ = (flags >> 10) & 15
                        assert format_ in (0, 6)
                        buffer = C.create_string_buffer(raw)
                        for iteration in range(count):
                            output = (C.c_uint64 * 30)()
                            slots = fn(0x1ef320 if format_ == 0 else 0x1ef8d0,
                                       C.c_int, V, V, V, C.c_uint16)(output, model, buffer, iteration)
                            assert slots == (1 if format_ == 0 else 2)
                            refs = []
                            for i in range(slots):
                                target = output[i * 15 + 2]
                                identity = None
                                if target in local_entities: identity = ['local', local_entities.index(target)]
                                elif target in system_entities: identity = ['system', system_entities.index(target)]
                                else: assert target == 0
                                refs.append(dict(id=output[i * 15], owner=output[i * 15 + 1], target=identity))
                            readers.append(dict(iteration=iteration, references=refs))
                    native['path_readers'] = readers
                    assert snapshot(local_entities)['pending_entities'] == calls[-1]['pending_entities']
                    assert reverse(local_entities) == calls[-1]['dependents']
                    assert reverse(system_entities) == calls[-1]['system_dependents']
                rows.append(dict(case, source_headers=headers, calls=calls, lookups=lookups,
                                 runtime_flags=[u32(e + 0x10) for e in local_entities], native=native))
        return dict(scope='R1.18_local_file_callbacks_with_original_prepared_system_registry',
                    dll_sha256=HASHES, cases=rows)
    finally:
        for directory in directories: directory.close()


def fixture(result):
    return '#pragma once\n// Synthetic populated-system registry and original local callbacks.\ninline constexpr const char* dependency_system_oracle = R"oracle(' + json.dumps(result, separators=(',', ':')) + ')oracle";\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fixture', type=Path)
    args = parser.parse_args()
    result = probe(args.dll_root.resolve(), cases())
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf8')
    if args.fixture: args.fixture.write_text(fixture(result), encoding='utf8')
    print('Observed', len(result['cases']), 'populated-system dependency cases')


if __name__ == '__main__':
    main()
