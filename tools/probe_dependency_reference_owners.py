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

                def source_node(id_, type_=33, links=(), path_profile=None, source_flags=None, descendants=None):
                    size = 368 if type_ == 13 else 34 if type_ == 47 else 256 if type_ == 62 else 128
                    raw = bytearray(size)
                    struct.pack_into('<HHIIIQ', raw, 0, type_, 4 if type_ == 47 else 0x20,
                                     size // 2, size // 2, 20 if type_ == 47 else 0, id_)
                    if type_ == 13:
                        transform = case.get('source_transforms', {}).get(str(id_), {})
                        struct.pack_into('<3d', raw, 168, *transform.get('reference_point', [0, 0, 0]))
                        struct.pack_into('<3d', raw, 192, *transform.get('translation', [0, 0, 0]))
                        struct.pack_into('<9d', raw, 216, *transform.get('matrix', [1, 0, 0, 0, 1, 0, 0, 0, 1]))
                        struct.pack_into('<d', raw, 288, transform.get('scale', 1))
                    elif type_ == 62:
                        profile = path_profile or {}
                        struct.pack_into('<9d', raw, 160, *profile.get('matrix',[1,0,0,0,1,0,0,0,1]))
                        struct.pack_into('<3d', raw, 232, *profile.get('translation',[0,0,0]))
                    elif type_ == 47:
                        profile = path_profile or {}
                        struct.pack_into('<I', raw, 12, profile.get('subtype', 20))
                        struct.pack_into('<H', raw, 32, profile.get('signature', 0x56e6))
                    else: struct.pack_into('<6q', raw, 56, -1, -2, -3, 1, 2, 3)
                    if source_flags is not None:struct.pack_into('<H',raw,2,source_flags)
                    if descendants is not None:struct.pack_into('<I',raw,104,descendants)
                    for link in links:
                        data = bytes.fromhex(link)
                        raw += struct.pack('<HH', 0x1000 + (len(data) + 4) // 2 - 1, 0x56d0) + data
                    struct.pack_into('<I', raw, 4, len(raw) // 2)
                    buffer = C.create_string_buffer(0x68 + len(raw)); keep.append(buffer)
                    node = C.addressof(buffer) + 0x20
                    C.memmove(node + 0x48, bytes(raw), len(raw))
                    if type_ == 62:
                        # Same source preparation called by physical reader
                        # 107830 before its node enters the model input path.
                        fn(0x1076b0, None, V)(node + 0x48)
                    return node,raw,C.string_at(node + 0x48, len(raw)).hex()

                def observe_source(owner, id_, type_, node, raw, prepared):
                    entity = ptr(node + 0x28)
                    assert entity
                    assert C.c_uint64.from_address(ptr(entity + 0x40) + 16).value == id_
                    loaded = ptr(entity + 0x40)
                    assert u32(loaded + 4) * 2 == len(raw)
                    sources.append(dict(model_id=u32(owner + 0x1b8), id=id_, type=type_, header_hex=raw.hex(),
                                        loaded_header_hex=C.string_at(loaded, len(raw)).hex()))
                    if type_ == 62:sources[-1]['prepared_header_hex'] = prepared
                    return entity

                def insert(owner, id_, type_=33, links=(), path_profile=None):
                    node,raw,prepared=source_node(id_,type_,links,path_profile)
                    assert fn(0x199e40, C.c_int, V, V, V, C.c_double, B)(owner, node, owner + 0x140, 0., False) == 0
                    entity=observe_source(owner,id_,type_,node,raw,prepared)
                    assert ptr(entity + 0x20) is None
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
                batches=[[0],[1],[2]]
                tree_nodes=[];tree_observations=[]
                if 'tree_sources' in case:
                    specs=case['tree_sources'];children=[[] for _ in specs]
                    for i,spec in enumerate(specs):
                        parent=spec['parent']
                        assert parent is None or 0<=parent<i
                        if parent is not None:children[parent].append(i)
                    subtrees=[[i] for i in range(len(specs))]
                    for i in reversed(range(len(specs))):
                        for child in children[i]:subtrees[i].extend(subtrees[child])
                    assert [j for i,s in enumerate(specs) if s['parent'] is None for j in subtrees[i]]==list(range(len(specs)))
                    nodes=[]
                    for i,spec in enumerate(specs):
                        assert spec['type'] in (14,33,47,62)
                        assert not children[i] or spec['type']==14
                        flags=spec.get('source_flags',(4 if spec['type']==47 else 0x20)|
                                       (0x80 if spec['parent'] is not None else 0)|(0x40 if children[i] else 0))
                        nodes.append(source_node(spec['id'],spec['type'],spec.get('links',[]),spec,flags,
                                                 len(subtrees[i])-1 if spec['type']==14 else None))
                    # Link only input nodes, before original entity creation.
                    # Runtime parent/child pointers are never patched.
                    for i,spec in enumerate(specs):
                        node=nodes[i][0]
                        if spec['parent'] is not None:V.from_address(node+0x10).value=nodes[spec['parent']][0]
                        if children[i]:
                            V.from_address(node+0x18).value=nodes[children[i][0]][0]
                            C.c_uint32.from_address(node+0x30).value=1
                        for first,second in zip(children[i],children[i][1:]):
                            V.from_address(nodes[first][0]).value=nodes[second][0]
                    tree_nodes=[None]*len(specs)
                    for i,spec in enumerate(specs):
                        if spec['parent'] is not None:continue
                        assert fn(0x199e40,C.c_int,V,V,V,C.c_double,B)(model,nodes[i][0],model+0x140,0.,False)==0
                        batch=[]
                        for j in subtrees[i]:
                            item=specs[j]
                            tree_nodes[j]=observe_source(model,item['id'],item['type'],*nodes[j])
                            batch.append(len(local));local.append(tree_nodes[j])
                            C.c_uint32.from_address(tree_nodes[j]+0x10).value=item.get('runtime_flags',0)
                        batches.append(batch);calls.append(observe())
                    for i,(spec,entity) in enumerate(zip(specs,tree_nodes,strict=True)):
                        parent=ptr(entity+0x20)
                        assert parent==(None if spec['parent'] is None else tree_nodes[spec['parent']])
                        listing=ptr(entity+0x38)
                        actual=[] if not listing else [tree_nodes.index(ptr(at)) for at in range(ptr(listing),ptr(listing+8),8)]
                        assert actual==children[i]
                        tree_observations.append(dict(id=spec['id'],parent=spec['parent'],children=actual,
                                                      runtime_flags=u32(entity+0x10)))
                block_nodes = []
                for block_source in case.get('block_sources', []):
                    local.append(insert(model, block_source['id'], 62, (), block_source))
                    block_nodes.append(local[-1])
                    C.c_uint32.from_address(local[-1] + 0x10).value = block_source.get('runtime_flags', 0)
                    batches.append([len(local)-1])
                    calls.append(observe())
                path_nodes = []
                for path_source in case.get('path_owners', []):
                    local.append(insert(model, path_source['id'], 47, path_source['links'], path_source))
                    path_nodes.append(local[-1])
                    C.c_uint32.from_address(local[-1] + 0x10).value = path_source.get('runtime_flags', 0)
                    batches.append([len(local)-1])
                    calls.append(observe())
                local.append(insert(model, 78, links=[case['payload_hex']])); calls.append(observe())
                batches.append([len(local)-1])
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
                path_queries = []
                if 'path_owners' in case:
                    # Exact inline empty collector initialization observed in
                    # 1023e0/1f14a0; the real vtable and vector initializer are
                    # used, with no replacement virtual functions.
                    identities = {entity: dict(model_id=7, id=C.c_uint64.from_address(ptr(entity + 0x40)+16).value)
                                  for entity in local}
                    identities.update({entity: dict(model_id=id_, id=C.c_uint64.from_address(ptr(entity + 0x40)+16).value)
                                       for id_, entities in zip((9,10), file_entities) for entity in entities})
                    contexts = {None: 'null', model: 'current', root_ref: 'reference_42',
                                nested_ref: 'nested_reference_43', nested_model_ref: 'model9_reference_43'}
                    query_sources = case.get('tree_sources', []) + case.get('block_sources', []) + case['path_owners']
                    for node, source_spec in zip(tree_nodes + block_nodes + path_nodes, query_sources, strict=True):
                        collector = C.create_string_buffer(0x38); keep.append(collector)
                        address = C.addressof(collector)
                        V.from_address(address).value = base + 0x52a1c0
                        fn(0x18de60, None, V, C.c_size_t)(address + 0x10, 0)
                        C.c_int64.from_address(address + 0x30).value = -1
                        code = fn(0x1017b0, C.c_int, V, V, V, V, B)(address, None, node, model, False)
                        begin, end = ptr(address+0x10), ptr(address+0x18)
                        collected = [] if not begin else [identities[ptr(p)] for p in range(begin,end,8)]
                        out = (C.c_double * 12)()
                        transform_code = fn(0x101dc0, C.c_int, V, V, V)(out, address, model) if code == 0 else None
                        path_queries.append(dict(id=source_spec['id'], native_code=code,
                            owner_context=contexts[ptr(address+0x28)], collected=collected,
                            terminal_index=C.c_int32.from_address(address+0x30).value,
                            transform_code=transform_code, native_matrix=list(out) if transform_code == 0 else None))
                        if 'block_sources' in case or 'tree_sources' in case:
                            owner = ptr(address+0x28)
                            path_queries[-1]['owner_kind'] = (C.CFUNCTYPE(C.c_int,V)(ptr(ptr(owner)+0x48))(owner)
                                                                    if owner else None)
                        if code == 0 and 'terminal_positions' in case:
                            terminal = C.c_int32.from_address(address+0x30).value
                            queries = []
                            for position in case['terminal_positions']:
                                # Use the original setter, including its -1 =
                                # last-object convention; do not patch fields.
                                fn(0x231ca0, None, V, C.c_int)(address, position)
                                matrix = (C.c_double * 12)()
                                status = fn(0x101dc0, C.c_int, V, V, V)(matrix, address, model)
                                queries.append(dict(requested_position=position,
                                    terminal_index=C.c_int32.from_address(address+0x30).value,
                                    transform_code=status,native_matrix=list(matrix) if status == 0 else None))
                            path_queries[-1]['terminal_queries'] = queries
                            fn(0x231ca0, None, V, C.c_int)(address, terminal)
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
                if path_queries: row['path_queries'] = path_queries
                if 'tree_sources' in case:row.update(input_batches=batches,tree_entities=tree_observations)
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
