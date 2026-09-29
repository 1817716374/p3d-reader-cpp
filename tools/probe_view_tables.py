#!/usr/bin/env python3
"""Probe original R1.18 table constructors with bounded linked input entities.

Initial core handlers only; no edit overlays or registered replacement handlers.
Every matching table has at least one valid child, avoiding the unrelated
all-empty default-view creation branch. Child model IDs are -1, color/style
attributes and model-link references are absent. Native table allocations are
reclaimed at process exit; synthetic storage is never passed to destructors.
"""
import argparse
import base64
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import struct

HASHES = {
    'ROOT/P3DKJ.dll': '37fc96b97d230ba31619af4fba09ee11b2a38bfffca616621f11b240079f5901',
    'ROOT/P3DKJJC.dll': 'd5735b0f6c706beafb5feb2a21933404c70eb0371d8d208bf48867a4c43a313c',
    'ROOT/P3DDC.dll': '1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03',
    'SHARE/p3dlibxml2.dll': 'ca24cc124168dd90958f79e72d2af9dbca1dd937338ce6dd07fb0500cf21270a',
}


def probe(root, corpus_report=None):
    if os.name != 'nt' or C.sizeof(C.c_void_p) != 8:
        raise RuntimeError('Requires Windows x64')
    for name, expected in HASHES.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Unsupported DLL: {name}')
    directories = [os.add_dll_directory(str(root/d)) for d in
                   ['ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM']]
    try:
        dll = C.WinDLL(str(root/'ROOT/P3DKJ.dll'))
        base = dll._handle
        C.CFUNCTYPE(None)(base+0x39f3c0)()
        handler = C.c_void_p.from_address(base+0x6647e0+14*8).value
        assert C.c_void_p.from_address(handler).value == base+0x546568
        assert not C.c_void_p.from_address(base+0x664be0+14*8).value
        file = C.create_string_buffer(0x1000)
        context = C.addressof(file)+0x6c8
        C.c_void_p.from_buffer(file, 0x6c8).value = base+0x535e10
        C.c_void_p.from_buffer(file, 0x768).value = C.addressof(file)

        def run(lite, specs):
            owned = []
            def entity(kind, subtype, slot=0, marker=0, fields=None):
                storage = C.create_string_buffer(0x20+0x48+0x120)
                owned.append(storage)
                address = C.addressof(storage)+0x20
                C.c_void_p.from_address(address+0x20).value = context
                body = bytearray(0x120)
                struct.pack_into('<III', body, 0, kind, 0x90, 0x90)
                struct.pack_into('<I', body, 0xc, subtype)
                struct.pack_into('<II', body, 0xf8, 0xffffffff, 0x31415926)
                struct.pack_into('<h', body, 0x108, slot)
                struct.pack_into('<i', body, 0x10c, -1)
                body[0x110:0x113] = bytes([marker, 17, 43])
                body[0x70:0x90] = bytes([marker])*32
                if fields:
                    struct.pack_into('<II', body, 0xf8, *fields['words'])
                    body[0x110:0x113] = bytes(fields['rgb'])
                    body[0x70:0x90] = bytes(fields['aux_bytes'])
                C.memmove(address+0x48, bytes(body), len(body))
                return address
            parent = entity(14, 1000 if lite else 1)
            children = [entity(s['type'], s['subtype'], s['slot'], (i+1) % 256, s.get('fields'))
                        for i, s in enumerate(specs)]
            for i, child in enumerate(children):
                C.c_void_p.from_address(child).value = children[i+1] if i+1<len(children) else None
                if specs[i].get('nested'):
                    # An otherwise valid grandchild must not join the direct
                    # sibling iteration even if its slot is still vacant.
                    C.c_void_p.from_address(child+0x18).value = entity(11, 1, 7, 250)
            C.c_void_p.from_address(parent+0x18).value = children[0] if children else None
            wrapper, table = C.create_string_buffer(48), C.create_string_buffer(0x120)
            C.c_void_p.from_buffer(wrapper, 0x10).value = context
            C.c_void_p.from_buffer(wrapper, 0x18).value = parent
            ctor = C.CFUNCTYPE(C.c_void_p, C.c_void_p, C.c_void_p)(base+(0x4b2550 if lite else 0x4abc00))
            assert ctor(table, wrapper) == C.addressof(table)
            slots = []
            for i in range(8):
                view = C.c_void_p.from_buffer(table, 0x50+i*8).value
                aux = C.c_void_p.from_buffer(table, 0x90+i*8).value
                assert view and aux
                slots.append({'rgb': list(C.string_at(view+0xec, 3)),
                              'words': list(struct.unpack('<II', C.string_at(view+0x10, 8))),
                              'slot': C.c_int32.from_address(view+0xc8).value,
                              'aux_bytes': list(C.string_at(aux+0x10, 32)),
                              'style_index': C.c_int32.from_address(view+0x148).value})
            return {'lite': lite, 'children': specs, 'slots': slots}

        def child(slot, kind=11, subtype=1, nested=False):
            return {'type': kind, 'subtype': subtype, 'slot': slot, 'nested': nested}
        specs = [[child(i) for i in range(8)], [child(i) for i in reversed(range(8))],
                 [child(6), child(2)], [child(7), child(4), child(0)],
                 [child(2), child(2), child(6)],
                 [child(-1), child(8), child(32767), child(-32768), child(4)],
                 [child(0, 12), child(1, 11, 2), child(2), child(2), child(5)],
                 [child(0, 12, nested=True), child(4, nested=True)]]
        specs.extend([[child(i)] for i in range(8)])
        rows = [run(lite, s) for lite in [False, True] for s in specs]
        corpus_rows, skipped = [], []
        if corpus_report:
            report = json.loads(corpus_report.read_text(encoding='utf8'))
            for file_row in report['files']:
                if file_row['execution_status'] != 'completed':
                    raise ValueError('Incomplete corpus audit')
                document = json.loads((Path(file_row['output'])/'document.json').read_text(encoding='utf8'))
                for container in document['initial_view_table_inputs']:
                    if container['status'] != 'resolved':
                        raise ValueError('Unresolved table input inventory')
                    for table in container['tables']:
                        identity = {'source_file_sha256': file_row['sha256'], 'source': table['source']}
                        if table.get('slot_projection_status') != 'conditional':
                            skipped.append(identity)
                            continue
                        children = []
                        for scan in table['child_scan']:
                            record = document['native_records'][scan['source']['native_record_index']]
                            data = base64.b64decode(record['data']['base64'], validate=True)
                            kind = record['element_type']
                            subtype = struct.unpack_from('<I', data, 16)[0]
                            spec = child(struct.unpack_from('<h', data, 0x10c)[0] if kind == 11 and subtype == 1 else 0,
                                         kind, subtype)
                            if kind == 11 and subtype == 1:
                                if len(data) < 0x123:
                                    raise ValueError('Truncated corpus view input')
                                spec['fields'] = {'rgb': list(data[0x114:0x117]),
                                                  'words': list(struct.unpack_from('<II', data, 0xfc)),
                                                  'aux_bytes': list(data[0x74:0x94])}
                            children.append(spec)
                        if not any(c['type'] == 11 and c['subtype'] == 1 and 0 <= c['slot'] < 8 for c in children):
                            raise ValueError('Cannot enter all-empty native default creation')
                        row = run(table['table_kind'] == 'lite', children)
                        row.update(identity)
                        corpus_rows.append(row)
        return {'scope': 'original_table_constructors_with_builtin_handlers_and_nonempty_valid_child_set',
                'dll_sha256': HASHES, 'ordinary_constructor_rva': '0x4abc00',
                'lite_constructor_rva': '0x4b2550', 'handler_vtable_rva': '0x546568',
                'all_empty_default_creation': 'not_evaluated',
                'file_collection_selection': 'not_evaluated', 'cases': rows,
                'corpus_field_copy_cases': corpus_rows, 'corpus_skipped_tables': skipped}
    finally:
        for directory in directories:
            directory.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--fixture', type=Path)
    parser.add_argument('--corpus-report', type=Path)
    args = parser.parse_args()
    result = probe(args.dll_root.resolve(), args.corpus_report)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf8')
    if args.fixture:
        fixture = {k: v for k, v in result.items() if not k.startswith('corpus_')}
        args.fixture.write_text('#pragma once\n// Synthetic inputs and original R1.18 numeric observations.\n'
            'inline constexpr const char* view_tables_oracle = R"oracle('
            +json.dumps(fixture, separators=(',', ':'))+')oracle";\n', encoding='utf8')
    print(f"Observed {len(result['cases'])} synthetic and {len(result['corpus_field_copy_cases'])} corpus-field table constructors: {args.output}")


if __name__ == '__main__':
    main()
