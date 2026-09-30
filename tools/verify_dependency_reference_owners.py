#!/usr/bin/env python3
"""Validate bound-reference native observations with an independent graph replay.

This is a research verifier, not support in project_native_dependency_load.
The C++ API still rejects general type-13 owner transitions.
"""
import argparse
import copy
import json
import struct
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_dependency_reference_owners import cases
from probe_view_frame import HASHES


def verify(result):
    assert result['dll_sha256'] == HASHES
    totals = dict(cases=0, source_records=0, root_callbacks=0, reference_bindings=0,
                  local_edges=0, other_model_edges=0, pending_entities=0, transitions=0)
    for supplied, row in zip(cases(), result['cases'], strict=True):
        assert all(row[k] == value for k, value in supplied.items())
        assert row['reference_ids'] == [42, 43, 43]
        assert row['reference_parent_contexts'] == ['current', 'model_9', 'reference_42']
        assert row['target_model_ids'] == [9, 10, 10]
        assert [(s['model_id'], s['id'], s['type']) for s in row['source_headers']] == [
            (9, 41, 33), (10, 41, 33), (9, 43, 13), (7, 77, 33), (7, 42, 13), (7, 41, 33), (7, 78, 33)]
        for source in row['source_headers']:
            raw = bytes.fromhex(source['header_hex'])
            loaded = bytes.fromhex(source['loaded_header_hex'])
            assert len(loaded) == len(raw) and struct.unpack_from('<Q', loaded, 16)[0] == source['id']
            prepared = bytearray(raw)
            if source['type'] == 13: prepared[2] |= 0x40
            assert loaded == prepared
            assert struct.unpack_from('<H', raw)[0] == source['type']
            assert struct.unpack_from('<Q', raw, 16)[0] == source['id']
            assert struct.unpack_from('<I', raw, 4)[0] * 2 == len(raw)
            assert struct.unpack_from('<I', raw, 8)[0] * 2 == (368 if source['type'] == 13 else 128)
            tail = raw[368 if source['type'] == 13 else 128:]
            if source['id'] in (77, 78):
                h, app = struct.unpack_from('<HH', tail)
                assert app == 0x56d0 and 2 * ((h & 255) + 1) == len(tail)
                assert tail[4:].hex() == row['payload_hex']
            else: assert not tail
        # Contexts 0/1/2 denote the caller model, outer reference and nested
        # reference. A model identity alone cannot describe this traversal.
        local = {}
        target_flags, owner_flags = row['target_flags'], row['owner_flags']
        # Entity tuple: output group (-1=current), index, flags, source type.
        models = [{41: (0, 0, target_flags, 33), 43: (0, 1, 0, 13)},
                  {41: (1, 0, target_flags, 33)}]
        def lookup(context, id_):
            return (local if context == 0 else models[context - 1]).get(id_)
        def transition(context, id_):
            entity = lookup(context, id_)
            if entity is None or (entity[2] & 8 and not row['owner_lookup_includes_deleted']): return None
            if entity[3] == 13:
                if context == 0 and id_ == 42: return 1
                if context == 1 and id_ == 43: return 2
                raise AssertionError('Unmodelled synthetic reference')
            return context if context else None
        path, format_, maximum = row['path'], row['format'], (1 << 64) - 1
        def references():
            if row['disabled']: return []
            if format_ == 4:
                context = transition(0, path[1])
                return [(path[0], lookup(context, path[0]) if context is not None else None),
                        (path[1], lookup(0, path[1]))]
            if format_ == 6:
                context = 0
                for id_ in reversed(path[1:]):
                    context = transition(context, id_)
                    if context is None: break
                first = (maximum, None) if context is None else (path[0], lookup(context, path[0]))
                return [first, (path[-1], lookup(0, path[-1]))] + [(maximum, None)] * (2 * (row['iterations'] - 1))
            refs = []
            for index in range(len(path)):
                context = 0
                for id_ in reversed(path[index:]):
                    previous = context
                    context = transition(context, id_)
                    if context is None: break
                refs.append((maximum, None) if context is None else (path[index], lookup(previous, path[index])))
            return refs
        expected = dict(local_dependents=[], file_dependents=[[[], []], [[]]], pending_entities=[])
        for index, id_ in enumerate((77, 42, 41, 78)):
            local[id_] = (-1, index, owner_flags if id_ == 42 else 0, 13 if id_ == 42 else 33)
            expected['local_dependents'].append([])
            if id_ in (77, 78):
                for target_id, entity in references():
                    if entity is None:
                        if target_id and index not in expected['pending_entities']: expected['pending_entities'].append(index)
                    elif not entity[2] & 0x20008:
                        lists = expected['local_dependents'] if entity[0] == -1 else expected['file_dependents'][entity[0]]
                        lists[entity[1]].insert(0, index)
            assert expected == row['calls'][index], (supplied, index, expected, row['calls'][index])
        for i, item in enumerate(row['transitions']):
            flags = owner_flags if i == 0 else target_flags if i == 3 else 0
            accepted = not (flags & 8 and not row['owner_lookup_includes_deleted'])
            assert item == dict(owner_id=(42, 43, 43, 41)[i], accepted=accepted,
                                result=('same_reference' if i == 3 else 'child_reference') if accepted else 'null')
        totals['cases'] += 1
        totals['source_records'] += len(row['source_headers'])
        totals['root_callbacks'] += len(row['calls'])
        totals['reference_bindings'] += 3
        totals['local_edges'] += sum(map(len, expected['local_dependents']))
        totals['other_model_edges'] += sum(len(ids) for group in expected['file_dependents'] for ids in group)
        totals['pending_entities'] += len(expected['pending_entities'])
        totals['transitions'] += len(row['transitions'])
    assert totals['cases'] == 408
    return totals


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = json.loads(args.report.read_text(encoding='utf8'))
    totals = verify(result)
    # Meaningful negative controls: one lost cross-model edge, one wrong
    # transition identity, and one prematurely cleared pending membership.
    mutations = []
    for kind in ('edge', 'transition', 'pending'):
        altered = copy.deepcopy(result)
        if kind == 'edge':
            next(ids for row in altered['cases'] for group in row['calls'][-1]['file_dependents'] for ids in group if ids).pop()
        elif kind == 'transition': altered['cases'][0]['transitions'][0]['result'] = 'same_reference'
        else: next(row['calls'][-1]['pending_entities'] for row in altered['cases'] if row['calls'][-1]['pending_entities']).clear()
        try: verify(altered)
        except AssertionError: mutations.append(kind)
        else: raise AssertionError('Negative control was not detected: ' + kind)
    totals['negative_controls_detected'] = mutations
    args.output.write_text(json.dumps(totals, indent=2) + '\n', encoding='utf8')
    print(totals)


if __name__ == '__main__':
    main()
