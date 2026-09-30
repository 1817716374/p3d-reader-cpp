#!/usr/bin/env python3
"""Independent source-to-affine arithmetic, ancestry and dependency replay."""
import argparse
import copy
import json
import math
import struct
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_dependency_reference_affines import cases
from verify_dependency_reference_owners import verify as verify_graph


def verify(result):
    assert result['scope'] == 'R1.18_original_bound_type13_nonidentity_affines'
    totals = verify_graph(result, cases())
    assert totals['cases'] == 288
    totals['affine_queries'] = 0
    identity = [[float(i == j) for j in range(4)] for i in range(4)]
    def multiply(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
    for row in result['cases']:
        matrices = {}
        for source in row['source_headers']:
            if source['type'] != 13: continue
            raw = bytes.fromhex(source['header_hex'])
            point = struct.unpack_from('<3d', raw, 168)
            translation = struct.unpack_from('<3d', raw, 192)
            linear = struct.unpack_from('<9d', raw, 216)
            scale = struct.unpack_from('<d', raw, 288)[0]
            # This family contains only unit orthogonal bases or nonrigid
            # bases. Original input normalizes columns even when the rigid
            # rescale predicate fails; zero source scale defaults to one.
            lengths = [math.sqrt(sum(linear[3*i+j]**2 for i in range(3))) for j in range(3)]
            normalized = [[linear[3*i+j] / lengths[j] if lengths[j] else float(i == 0)
                           for j in range(3)] for i in range(3)]
            scale = scale or 1.
            matrix = [[normalized[i][j] * scale for j in range(3)] for i in range(3)]
            for i in range(3): matrix[i].append(translation[i] - sum(matrix[i][j] * point[j] for j in range(3)))
            matrices[source['id']] = matrix + [[0., 0., 0., 1.]]
        queries = row['affine_queries']
        assert [(q['reference_index'], q['stop_reference_index']) for q in queries] == [
            (0, None), (1, None), (2, None), (2, 0), (2, 2), (0, 2)]
        for query in queries:
            current, stop = query['reference_index'], query['stop_reference_index']
            expected = identity
            while current is not None and current != stop:
                expected = multiply(matrices[(42, 43, 43)[current]], expected)
                current = (None, None, 0)[current]
            values = [v for r in expected[:3] for v in r]
            assert len(query['native_matrix']) == 12
            assert all(math.isclose(a, b, rel_tol=2e-14, abs_tol=2e-13)
                       for a, b in zip(query['native_matrix'], values)), (row['transform_case'], query)
            totals['affine_queries'] += 1
    return totals


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = json.loads(args.report.read_text(encoding='utf8'))
    totals = verify(result)
    caught = []
    for kind in ('matrix', 'source', 'stop', 'edge'):
        altered = copy.deepcopy(result)
        row = altered['cases'][0]
        if kind == 'matrix': row['affine_queries'][2]['native_matrix'][3] += 1
        elif kind == 'source':
            source = next(s for s in row['source_headers'] if s['type'] == 13)
            for key in ('header_hex', 'loaded_header_hex'):
                raw = bytearray.fromhex(source[key]); struct.pack_into('<d', raw, 288, 4.)
                source[key] = raw.hex()
        elif kind == 'stop': row['affine_queries'][3]['stop_reference_index'] = None
        else:
            next(ids for r in altered['cases'] for group in r['calls'][-1]['file_dependents'] for ids in group if ids).pop()
        try: verify(altered)
        except AssertionError: caught.append(kind)
        else: raise AssertionError('Negative control not detected: ' + kind)
    totals['negative_controls_detected'] = caught
    args.output.write_text(json.dumps(totals, indent=2) + '\n', encoding='utf8')
    print(totals)


if __name__ == '__main__': main()
