#!/usr/bin/env python3
"""Original constructed/bound type-13 owners with nonidentity source transforms.

Extends the identity experiment, without changing reference fields after input.
Records graph callbacks and original 33af90 current-to-ancestor affine queries.
No scale providers, clipping, nonzero model origins or external model loading.
"""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_dependency_reference_owners import cases as owner_cases, probe


def cases():
    rotation = dict(matrix=[0, -1, 0, 1, 0, 0, 0, 0, 1], scale=2,
                    translation=[10, 20, 30], reference_point=[1, 2, 3])
    nested = dict(matrix=[1, 0, 0, 0, 0, -1, 0, 1, 0], scale=0.5,
                  translation=[3, 7, 11], reference_point=[-1, 2, 4])
    mirror = dict(matrix=[-1, 0, 0, 0, 1, 0, 0, 0, 1], scale=-2,
                  translation=[-8, 5, 13], reference_point=[2, -3, 1])
    shear = dict(matrix=[1, 2, 0, 0, 1, 0.5, 0, 0, 1], scale=0.25,
                 translation=[1, -2, 4], reference_point=[-4, 2, 8])
    zero = dict(matrix=[1, 0, 0, 0, 1, 0, 0, 0, 1], scale=0,
                translation=[7, 11, -13], reference_point=[1, 2, 3])
    singular = dict(matrix=[1, 2, 3, 2, 4, 6, 0, 0, 0], scale=2,
                    translation=[3, -5, 7], reference_point=[-1, 2, -3])
    for label, outer, inner in (('rotation', rotation, nested), ('mirror_shear', mirror, shear),
                                ('zero_scale', zero, rotation), ('singular', shear, singular)):
        for case in owner_cases():
            if case['iterations'] != 1: continue
            if case['format'] != 4 and case['path'] != [41, 43, 42]: continue
            yield dict(case, transform_case=label, source_transforms={'42': outer, '43': inner})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fixture', type=Path)
    args = parser.parse_args()
    result = probe(args.dll_root.resolve(), cases())
    result['scope'] = 'R1.18_original_bound_type13_nonidentity_affines'
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf8')
    if args.fixture:
        # Graph flags do not alter these source-to-affine observations. Keep
        # one matrix fixture per transform, proving equality before compacting.
        profiles = {}
        for row in result['cases']:
            item = dict(transform_case=row['transform_case'],
                        source_headers=[s for s in row['source_headers'] if s['type'] == 13],
                        affine_queries=row['affine_queries'])
            if row['transform_case'] in profiles:
                assert profiles[row['transform_case']] == item
            profiles[row['transform_case']] = item
        compact = dict(scope=result['scope'], dll_sha256=result['dll_sha256'],
                       native_case_count=len(result['cases']), cases=list(profiles.values()))
        args.fixture.write_text('#pragma once\n// Synthetic original constructed-reference observations.\n'
            'inline constexpr const char* dependency_reference_affines_oracle = R"oracle('
            + json.dumps(compact, separators=(',', ':')) + ')oracle";\n', encoding='utf8')
    print('Observed', len(result['cases']), 'nonidentity bound-reference cases')


if __name__ == '__main__': main()
