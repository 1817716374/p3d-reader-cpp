#!/usr/bin/env python3
"""Observe original format-6 and special format-0 owner paths in plain models.

Uses the complete initial callback and separately observes original reader
slots after registration. Standard type-33 roots reach a null model +58 slot;
this is not evidence about type-13/47 reference transitions or later retry.
"""
import argparse
import json
import struct
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_dependency_system import probe


def payload(format_, path, count=1, disabled=False):
    header = struct.pack('<4H', 10000, 4, (format_ << 10) | int(disabled),
                         len(path) if format_ == 0 else count)
    prefix = struct.pack('<I12x', len(path)) if format_ == 6 else b''
    return (header + prefix + b''.join(struct.pack('<Q', id_) for id_ in path)).hex()


def cases():
    maximum, high = (1 << 64) - 1, 1 << 63
    paths = [[], [41], [42], [0], [maximum], [high], [500],
             [41, 42], [42, 41], [0, 42], [41, 0], [41, 500],
             [41, maximum], [maximum, 42], [41, 42, 41], [41, 42, 500]]
    for format_ in (0, 6):
        for path in paths:
            for count in ((0, 1, 3) if format_ == 6 else (1,)):
                for flags, policy in ((0, False), (8, False), (8, True), (0x20000, False)):
                    for disabled in (False, True):
                        for local_first in (False, True):
                            ids = [41, 77, 78] if local_first else [77, 41, 78]
                            raw = payload(format_, path, count, disabled)
                            yield dict(format=format_, path=path, iterations=count,
                                       system_ids=[41, 42, maximum, high], ids=ids,
                                       system_flags=flags, local_flags=flags,
                                       plain_root_owner_profile=True,
                                       standard_model_owner_transition_known_null=True,
                                       owner_lookup_includes_deleted=policy,
                                       observe_path_readers=True,
                                       dependency_payloads=[[] if id_ == 41 else [raw] for id_ in ids])


def fixture(result):
    return '#pragma once\n// Synthetic original owner-path callbacks and reader slots.\ninline constexpr const char* dependency_paths_oracle = R"oracle(' + json.dumps(result, separators=(',', ':')) + ')oracle";\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fixture', type=Path)
    args = parser.parse_args()
    result = probe(args.dll_root.resolve(), cases())
    result['scope'] = 'R1.18_owner_paths_with_plain_type_33_roots_and_null_model_transition'
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf8')
    if args.fixture: args.fixture.write_text(fixture(result), encoding='utf8')
    print('Observed', len(result['cases']), 'owner-path dependency cases')


if __name__ == '__main__':
    main()
