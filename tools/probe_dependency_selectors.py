#!/usr/bin/env python3
"""Original formats 2/3/4/5/7 callbacks with direct and plain-root owners.

Nonzero owners are standard type-33 roots, with no custom geometry handler.
The service's original deleted-owner policy is explicit. Special owner paths,
reference instances, geometry selection and regeneration are outside this probe.
"""
import argparse
import json
import struct
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_dependency_system import probe


def payload(format_, selector, disabled):
    maximum = (1 << 64) - 1
    data = []
    if format_ == 7:
        for id_ in (41, 42, 77, 78, 0, maximum, 1 << 63, 500):
            entry = bytearray(24)
            struct.pack_into('<II', entry, 0, id_ >> 32, id_ & 0xffffffff)
            entry[8] = selector
            struct.pack_into('<HHd', entry, 12, 1, 2, 12.5)
            data.append(entry)
    else:
        for a, b, ra, rb in [(41, 42, 0, 0), (41, 42, 42, 41), (77, 78, 500, 78),
                             (maximum, 41, 0, maximum), (0, maximum, 42, maximum),
                             (41, 42, maximum, 0), (42, 41, 500, 42)]:
            if format_ in (4, 5):
                entry = bytearray(16 if format_ == 4 else 24)
                struct.pack_into('<Q', entry, 0, a)
                struct.pack_into('<Q', entry, 8 if format_ == 4 else 16, ra)
            else:
                entry = bytearray(40 if format_ == 2 else 48)
                entry[0] = selector
                if selector in (2, 8): struct.pack_into('<4Q', entry, 8, a, b, ra, rb)
                else: struct.pack_into('<2Q', entry, 8, a, ra)
            data.append(entry)
    return (struct.pack('<4H', 999, 1, (format_ << 10) | int(disabled), len(data)) + b''.join(data)).hex()


def cases():
    for format_ in (2, 3, 4, 5, 7):
        for selector in ((0, 2, 8, 255) if format_ in (2, 3) else (0, 1, 2, 3) if format_ == 7 else (0,)):
            for system_flags in (0, 8, 0x20000):
                for local_flags in (0, 8):
                    for disabled in (False, True):
                        for local_first in (False, True):
                            ids = [41, 77, 78] if local_first else [77, 41, 78]
                            raw = payload(format_, selector, disabled)
                            yield dict(system_ids=[41, 42, (1 << 64) - 1, 1 << 63], ids=ids,
                                       system_flags=system_flags, local_flags=local_flags,
                                       plain_root_owner_profile=True,
                                       dependency_payloads=[[] if id_ == 41 else [raw] for id_ in ids])
    # Explicitly vary the real service policy where a deleted nonzero owner
    # can still lead to an accepted local target. No native callback changes.
    for format_ in (2, 3, 4, 5):
        for local_first in (False, True):
            ids = [41, 77, 78] if local_first else [77, 41, 78]
            raw = payload(format_, 2 if format_ in (2, 3) else 0, False)
            yield dict(system_ids=[41, 42, (1 << 64) - 1, 1 << 63], ids=ids,
                       system_flags=8, local_flags=0, plain_root_owner_profile=True,
                       owner_lookup_includes_deleted=True,
                       dependency_payloads=[[] if id_ == 41 else [raw] for id_ in ids])


def fixture(result):
    return '#pragma once\n// Synthetic original multi-slot selectors and standard type-33 root owners.\ninline constexpr const char* dependency_selectors_oracle = R"oracle(' + json.dumps(result, separators=(',', ':')) + ')oracle";\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fixture', type=Path)
    args = parser.parse_args()
    result = probe(args.dll_root.resolve(), cases())
    result['scope'] = 'R1.18_formats_2_3_4_5_7_original_callbacks_with_plain_type_33_root_owners'
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf8')
    if args.fixture: args.fixture.write_text(fixture(result), encoding='utf8')
    print('Observed', len(result['cases']), 'selector dependency cases')


if __name__ == '__main__':
    main()
