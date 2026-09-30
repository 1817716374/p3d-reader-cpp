#!/usr/bin/env python3
"""Original format-8 callbacks with multiple resident models and system fallback.

All files/models use original constructors. Other-model targets use complete
199e40 input; system targets use the audited prepared registration core.
Models remain caller-held. No disk load, retry, flush or final unload.
"""
import argparse
import json
import struct
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_dependency_system import probe


def cases():
    maximum = (1 << 64) - 1
    entries = [(-1, 41), (-1, 51), (-1, 77), (-1, 78), (-1, 500), (-1, maximum),
               (9, 41), (9, 42), (9, 62), (9, 500), (7, 41), (7, 42), (7, 51),
               (-2, 51), (2, 51), (-3, 41), (123, 42), (-(1 << 31), 41), ((1 << 31) - 1, 0), (-1, 0)]
    for disabled in (False, True):
        for system_flags in (0, 8, 0x20000):
            for other_flags in (0, 8):
                for local_first in (False, True):
                    for model_count in (0, 1, 3):
                        models = [dict(model_id=9, ids=[41, 51, 61, maximum], flags=0),
                                  dict(model_id=-2, ids=[41, 51, 62], flags=other_flags),
                                  dict(model_id=2, ids=[41, 51, 63], flags=0)][:model_count]
                        raw = struct.pack('<4H', 999, 1, (8 << 10) | int(disabled), len(entries))
                        raw += b''.join(struct.pack('<iIQ', model, 0x12345678, id_) for model, id_ in entries)
                        ids = [41, 77, 78] if local_first else [77, 41, 78]
                        yield dict(system_ids=[41, 42, maximum], ids=ids,
                                   system_flags=system_flags, local_flags=0,
                                   file_models=models, dependency_payloads=[[] if id_ == 41 else [raw.hex()] for id_ in ids])


def fixture(result):
    return '#pragma once\n// Synthetic format-8 original resident-model selection and file fallback.\ninline constexpr const char* dependency_models_oracle = R"oracle(' + json.dumps(result, separators=(',', ':')) + ')oracle";\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fixture', type=Path)
    args = parser.parse_args()
    result = probe(args.dll_root.resolve(), cases())
    result['scope'] = 'R1.18_format_8_resident_model_selection_and_original_system_file_fallback'
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf8')
    if args.fixture: args.fixture.write_text(fixture(result), encoding='utf8')
    print('Observed', len(result['cases']), 'resident-model dependency cases')


if __name__ == '__main__':
    main()
