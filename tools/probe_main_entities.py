#!/usr/bin/env python3
"""Load one audited public P3D copy through original R1.18 system/model input.

Run each source in a separate process so global handler/font caches cannot leak
between cases. Input is the same source inventory used by the model-header
probe. The output contains source-derived bytes; keep it outside the repository.
"""
import argparse
import ctypes as C
import json
import os
from pathlib import Path
import shutil
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from native_main_entity_input import acquire
from probe_main_file_open import digest, observe
from probe_main_model_headers import preflight
from probe_view_frame import HASHES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for option in ('dll-root', 'source-inventory', 'copy-dir', 'output'):
        parser.add_argument('--' + option, type=Path, required=True)
    parser.add_argument('--case-index', type=int, required=True)
    args = parser.parse_args()
    assert os.name == 'nt' and C.sizeof(C.c_void_p) == 8
    inventory = json.loads(args.source_inventory.read_text(encoding='utf-8-sig'))
    assert 0 <= args.case_index < len(inventory)
    row = inventory[args.case_index]
    expected_ids = preflight(row)
    root = args.dll_root.resolve()
    for name, sha in HASHES.items():
        assert digest(root / name) == sha, name
    source = Path(row['source']['path']).resolve()
    assert digest(source) == row['source']['sha256']
    assert source.stat().st_size == row['source']['bytes']
    target = (args.copy_dir / str(args.case_index) / source.name).resolve()
    assert target not in {Path(r['source']['path']).resolve() for r in inventory}
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)
    directories = [os.add_dll_directory(str(root / d))
                   for d in ('ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM')]
    try:
        dll = C.WinDLL(str(root / 'ROOT/P3DKJ.dll'))
        base = dll._handle
        # No optional plugin initialization callbacks in this core-only profile.
        assert all(C.c_void_p.from_address(base + r).value is None
                   for r in (0x640028, 0x640078))
        for rva in (0x163a90, 0x39f3c0):
            C.CFUNCTYPE(None)(base + rva)()
        def load(base, file, directory):
            assert {m['model_id'] for m in directory} == expected_ids
            return acquire(base, file, directory)
        result = observe(base, target, load)
        assert digest(source) == digest(target) == row['source']['sha256']
        result.update(source=row['source'], copy_path=str(target),
            source_and_copy_unchanged=True, dll_sha256=HASHES,
            scope='original_system_first_then_directory_models_core_console_input',
            host_context='inspected_shell_original_services_and_inline_font_outer_fields',
            dependency_outer_flush='not_executed', final_unload='not_executed')
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
        print(source.name, len(result['model_loading']['models']), 'models loaded; source unchanged')
    finally:
        for directory in directories:
            directory.close()


if __name__ == '__main__':
    main()
