#!/usr/bin/env python3
"""Probe complete XML style imports with unmodified R1.18 DLLs (Windows x64).

Uses a null file context, no GUI/plugin initialization, and releases returned
objects. Synthetic inputs are bounded. Optional corpus audit inputs use their
retained XML and are checked for small Usages before executing native code.
This records object construction, not table registration or scene binding.
"""
import argparse
import ctypes as C
import hashlib
import json
import os
import re
from pathlib import Path
import xml.etree.ElementTree as ET

HASHES = {
    "ROOT/P3DKJ.dll": "37fc96b97d230ba31619af4fba09ee11b2a38bfffca616621f11b240079f5901",
    "ROOT/P3DKJJC.dll": "d5735b0f6c706beafb5feb2a21933404c70eb0371d8d208bf48867a4c43a313c",
    "ROOT/P3DDC.dll": "1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03",
    "SHARE/p3dlibxml2.dll": "ca24cc124168dd90958f79e72d2af9dbca1dd937338ce6dd07fb0500cf21270a",
}
MINIMAL = '<ShowStyle Name="x"><Flags/><Overrides DisplayMode="6"/></ShowStyle>'
CASES = [MINIMAL, MINIMAL.replace('ShowStyle', 'Other'), MINIMAL.replace('Name="x"', 'Name=""'),
         MINIMAL.replace('Name="x"', ''), MINIMAL.replace('<Flags/>', ''),
         MINIMAL.replace('<Flags/>', '<Flags DisplayShadows="true"/><Flags/>'),
         MINIMAL.replace('<Flags/>', '<Flags/><Flags DisplayShadows="true"/>'),
         MINIMAL.replace('<Overrides DisplayMode="6"/>', '<Overrides DisplayMode="2"/><Overrides DisplayMode="6"/>'),
         MINIMAL.replace('<Overrides DisplayMode="6"/>', '<Overrides/><Overrides DisplayMode="6"/>'),
         MINIMAL.replace('<Flags/>', '<Group><Flags/></Group>'),
         MINIMAL.replace('Name="x"', 'Name="样式 &amp; 长名称" EnvironmentName="环境名称" Usages="0,2-4"'),
         MINIMAL.replace('<ShowStyle ', '<ShowStyle xmlns="urn:test" '),
         MINIMAL.replace('<Flags/>', '<Flags xmlns="urn:test"/>'),
         MINIMAL.replace('<ShowStyle ', '<s:ShowStyle xmlns:s="urn:test" ').replace('</ShowStyle>', '</s:ShowStyle>')]


def corpus_cases(folder):
    report = json.loads((folder / 'corpus-report.json').read_text(encoding='utf-8'))
    for file in report['files']:
        document = json.loads((Path(file['output']) / 'document.json').read_text(encoding='utf-8'))
        for table in document['display_style_sources']['tables']:
            for entry in table['entries']:
                source = entry['source']
                attribute = document['graphics_records'][source['graphics_record_index']]['attributes'][source['attribute_ordinal']]
                yield {'source_file': file['source'], 'source_sha256': file['sha256'],
                       'source': source, 'xml': attribute['decoded']['xml'],
                       'library_import': entry['native_xml_import']}


def probe(root, inputs):
    for name, expected in HASHES.items():
        if hashlib.sha256((root / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Unsupported DLL version: {name}')
    directories = [os.add_dll_directory(str(root / name))
                   for name in ['ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM']]
    try:
        dll = C.WinDLL(str(root / 'ROOT/P3DKJ.dll'))
        base = dll._handle
        importer = C.CFUNCTYPE(C.c_void_p, C.c_void_p, C.c_void_p, C.c_void_p)(base + 0xd90a0)
        release = C.CFUNCTYPE(C.c_uint32, C.c_void_p)(base + 0x75a0)

        def read(ptr, offset, ctype):
            return ctype.from_address(ptr + offset).value

        def string(ptr, offset):
            length = read(ptr, offset + 16, C.c_uint64)
            capacity = read(ptr, offset + 24, C.c_uint64)
            address = read(ptr, offset, C.c_void_p) if capacity >= 8 else ptr + offset
            return C.string_at(address, length * 2).decode('utf-16le')

        rows = []
        for source in inputs:
            xml = source['xml']
            # Native Usages can allocate enormous bitmaps or loop forever.
            # Only execute a bounded subset here; parser tests cover the rest.
            tree = ET.fromstring(xml)
            usages = tree.attrib.get('Usages', '')
            bounded_grammar = r'[0-9]+(?: *- *[0-9]+)?(?: *, *[0-9]+(?: *- *[0-9]+)?)*'
            if ((usages and not re.fullmatch(bounded_grammar, usages)) or
                    any(int(n) > 1000 for n in re.findall(r'\d+', usages))):
                raise ValueError('probe requires bounded decimal Usages')
            buffer = C.create_unicode_buffer(xml)
            out = C.c_void_p()
            importer(C.byref(out), buffer, None)
            row = dict(source, constructed=bool(out.value))
            if out.value:
                ptr = out.value
                try:
                    row['vtable_rva'] = hex(read(ptr, 0, C.c_uint64) - base)
                    row['packed_flags_at_48'] = read(ptr, 0x48, C.c_uint32)
                    row['packed_flags_at_50'] = read(ptr, 0x50, C.c_uint32)
                    fields = {'Name': string(ptr, 0x18), 'EnvironmentName': string(ptr, 0xb8)}
                    for name, offset in [('VisibleEdgeColor', 0x54), ('VisibleEdgeWeight', 0x58),
                                         ('HiddenEdgeWeight', 0x5c), ('FillColor', 0x68),
                                         ('LineStyle', 0x6c), ('LineWeight', 0x70),
                                         ('BackgroundColor', 0x80), ('DisplayHandler', 0x90)]:
                        fields['Overrides.' + name] = read(ptr, offset, C.c_uint32)
                    fields['Overrides.Material'] = read(ptr, 0x78, C.c_uint64)
                    fields['EnvironmentTypeDisplayed'] = read(ptr, 0xb0, C.c_uint32)
                    fields['ShowGroundFromBelow'] = read(ptr, 0x100, C.c_bool)
                    row['fields'] = fields
                    row['double_bits'] = {name: f'{read(ptr, offset, C.c_uint64):016x}' for name, offset in [
                        ('Overrides.Transparency', 0x60), ('Overrides.HLineTransparencyThreshold', 0x88),
                        ('GroundPlaneColor.R', 0xd8), ('GroundPlaneColor.G', 0xe0), ('GroundPlaneColor.B', 0xe8),
                        ('GroundPlaneHeight', 0xf0), ('GroundPlaneTransparency', 0xf8)]}
                    bitmap = read(ptr, 0xa8, C.c_void_p)
                    extent = read(bitmap, 8, C.c_uint32)
                    if extent > 1001:
                        raise RuntimeError('unexpected native bitmap extent')
                    data = read(bitmap, 0, C.c_void_p)
                    row['usages'] = {'bit_length': extent, 'bits': [i for i in range(extent)
                        if read(data, 2 * (i // 16), C.c_uint16) & (1 << (i % 16))]}
                    row['display_handler_present'] = bool(read(ptr, 0x98, C.c_void_p))
                    row['display_handler_data_present'] = bool(read(ptr, 0xa0, C.c_void_p))
                finally:
                    release(out)
            rows.append(row)
        return {'dll_sha256': HASHES, 'importer_rva': '0xd90a0', 'file_context': None,
                'host_initialized': False, 'cases': rows}
    finally:
        for directory in directories:
            directory.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('distribution', type=Path)
    parser.add_argument('--corpus-audit', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fixture', type=Path, help='write a C++ fixture for synthetic inputs only')
    args = parser.parse_args()
    if os.name != 'nt' or C.sizeof(C.c_void_p) != 8:
        parser.error('requires 64-bit Python on Windows')
    inputs = [{'synthetic_index': i, 'xml': xml} for i, xml in enumerate(CASES)]
    if args.corpus_audit:
        inputs.extend(corpus_cases(args.corpus_audit))
    result = probe(args.distribution.resolve(strict=True), inputs)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    if args.fixture:
        synthetic = [row for row in result['cases'] if 'synthetic_index' in row]
        data = json.dumps(synthetic, ensure_ascii=True, separators=(',', ':'))
        args.fixture.parent.mkdir(parents=True, exist_ok=True)
        args.fixture.write_text('// Generated by tools/probe_style_object.py from original R1.18 DLLs.\n'
                                '// Only synthetic XML; returned objects released after observation.\n'
                                '#pragma once\ninline constexpr auto style_object_oracle_json = R"P3D(\n' +
                                data + '\n)P3D";\n', encoding='utf-8')
    print(f"Recorded {len(result['cases'])} full native XML imports in {args.output}")


if __name__ == '__main__':
    main()
