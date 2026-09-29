#!/usr/bin/env python3
"""Observe original R1.18 file-table lookup on a resident native collection.

Uses the bounded root/child construction of probe_view_collection.py. Inserts
an explicit first type46/subtype8 root and calls the complete 4b2350 getter.
No model loading or UI: synthetic default model -2 suppresses new default-table
creation on a miss. Observations of misses are not general host fallback rules.
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
        dll = C.WinDLL(str(root/'ROOT/P3DKJ.dll')); base = dll._handle
        C.CFUNCTYPE(None)(base+0x39f3c0)()
        handler = C.c_void_p.from_address(base+0x6647e0+14*8).value
        assert C.c_void_p.from_address(handler).value == base+0x546568
        ctor = C.CFUNCTYPE(C.c_void_p, C.c_void_p, C.c_void_p)(base+0x4b2f70)
        # All three are original leaf functions: return *(this + offset).
        # 16e2a0:+30, b5190:+38, daf60:+40. No Python callbacks execute.
        vtable = (C.c_void_p*36)()
        for slot, rva in [(0x20,0x16e2a0),(0x60,0xb5190),(0x118,0xdaf60),(0x38,0x6570)]:
            vtable[slot//8] = base+rva

        def units(address):
            size = C.c_uint64.from_address(address+16).value
            capacity = C.c_uint64.from_address(address+24).value
            assert size <= 511
            p = C.c_void_p.from_address(address).value if capacity >= 8 else address
            return list(struct.unpack('<'+'H'*size, C.string_at(p, size*2)))

        def link(key, data):
            payload = struct.pack('<HHI', key, 0, len(data))+data
            total = (len(payload)+4+15)//16*16
            words = total//2
            header = 0x1000 | (words-1) if words <= 256 else 0x5300 | (words//8)
            assert words <= 256 or words % 8 == 0 and words//8 <= 255
            payload += bytes(total-4-len(payload))
            return {'header': header, 'app': 0x56d2, 'payload_hex': payload.hex()}

        def text(value):
            return b'\xff\xfe\x01\x00'+value.encode('latin1')

        def spec(lite, first='A', second='B', links=None):
            return {'lite': lite, 'links': links if links is not None else [link(1,text(first)),link(2,text(second))]}

        def run(specs, ids=None, queries=None, header_prefix=None):
            ids = ids if ids is not None else [100+i for i in range(len(specs))]
            owned = []
            def alloc(n):
                p = C.create_string_buffer(n); owned.append(p); return p
            file, table, block, output = alloc(0x1000), alloc(0x38), alloc(0x48), alloc(0x48)
            context = C.addressof(file)+0x6c8
            C.c_void_p.from_buffer(file,0x6c8).value = base+0x535e10
            C.c_void_p.from_buffer(file,0x768).value = C.addressof(file)
            addresses, observed_names = [], []
            for i, item in enumerate(specs):
                links = bytearray()
                for l in item['links']:
                    payload = bytes.fromhex(l['payload_hex'])
                    # Only complete, bounded native linkage payloads are executed.
                    if l['app'] == 0x56d2 and len(payload) >= 8:
                        assert struct.unpack_from('<I',payload,4)[0] <= len(payload)-8
                    links += struct.pack('<HH',l['header'],l['app'])+payload
                body = bytearray(0x120)+links
                struct.pack_into('<III',body,0,14,len(body)//2,0x90)
                struct.pack_into('<I',body,0xc,1000 if item['lite'] else 1)
                struct.pack_into('<Q',body,0x10,ids[i])
                entity_storage = alloc(0x20+0x48+len(body))
                entity = C.addressof(entity_storage)+0x20
                C.memmove(entity+0x48, bytes(body),len(body))
                child_storage = alloc(0x20+0x48+0x120)
                child = C.addressof(child_storage)+0x20
                child_body = bytearray(0x120)
                struct.pack_into('<III',child_body,0,11,0x90,0x90)
                struct.pack_into('<I',child_body,0xc,1)
                struct.pack_into('<i',child_body,0x10c,-1)
                C.memmove(child+0x48,bytes(child_body),len(child_body))
                C.c_void_p.from_address(child+0x20).value = context
                C.c_void_p.from_address(entity+0x18).value = child
                C.c_void_p.from_address(entity+0x20).value = context
                record = alloc(0x48)
                for offset, value in [(0,C.addressof(vtable)),(0x18,handler),(0x30,context),
                                      (0x38,entity+0x48),(0x40,entity)]:
                    C.c_void_p.from_buffer(record,offset).value = value
                addresses.append(C.addressof(record))
                # Name reads are observed separately on real constructor output
                # so replaced objects need not remain alive for introspection.
                wrapper, native_table = alloc(48), alloc(0x120)
                C.c_void_p.from_buffer(wrapper,0x10).value = context
                C.c_void_p.from_buffer(wrapper,0x18).value = entity
                table_ctor = C.CFUNCTYPE(C.c_void_p,C.c_void_p,C.c_void_p)(base+(0x4b2550 if item['lite'] else 0x4abc00))
                table_ctor(native_table,wrapper)
                observed_names.append([units(C.addressof(native_table)+0x10),units(C.addressof(native_table)+0x30)])
            pointers = (C.c_void_p*len(addresses))(*addresses); owned.append(pointers)
            for offset,value in [(0x18,C.addressof(pointers)),(0x20,C.addressof(pointers)+C.sizeof(pointers))]:
                C.c_void_p.from_buffer(block,offset).value=value
            C.c_void_p.from_buffer(table,0x18).value=C.addressof(block)
            C.c_void_p.from_buffer(file,0x800).value=C.addressof(table)
            assert ctor(output,file)==C.addressof(output)
            begin,end = (C.c_void_p.from_buffer(output,o).value or 0 for o in [8,16])
            assert 0 <= (end-begin)//8 <= len(specs)
            selected=[]
            for p in range(begin,end,8):
                obj=C.c_void_p.from_address(p).value
                record=C.c_void_p.from_address(obj+0xe0).value
                index=addresses.index(record)
                assert [units(obj+0x10),units(obj+0x30)]==observed_names[index]
                selected.append(index)
            native_objects = [C.c_void_p.from_address(p).value for p in range(begin,end,8)]
            header_body, header_record = alloc(0x254), alloc(0x48)
            struct.pack_into('<IIII', header_body, 0, 46, 0x12a, 0x12a, 8)
            if header_prefix is not None:
                assert len(header_prefix)==0x250
                C.memmove(header_body, header_prefix, len(header_prefix))
            C.c_void_p.from_buffer(header_record).value = C.addressof(vtable)
            C.c_void_p.from_buffer(header_record,0x38).value = C.addressof(header_body)
            roots = (C.c_void_p*(len(addresses)+1))(C.addressof(header_record),*addresses)
            C.c_void_p.from_buffer(block,0x18).value = C.addressof(roots)
            C.c_void_p.from_buffer(block,0x20).value = C.addressof(roots)+C.sizeof(roots)
            C.c_void_p.from_buffer(file,0xf30).value = C.addressof(output)
            C.c_int32.from_buffer(file,0x18c).value = -2
            getter = C.CFUNCTYPE(C.c_uint32,C.c_void_p,C.c_void_p)(base+0x4b2350)
            observations=[]
            if queries is None:queries=list(dict.fromkeys(ids+[0,0xffffffffffffffff,0xfedcba9876543210]))
            for query in queries:
                C.c_uint64.from_buffer(header_body,0x248).value = query
                result = C.c_void_p()
                error = getter(C.byref(result),file)
                index = selected[native_objects.index(result.value)] if result.value else None
                observations.append({'requested_id':query,'native_return_code':error,
                                     'selected_input_index':index})
                if result.value:
                    vt=C.c_void_p.from_address(result.value).value
                    release=C.c_void_p.from_address(vt+8).value
                    C.CFUNCTYPE(None,C.c_void_p)(release)(result.value)
                assert C.c_void_p.from_buffer(output,8).value==begin or begin==0
                assert C.c_void_p.from_buffer(output,16).value==end or end==0
            return {'inputs':specs,'assigned_ids':ids,'collection_input_indices':selected,
                    'queries':observations}

        cases = [[],[spec(False)],[spec(True)],
                 [spec(False),spec(False),spec(True)],
                 [spec(True),spec(False,'C'),spec(False),spec(True)],
                 [spec(False),spec(True,'a'),spec(True,'A','b')],
                 [spec(False),spec(False),spec(True),spec(True)],
                 [spec(False,'A\x00tail'),spec(True,'A')],
                 [spec(False,links=[]),spec(True,links=[link(1,b'\xfe\xff')])],
                 [spec(False,links=[]),spec(True,links=[link(1,text('x'*511))])],
                 [spec(False,links=[link(1,text('first')),link(1,text('last'))]),spec(True,'first','')],
                 [spec(False,links=[link(1,b'\xfe\xff'),link(1,text('last'))]),spec(True,links=[])],
                 [spec(False,links=[link(1,b'\xff\xfd'+struct.pack('<HHH',0x4e2d,0xd800,0))]),
                  spec(True,links=[link(1,b'\xff\xfd'+struct.pack('<HHH',0x4e2d,0xd800,0))])],
                 [spec(False,links=[link(1,b'\xff\xfd'+b'x\x00'*511+b'\0\0'+b'y\x00'*8)]),
                  spec(True,links=[link(1,b'\xff\xfd'+b'x\x00'*511+b'\0\0')])]]
        rows=[run(c) for c in cases]
        rows.append(run([spec(False,'X'),spec(False,'Y'),spec(False,'Z')],
                        [0x100000000,0xffffffffffffffff,0]))
        rows.append(run([spec(False,'X'),spec(False,'Y')],[777,777]))
        corpus=[]
        if corpus_report:
            report=json.loads(corpus_report.read_text(encoding='utf8'))
            for f in report['files']:
                if f['execution_status']!='completed':raise ValueError('Incomplete corpus report')
                doc=json.loads((Path(f['output'])/'document.json').read_text(encoding='utf8'))
                for container in doc['initial_view_table_inputs']:
                    specs=[];identities=[]
                    for t in container['tables']:
                        if t['status']=='not_a_view_table_input':continue
                        if t['status']!='conditional':raise ValueError('Unresolved table input')
                        record=doc['native_records'][t['source']['native_record_index']]
                        links=[]
                        for l in record['links']:
                            if l['app']!=0x56d2 or not l['header']&0x1000:continue
                            payload=base64.b64decode(l['payload']['base64'])
                            if len(payload)<2:raise ValueError('Truncated string key')
                            if struct.unpack_from('<H',payload)[0] not in [1,2]:continue
                            links.append({'header':l['header'],'app':l['app'],'payload_hex':payload.hex()})
                        specs.append(spec(t['table_kind']=='lite',links=links));identities.append(t['source'])
                    selection=container['file_table_selection']
                    if selection['status']!='conditional':raise ValueError('Established direct file query required')
                    header_record=doc['native_records'][selection['query_source']['native_record_index']]
                    header_data=base64.b64decode(header_record['data']['base64'])
                    assert header_record['element_type']==46 and struct.unpack_from('<I',header_data,16)[0]==8
                    query=struct.unpack_from('<Q',header_data,0x24c)[0]
                    assert query==selection['requested_id']
                    row=run(specs,[s['assigned_id'] for s in identities],[query],header_data[4:0x254])
                    row.update({'source_file_sha256':f['sha256'],'source_identities':identities,
                                'query_source':selection['query_source']})
                    corpus.append(row)
        return {'scope':'original_file_table_getter_on_resident_input_with_default_model_minus2', 'dll_sha256':HASHES,
                'constructor_rva':'0x4b2f70','getter_rva':'0x4b2350','lookup_rva':'0x4af930','record_getter_rvas':['0x16e2a0','0xb5190','0xdaf60'],
                'active_table_selection':'not_evaluated','cases':rows,'corpus_cases':corpus}
    finally:
        for directory in directories:directory.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fixture',type=Path)
    parser.add_argument('--corpus-report',type=Path)
    args=parser.parse_args();result=probe(args.dll_root.resolve(),args.corpus_report)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    if args.fixture:
        fixture={k:v for k,v in result.items() if k!='corpus_cases'}
        args.fixture.write_text('#pragma once\n// Original R1.18 file-table lookup observations on synthetic input.\n'
            'inline constexpr const char* view_selection_oracle = R"oracle('
            +json.dumps(fixture,separators=(',',':'))+')oracle";\n',encoding='utf8')
    print(f"Observed {len(result['cases'])} synthetic and {len(result['corpus_cases'])} corpus collections: {args.output}")


if __name__=='__main__':main()
