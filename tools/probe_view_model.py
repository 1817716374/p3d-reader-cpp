#!/usr/bin/env python3
"""Observe original type11 constructor model-ID decisions on bounded input.

Complete original 4b46b0, original system context, resident signed-ID tree,
no attributes or linkages. Host +90 uses inspected original constant return
leaves; this controls acceptance, not a real host implementation. Model values
are non-dereferenced sentinel pointers. No synthetic destructor is called.
"""
import argparse, base64, ctypes as C, hashlib, json, os, struct
from pathlib import Path
HASHES = {
    'ROOT/P3DKJ.dll': '37fc96b97d230ba31619af4fba09ee11b2a38bfffca616621f11b240079f5901',
    'ROOT/P3DKJJC.dll': 'd5735b0f6c706beafb5feb2a21933404c70eb0371d8d208bf48867a4c43a313c',
    'ROOT/P3DDC.dll': '1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03',
    'SHARE/p3dlibxml2.dll': 'ca24cc124168dd90958f79e72d2af9dbca1dd937338ce6dd07fb0500cf21270a',
}

def probe(root, corpus_report=None):
    if os.name!='nt' or C.sizeof(C.c_void_p)!=8:raise RuntimeError('Requires Windows x64')
    for name,expected in HASHES.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=expected:raise ValueError(name)
    directories=[os.add_dll_directory(str(root/d)) for d in ['ROOT','SHARE','SHARE/vcredist/X64','PLATFORM']]
    try:
        dll=C.WinDLL(str(root/'ROOT/P3DKJ.dll'));base=dll._handle
        ctor=C.CFUNCTYPE(C.c_void_p,C.c_void_p,C.c_void_p)(base+0x4b46b0)
        def run(source,entries,host,default):
            owned=[]
            def alloc(n):
                obj=C.create_string_buffer(n);owned.append(obj);return obj
            def ptr(obj,offset,value):C.c_void_p.from_buffer(obj,offset).value=value
            file=alloc(0x1000);context=C.addressof(file)+0x6c8
            ptr(file,0x6c8,base+0x535e10);ptr(file,0x768,C.addressof(file))
            C.c_int32.from_buffer(file,0x18c).value=default
            nil=alloc(0x30);nil_addr=C.addressof(nil);nil[0x19]=b'\x01'
            for offset in [0,8,16]:ptr(nil,offset,nil_addr)
            def tree(items,parent):
                if not items:return nil_addr
                m=len(items)//2;key,present=items[m];node=alloc(0x30);address=C.addressof(node)
                ptr(node,0,tree(items[:m],address));ptr(node,8,parent);ptr(node,16,tree(items[m+1:],address))
                C.c_int32.from_buffer(node,0x20).value=key
                ptr(node,0x28,C.addressof(alloc(8)) if present else 0)
                return address
            ptr(nil,8,tree(sorted(entries),nil_addr));ptr(file,0x6a8,nil_addr)
            if host!='absent':
                vt=alloc(0x98);ptr(vt,0x90,base+(0x6570 if host=='accept' else 0x8a80))
                h=alloc(8);ptr(h,0,C.addressof(vt));ptr(file,0xee0,C.addressof(h))
            entity=alloc(0x20+0x48+0x120);address=C.addressof(entity)+0x20
            body=bytearray(0x120);struct.pack_into('<IIII',body,0,11,0x90,0x90,1)
            struct.pack_into('<i',body,0x10c,source);C.memmove(address+0x48,bytes(body),len(body))
            wrapper,output=alloc(48),alloc(0x190);ptr(wrapper,0x10,context);ptr(wrapper,0x18,address)
            assert ctor(output,wrapper)==C.addressof(output)
            assert C.c_void_p.from_buffer(output,0xe0).value==C.addressof(file)
            assert not C.c_void_p.from_buffer(output,0xd0).value
            assert not C.c_void_p.from_buffer(output,0x160).value
            return {'source_id':source,'registry_entries':entries,'host_result':host,'file_default_id':default,
                    'constructor_model_id':C.c_int32.from_buffer(output,0xe8).value,'model_object_bound':False}
        sets=[[],[[7,True]], [[-2147483648,True],[0,False],[2147483647,True]],
              [[-2,False],[-1,False],[0,True],[7,False],[2147483647,True]]]
        rows=[run(source,entries,host,default)
              for source in [-2147483648,-3,-2,-1,0,1,7,8,2147483647]
              for entries in sets for host in ['absent','reject','accept']
              for default in [-2147483648,-2,-1,0,99]]
        corpus=[]
        if corpus_report:
            report=json.loads(corpus_report.read_text(encoding='utf8'))
            for f in report['files']:
                assert f['execution_status']=='completed'
                d=json.loads((Path(f['output'])/'document.json').read_text(encoding='utf8'))
                default=d['file_header']['initial_probe']['default_model_id'];default=default if default<0x80000000 else default-0x100000000
                for container in d['initial_view_table_inputs']:
                    for table in container['tables']:
                        if table['status']!='conditional':continue
                        for slot in table['slots']:
                            if 'source' not in slot:continue
                            source=slot['source'];data=base64.b64decode(d['native_records'][source['native_record_index']]['data']['base64'])
                            model=struct.unpack_from('<i',data,0x110)[0]
                            for profile,entries,host in [('empty_no_host',[],'absent'),('resident',[[model,True]],'absent'),('host_accept',[],'accept')]:
                                row=run(model,entries,host,default)
                                row.update({'source_file_sha256':f['sha256'],'source':source,'slot_index':slot['slot_index'],'profile':profile})
                                corpus.append(row)
        return {'scope':'R1.18_type11_model_id_with_controlled_registry_and_host_result','dll_sha256':HASHES,
                'constructor_rva':'0x4b46b0','host_return_leaves':['0x6570','0x8a80'],
                'model_object_resolution':'not_evaluated','cases':rows,'corpus_cases':corpus}
    finally:
        for directory in directories:directory.close()

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dll-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--fixture',type=Path);p.add_argument('--corpus-report',type=Path)
    a=p.parse_args();r=probe(a.dll_root.resolve(),a.corpus_report)
    a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf8')
    if a.fixture:
        f={k:v for k,v in r.items() if k!='corpus_cases'}
        a.fixture.write_text('#pragma once\n// Original R1.18 model-ID observations on bounded synthetic input.\ninline constexpr const char* view_model_oracle = R"oracle('+json.dumps(f,separators=(',',':'))+')oracle";\n',encoding='utf8')
    print(f"Observed {len(r['cases'])} synthetic and {len(r['corpus_cases'])} corpus-field model queries")
if __name__=='__main__':main()
