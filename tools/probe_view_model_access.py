#!/usr/bin/env python3
"""Observe original R1.18 view model acquisition and cache behavior.

Nonloading paths use bounded resident objects with explicit state bytes.
Forced paths execute only inspected file-guard, missing-model, or null model
context failures. The last case returns a model pointer despite helper error 1.
No host/TLS callback, patched instruction or synthetic destructor executes.
"""
import argparse, ctypes as C, hashlib, json, os
from pathlib import Path

HASHES = {
    'ROOT/P3DKJ.dll': '37fc96b97d230ba31619af4fba09ee11b2a38bfffca616621f11b240079f5901',
    'ROOT/P3DKJJC.dll': 'd5735b0f6c706beafb5feb2a21933404c70eb0371d8d208bf48867a4c43a313c',
    'ROOT/P3DDC.dll': '1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03',
    'SHARE/p3dlibxml2.dll': 'ca24cc124168dd90958f79e72d2af9dbca1dd937338ce6dd07fb0500cf21270a',
}

def probe(root, constructor_probe=None):
    if os.name!='nt' or C.sizeof(C.c_void_p)!=8:raise RuntimeError('Requires Windows x64')
    for name,expected in HASHES.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=expected:raise ValueError(name)
    directories=[os.add_dll_directory(str(root/d)) for d in ['ROOT','SHARE','SHARE/vcredist/X64','PLATFORM']]
    try:
        dll=C.WinDLL(str(root/'ROOT/P3DKJ.dll')); base=dll._handle
        getter=C.CFUNCTYPE(C.c_void_p,C.c_void_p,C.c_bool)(base+0x4b5800)
        preferred_getter=C.CFUNCTYPE(C.c_void_p,C.c_void_p)(base+0x4b5900)
        acquire=C.CFUNCTYPE(C.c_void_p,C.c_void_p,C.c_void_p,C.c_int32,C.c_bool,C.c_bool,C.c_bool)(base+0x12d0e0)
        def run(source,entries,state14=1,state154=1,*,cached=None,preferred=None,use_preferred=False,
                file_present=True,force=False,file_state=0,suppressed=()):
            owned=[]
            def alloc(n):
                obj=C.create_string_buffer(n);owned.append(obj);return obj
            def ptr(obj,offset,value):C.c_void_p.from_buffer(obj,offset).value=value
            file,view=alloc(0x2000),alloc(0x190)
            addresses={0:C.addressof(file)+0x6c8}
            for index in [1,2,3]:addresses[index]=C.addressof(alloc(0x800))
            inverse={v:k for k,v in addresses.items()}
            def identity(pointer):return inverse[pointer] if pointer else None
            vt=alloc(0x100)
            # Only forced null-context cases invoke this vtable: original
            # refcount leaves at +a8/+b0 and original return-null leaf at +28.
            ptr(vt,0xa8,base+0x7540);ptr(vt,0xb0,base+0x75a0);ptr(vt,0x28,base+0x8a80)
            states=[]
            for index,address in addresses.items():
                meta=alloc(0x20)
                a,b=(state14,state154) if index in [0,1] else (1,1)
                C.c_uint8.from_buffer(meta,0x14).value=a
                C.c_void_p.from_address(address+0x138).value=C.addressof(meta)
                C.c_uint8.from_address(address+0x154).value=b
                C.c_void_p.from_address(address).value=C.addressof(vt)
                C.c_int32.from_address(address+8).value=1
                C.c_void_p.from_address(address+0xa0).value=C.addressof(file)
                C.c_uint8.from_address(address+0x7a5).value=1 # registration already accounted for
                states.append({'object_index':index,'status_record_byte14':a,'model_byte154':b})
            def tree(items):
                nil=alloc(0x30);nil_addr=C.addressof(nil);nil[0x19]=b'\x01'
                for offset in [0,8,16]:ptr(nil,offset,nil_addr)
                def branch(items,parent):
                    if not items:return nil_addr
                    m=len(items)//2;key,index=items[m];node=alloc(0x30);address=C.addressof(node)
                    ptr(node,0,branch(items[:m],address));ptr(node,8,parent);ptr(node,16,branch(items[m+1:],address))
                    C.c_int32.from_buffer(node,0x20).value=key
                    ptr(node,0x28,addresses[index] if index is not None else 0)
                    return address
                ptr(nil,8,branch(sorted(items),nil_addr));return nil_addr
            ptr(file,0x6a8,tree(entries));ptr(file,0x6b8,tree([(key,None) for key in suppressed]))
            C.c_int32.from_buffer(file,0x680).value=file_state
            ptr(view,0xd0,addresses[cached] if cached is not None else 0)
            ptr(view,0xd8,addresses[preferred] if preferred is not None else 0)
            ptr(view,0xe0,C.addressof(file) if file_present else 0)
            C.c_int32.from_buffer(view,0xe8).value=source
            row={'stored_model_id':source,'registry_entries':entries,'states':states,'cached_before':cached,
                 'preferred':preferred,'use_preferred':use_preferred,'file_present':file_present,
                 'force_load':force,'file_state_680':file_state,'suppressed_ids':list(suppressed)}
            enters_loader=(force or use_preferred) and (not use_preferred or preferred is None) and cached is None and file_present and source!=-2
            if enters_loader:
                code=C.c_uint32(0xdeadbeef)
                result=acquire(file,C.byref(code),source,True,True,False)
                row['helper_return_code']=code.value;row['helper_object_index']=identity(result)
            before=bytes(view)
            result=preferred_getter(view) if use_preferred else getter(view,force)
            after=bytes(view)
            assert before[:0xd0]==after[:0xd0] and before[0xd8:]==after[0xd8:]
            assert all(C.c_int32.from_address(p+8).value==1 for p in addresses.values())
            row['returned_object_index']=identity(result)
            row['cached_after']=identity(C.c_void_p.from_buffer(view,0xd0).value)
            if row['cached_after'] is not None:
                # A cached hit survives changed ID, detached file and unavailable
                # model state. Both getters must leave the existing cache alone.
                ptr(view,0xe0,0);C.c_int32.from_buffer(view,0xe8).value=-2
                for p in addresses.values():C.c_uint8.from_address(p+0x154).value=0
                row['cached_requery']=[identity(getter(view,flag)) for flag in [False,True]]
            return row
        sources=[-2147483648,-3,-2,-1,0,1,7,8,2147483647]
        rows=[]
        for source in sources:
            registries=[[],[[source,None]],sorted(dict([(-2147483648,3),(0,None),(2147483647,2),(source,1)]).items())]
            for entries in registries:
                for a in [0,1,128,255]:
                    for b in [0,1,128,255]:rows.append(run(source,entries,a,b))
            for cached in [None,2]:
                for force in [False,True]:rows.append(run(source,[],cached=cached,file_present=False,force=force))
            rows.append(run(source,[],cached=2,file_present=True,force=True))
            for guard in [-2147483648,-1,0]:rows.append(run(source,[],force=True,file_state=guard))
            for preferred,cached in [(3,None),(3,2),(None,2),(None,None)]:
                rows.append(run(source,[],preferred=preferred,cached=cached,use_preferred=True))
            if source not in [-1,-2]:
                for entries,suppressed in [([],()),([[source,None]],()),([], (source,))]:
                    rows.append(run(source,entries,force=True,file_state=1,suppressed=suppressed))
                for a,b in [(0,0),(1,1),(128,255)]:
                    rows.append(run(source,[[source,1]],a,b,force=True,file_state=1))
                rows.append(run(source,[[source,1]],use_preferred=True,file_state=1))
        corpus=[]
        if constructor_probe:
            source=json.loads(constructor_probe.read_text(encoding='utf8'))
            for prior in source['corpus_cases']:
                if prior['profile']!='resident':continue
                model=prior['constructor_model_id']
                for profile,a,b,entries in [('resident_ready',1,1,[[model,1]]),('resident_not_ready',1,0,[[model,1]]),('resident_absent',1,1,[])]:
                    row=run(model,entries,a,b)
                    row.update({k:prior[k] for k in ['source_file_sha256','source','slot_index']})
                    row['profile']=profile;corpus.append(row)
        return {'scope':'R1.18_view_model_access_with_explicit_resident_state_and_bounded_load_failures',
                'dll_sha256':HASHES,'getter_rva':'0x4b5800','preferred_getter_rva':'0x4b5900',
                'load_helper_rva':'0x12d0e0','host_callbacks':'not_executed','cases':rows,'corpus_cases':corpus}
    finally:
        for d in directories:d.close()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['dll-root','output']:p.add_argument('--'+name,type=Path,required=True)
    for name in ['fixture','constructor-probe']:p.add_argument('--'+name,type=Path)
    a=p.parse_args();r=probe(a.dll_root.resolve(),a.constructor_probe)
    a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf8')
    if a.fixture:
        f={k:v for k,v in r.items() if k!='corpus_cases'}
        a.fixture.write_text('#pragma once\n// Original R1.18 model object/cache observations.\ninline constexpr const char* view_model_access_oracle = R"oracle('+json.dumps(f,separators=(',',':'))+')oracle";\n',encoding='utf8')
    print({'synthetic':len(r['cases']),'corpus':len(r['corpus_cases'])})
if __name__=='__main__':main()
