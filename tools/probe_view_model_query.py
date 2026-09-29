#!/usr/bin/env python3
"""Observe R1.18 table model filtering and complete resident file model query.

Model/flag values describe current view storage. The full query uses one
record-backed cached table, an explicit matching type46 header and original
table constructor/refcount methods. No model object is acquired or loaded.
"""
import argparse, ctypes as C, hashlib, json, os, struct
from pathlib import Path

HASHES = {
    'ROOT/P3DKJ.dll': '37fc96b97d230ba31619af4fba09ee11b2a38bfffca616621f11b240079f5901',
    'ROOT/P3DKJJC.dll': 'd5735b0f6c706beafb5feb2a21933404c70eb0371d8d208bf48867a4c43a313c',
    'ROOT/P3DDC.dll': '1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03',
    'SHARE/p3dlibxml2.dll': 'ca24cc124168dd90958f79e72d2af9dbca1dd937338ce6dd07fb0500cf21270a',
}

def probe(root, corpus_report=None, constructor_probe=None):
    if os.name!='nt' or C.sizeof(C.c_void_p)!=8: raise RuntimeError('Requires Windows x64')
    for name, expected in HASHES.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=expected: raise ValueError(name)
    directories=[os.add_dll_directory(str(root/d)) for d in ['ROOT','SHARE','SHARE/vcredist/X64','PLATFORM']]
    try:
        dll=C.WinDLL(str(root/'ROOT/P3DKJ.dll')); base=dll._handle
        C.CFUNCTYPE(None)(base+0x39f3c0)()
        match=C.CFUNCTYPE(C.c_bool,C.c_void_p,C.c_int32,C.c_bool,C.c_int32)(base+0x4adc70)
        query=C.CFUNCTYPE(C.c_int32,C.c_void_p)(base+0x12ef50)
        ctor=C.CFUNCTYPE(C.c_void_p,C.c_void_p,C.c_void_p,C.c_bool)(base+0x4ac3c0)
        owned=[]
        def alloc(n):
            obj=C.create_string_buffer(n); owned.append(obj); return obj
        def ptr(obj,offset,value): C.c_void_p.from_buffer(obj,offset).value=value
        file,table,collection,listing,block=alloc(0x1000),alloc(0x120),alloc(0x48),alloc(0x38),alloc(0x48)
        context=C.addressof(file)+0x6c8
        ptr(file,0x6c8,base+0x535e10); ptr(file,0x768,C.addressof(file))
        assert ctor(table,file,False)==C.addressof(table)
        assert C.c_void_p.from_buffer(table).value==base+0x557b68
        # Keep an owning ref so query's original add/release cannot destroy
        # Python-owned storage. No destructor is ever called on this graph.
        C.c_int32.from_buffer(table,8).value=1
        vtable=alloc(0x120)
        for slot,rva in [(0x20,0x16e2a0),(0x60,0xb5190),(0x118,0xdaf60),(0x38,0x6570)]:
            ptr(vtable,slot,base+rva)
        header,record,body,table_record=alloc(0x254),alloc(0x48),alloc(0x120),alloc(0x48)
        struct.pack_into('<IIII',header,0,46,0x12a,0x12a,8)
        struct.pack_into('<Q',header,0x248,0x1122334455667788)
        struct.pack_into('<Q',body,0x10,0x1122334455667788)
        for obj,data in [(record,header),(table_record,body)]:
            ptr(obj,0,C.addressof(vtable));ptr(obj,0x38,C.addressof(data))
        ptr(table,0xe0,C.addressof(table_record))
        roots=alloc(8);ptr(roots,0,C.addressof(record))
        ptr(block,0x18,C.addressof(roots));ptr(block,0x20,C.addressof(roots)+8)
        ptr(listing,0x18,C.addressof(block));ptr(file,0x800,C.addressof(listing))
        tables=alloc(8);ptr(tables,0,C.addressof(table))
        ptr(collection,8,C.addressof(tables));ptr(collection,16,C.addressof(tables)+8)
        ptr(collection,0,base+0x557ae0);ptr(collection,0x20,C.addressof(file))
        C.c_int32.from_buffer(file,0x18c).value=-2
        ptr(file,0xf30,C.addressof(collection))
        views=[alloc(0x190) for _ in range(8)]
        def set_views(models,flags):
            assert len(models)==len(flags)==8
            for i,(model,flag) in enumerate(zip(models,flags)):
                ptr(table,0x50+i*8,C.addressof(views[i]) if model is not None else 0)
                if model is not None: C.c_int32.from_buffer(views[i],0xe8).value=model
                C.c_uint8.from_buffer(views[i],0x10).value=flag
        def run_query(models,flags,dirty=None):
            assert all(m is not None for m in models)
            dirty=dirty if dirty is not None else [0]*8
            set_views(models,flags)
            for i,v in enumerate(dirty): C.c_uint8.from_buffer(table,0xd0+i).value=v
            result=query(file)
            after=list(C.string_at(C.addressof(table)+0xd0,8))
            assert C.c_int32.from_buffer(table,8).value==1
            return {'models':models,'flags_low':flags,'dirty_before':dirty,'model_id':result,'dirty_after':after}

        patterns=[[None]*8]+[[m]*8 for m in [-2147483648,-3,-2,-1,0,7,2147483647]]
        patterns += [[-2,7]+[None]*6,[7,-2]+[None]*6,[None,-2,None,7,None,7,None,None],
                     [7,None,7,None,7,None,7,None],[-2]*7+[7],[7]+[-2]*7,
                     [-2147483648,-3,-2,-1,0,7,2147483647,99]]
        filter_rows=[]
        for models in patterns:
            for mask in [0,1,2,128,255]:
                flags=[(0x80 if mask>>i&1 else 0)|i for i in range(8)]
                set_views(models,flags); observations=[]
                for requested in [-2147483648,-3,-2,-1,0,7,99,2147483647]:
                    for all_same,slot in [(True,-1)]+[(False,s) for s in [-2147483648,-1,0,1,7,8,2147483647]]:
                        # Skip precisely paths that would dereference a null
                        # view. The all_same branch explicitly skips nulls.
                        if not all_same:
                            if 0<=slot<=7:
                                if models[slot] is None:continue
                            else:
                                stop=next((i for i,f in enumerate(flags) if f&0x80),7)
                                if any(models[i] is None for i in range(stop+1)):continue
                        observations.append({'requested_id':requested,'all_same':all_same,'slot':slot,
                                             'matches':bool(match(table,requested,all_same,slot))})
                filter_rows.append({'models':models,'flags_low':flags,'queries':observations})
        query_rows=[]
        models=[-2147483648,-3,-2,-1,0,7,2147483647,99]
        for mask in range(256):
            flags=[(0x80 if mask>>i&1 else 0)|i for i in range(8)]
            for dirty in [[0]*8,[0x7f,0x80,2,3,4,5,6,0xff]]:
                query_rows.append(run_query(models,flags,dirty))
        absent_rows=[]
        for empty in [False,True]:
            struct.pack_into('<Q',header,0x248,0xfedcba9876543210)
            ptr(collection,16,C.addressof(tables)+(0 if empty else 8))
            before=list(C.string_at(C.addressof(table)+0xd0,8))
            absent_rows.append({'empty_collection':empty,'model_id':query(file),
                                'table_dirty_unchanged':before==list(C.string_at(C.addressof(table)+0xd0,8))})
            assert C.c_int32.from_buffer(table,8).value==1
        struct.pack_into('<Q',header,0x248,0x1122334455667788)
        ptr(collection,16,C.addressof(tables)+8)
        corpus=[]
        if corpus_report:
            if not constructor_probe:raise ValueError('--constructor-probe is required for corpus')
            report=json.loads(corpus_report.read_text(encoding='utf8'))
            constructor=json.loads(constructor_probe.read_text(encoding='utf8'))
            def key(sha,source,slot,profile):return (sha,json.dumps(source,sort_keys=True),slot,profile)
            bykey={key(r['source_file_sha256'],r['source'],r['slot_index'],r['profile']):r['constructor_model_id']
                   for r in constructor['corpus_cases']}
            for f in report['files']:
                assert f['execution_status']=='completed'
                doc=json.loads((Path(f['output'])/'document.json').read_text(encoding='utf8'))
                for container in doc['initial_view_table_inputs']:
                    for table_input in container['tables']:
                        if table_input['status']!='conditional':continue
                        assert len(table_input['slots'])==8
                        flags=[s['constructor_words'][0]&0xff for s in table_input['slots']]
                        for profile in ['empty_no_host','resident','host_accept']:
                            models=[bykey[key(f['sha256'],s['source'],s['slot_index'],profile)] for s in table_input['slots']]
                            row=run_query(models,flags)
                            row.update({'source_file_sha256':f['sha256'],'table_source':table_input['source'],'profile':profile})
                            corpus.append(row)
        return {'scope':'R1.18_current_view_values_with_one_matching_resident_table','dll_sha256':HASHES,
                'filter_rva':'0x4adc70','file_query_rva':'0x12ef50','table_constructor_rva':'0x4ac3c0',
                'model_loading':'not_executed','filter_cases':filter_rows,'query_cases':query_rows,
                'absent_cases':absent_rows,'corpus_cases':corpus}
    finally:
        for d in directories:d.close()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['dll-root','output']:p.add_argument('--'+name,type=Path,required=True)
    for name in ['fixture','corpus-report','constructor-probe']:p.add_argument('--'+name,type=Path)
    a=p.parse_args();r=probe(a.dll_root.resolve(),a.corpus_report,a.constructor_probe)
    a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf8')
    if a.fixture:
        f={k:v for k,v in r.items() if k!='corpus_cases'}
        a.fixture.write_text('#pragma once\n// Original R1.18 current-view model query observations.\ninline constexpr const char* view_model_query_oracle = R"oracle('+json.dumps(f,separators=(',',':'))+')oracle";\n',encoding='utf8')
    print({'filter_queries':sum(len(c['queries']) for c in r['filter_cases']),
           'file_queries':len(r['query_cases']),'corpus_queries':len(r['corpus_cases'])})
if __name__=='__main__':main()
