#!/usr/bin/env python3
"""Observe full original lazy model bounds creation (19c6a0/df270/deeb0).

Synthetic bounded record handles and list blocks use original virtual leaves.
No patched instructions, Python callbacks, host or synthetic destructors.
All native allocations are retained until process exit.
"""
import argparse, ctypes as C, hashlib, json, os, random, struct, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_view_frame import HASHES

def record(words=None, runtime=0, flags=0x20, extended=0, override=None, key_mode='absent'):
    return dict(words=words or [-10,-20,-30,10,20,30,40], runtime_flags=runtime,
                element_flags=flags, extended_flags=extended, override=override, key_mode=key_mode)

def cases():
    rows=[]
    def add(records, blocks=None):
        for query in [False,True]:
            for spatial in [0,1,2]:
                rows.append(dict(records=records, blocks=blocks if blocks is not None else [list(range(len(records)))],
                                 model_query_68=query, spatial_byte_78=spatial))
    add([]); add([], [[],[],[]])
    for runtime in [0,8,0x20000,0x20008,0x400000,0x800000,0xc00000,0x400008]:
        for flags in [0,0x20,0xe0]:
            for extended in [0,0x400,0x800,0xc00,0x1000]:
                add([record(runtime=runtime,flags=flags,extended=extended)])
    for mode in ['absent','null','before','after','exact','sandwich']:
        for runtime in [0,0x400000,0x800000,0xc00000]:
            add([record(runtime=runtime,key_mode=mode,override=[-101,-102,-103,104,105,106])])
    limits=[-(1<<63),-(1<<63)+1,-(1<<52)-1,-(1<<52),0,(1<<52)-1,1<<52,(1<<63)-2,(1<<63)-1]
    for value in limits:
        for axis in range(6):
            r=record(); r['words'][axis]=value;add([r])
    for value in limits:
        add([record([value]*7)])
    # Shifted header range and nontrivial linked blocks, with repeated identity.
    add([record([9,-1,-2,-3,4,5,6],runtime=0x400000)])
    add([record(runtime=8),record(),record(runtime=0x400000,key_mode='sandwich',override=[-99,-88,-77,11,22,33]),record(extended=0x800)],
        [[],[0],[],[1,2,3],[],[2,1],[]])
    rng=random.Random(0xDF270)
    for count in [1,2,8,16,33,65,129,257]:
        records=[]
        for i in range(count):
            low=[rng.randrange(-10000,10000) for _ in range(3)]
            high=[v+rng.randrange(1,1000) for v in low]
            records.append(record(low+high+[0],runtime=0x400000 if i%7==0 else 0,
                                  key_mode='exact',override=[v-100 for v in low]+[v+100 for v in high]))
        add(records,[[],list(range(count//2)),[],list(range(count//2,count)),[]])
    return rows

def record_bytes(row):
    data=bytearray(132)
    struct.pack_into('<HHIIIQ',data,4,33,row['element_flags'],64,64,0,1)
    struct.pack_into('<H',data,36,row['extended_flags'])
    struct.pack_into('<7q',data,60,*row['words'])
    return data

def probe(root, inputs=None):
    if os.name!='nt' or C.sizeof(C.c_void_p)!=8:raise RuntimeError('Requires Windows x64')
    for name,h in HASHES.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=h:raise ValueError(name)
    dirs=[os.add_dll_directory(str(root/d)) for d in ['ROOT','SHARE','SHARE/vcredist/X64','PLATFORM']]
    try:
        dll=C.WinDLL(str(root/'ROOT/P3DKJ.dll'));base=dll._handle
        bounds_fn=C.CFUNCTYPE(C.c_int32,C.c_void_p,C.c_void_p)(base+0x19c6a0)
        get_provider=C.CFUNCTYPE(C.c_void_p,C.c_void_p)(base+0xdf270)
        rows=[]
        for row in cases() if inputs is None else inputs:
            keep=[]
            def buffer(n):
                b=C.create_string_buffer(n);keep.append(b);return b
            def ptr(b,at,v):C.c_void_p.from_buffer(b,at).value=v
            model=buffer(0x800);vt=buffer(0x70)
            ptr(model,0,C.addressof(vt));ptr(vt,0x28,base+0x8a80)
            ptr(vt,0x68,base+(0x6570 if row['model_query_68'] else 0x8a80))
            C.c_uint8.from_buffer(model,0x78).value=row['spatial_byte_78']
            entities=[];snapshots=[]
            for item in row['records']:
                data=record_bytes(item);header=buffer(len(data));C.memmove(header,bytes(data),len(data))
                entity=buffer(0x48);ptr(entity,0x40,C.addressof(header)+4)
                C.c_uint32.from_buffer(entity,0x10).value=item['runtime_flags']
                key=base+0x643e52;mode=item['key_mode'];entries=[]
                if mode!='absent':
                    payload=buffer(56)
                    if item['override'] is not None:struct.pack_into('<6q',payload,8,*item['override'])
                    for delta in ({'null':[0],'before':[-1],'after':[1],'exact':[0],'sandwich':[-1,0,1]}[mode]):
                        entry=buffer(24);ptr(entry,0,key+delta)
                        ptr(entry,8,None if mode=='null' else C.addressof(payload));entries.append(entry)
                    for a,b in zip(entries,entries[1:]):ptr(a,16,C.addressof(b))
                    ptr(entity,8,C.addressof(entries[0]))
                entities.append(entity);snapshots.extend([(header,bytes(header)),(entity,bytes(entity))])
            blocks=[]
            for indices in row['blocks']:
                block=buffer(0x28);array=buffer(max(8,8*len(indices)))
                for i,index in enumerate(indices):ptr(array,8*i,C.addressof(entities[index]))
                ptr(block,0x18,C.addressof(array));ptr(block,0x20,C.addressof(array)+8*len(indices));blocks.append(block)
            for a,b in zip(blocks,blocks[1:]):ptr(a,8,C.addressof(b))
            if blocks:ptr(model,0x158,C.addressof(blocks[0]))
            bounds=(C.c_double*6)(11,22,33,44,55,66)
            code=bounds_fn(model,bounds)
            cache=C.c_void_p.from_buffer(model,0x1c8).value;assert cache
            provider=C.c_void_p.from_address(cache+0x18).value;assert provider
            assert get_provider(cache)==provider
            node=C.c_void_p.from_address(provider+0x80).value;assert node
            assert all(bytes(obj)==before for obj,before in snapshots)
            rows.append(dict(row,return_code=code,double_range=list(bounds),
                             integer_range=list(struct.unpack('<6q',C.string_at(node,48))),
                             leaf_capacity=C.c_uint64.from_address(provider+0x98).value,
                             root_kind=C.c_uint32.from_address(node+0x38).value,
                             provider_spatial=C.c_uint8.from_address(provider+0x88).value))
        return dict(scope='R1.18_full_lazy_model_bounds_initial_population',dll_sha256=HASHES,
                    native_allocations='retained_until_process_exit',cases=rows)
    finally:
        for d in dirs:d.close()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path);a=p.parse_args();result=probe(a.dll_root.resolve())
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    if a.fixture:a.fixture.write_text('#pragma once\n// Original R1.18 full lazy bounds provider observations.\ninline constexpr const char* model_bounds_provider_oracle = R"oracle('+json.dumps(result,separators=(',',':'))+')oracle";\n',encoding='utf8')
    print('Observed',len(result['cases']),'lazy model bounds providers')
if __name__=='__main__':main()
