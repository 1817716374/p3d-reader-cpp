#!/usr/bin/env python3
"""Original prepared-tree registration, ID collision handling and cache update.

Executes 1a5da0, 199fa0 and 19c6a0 with real native entities and ID trees.
Input trees are already prepared; file-input callbacks and complete model load
are not executed. Native allocations remain until process exit. A bounded
record-storage page avoids page growth/release; no synthetic destructors,
Python native callbacks or patched code are used.
"""
import argparse,ctypes as C,hashlib,json,os,struct,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_view_frame import HASHES

def cases():
    rows=[]
    shapes=[[-1],[-1,0,0,-1],[-1,0,1,1,0,-1,5,6],[-1]*33]
    for parents in shapes:
        for ids in [list(range(1,len(parents)+1)),[1]*len(parents),[1+i%3 for i in range(len(parents))],[(1<<64)-1]*len(parents)]:
            for flags in [0,0x400,0x800,0x1000]:
                for spatial in [False,True]:
                    rows.append(dict(parents=parents,ids=ids,initial_counter=max(100,max(ids)),
                                     extended_flags=flags,spatial=spatial))
    return rows

def probe(root, input_cases=None, prepare_ids=False):
    if os.name!='nt' or C.sizeof(C.c_void_p)!=8:raise RuntimeError('Requires Windows x64')
    for name,h in HASHES.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=h:raise ValueError(name)
    dirs=[os.add_dll_directory(str(root/d)) for d in ['ROOT','SHARE','SHARE/vcredist/X64','PLATFORM']]
    try:
        dll=C.WinDLL(str(root/'ROOT/P3DKJ.dll'));base=dll._handle
        register=C.CFUNCTYPE(C.c_int,C.c_void_p,C.c_void_p,C.c_void_p,C.c_bool,C.c_bool,C.c_bool,C.c_double,C.c_bool)(base+0x1a5da0)
        prepare=C.CFUNCTYPE(C.c_int,C.c_void_p,C.c_void_p,C.c_bool,C.c_bool,C.c_bool)(base+0x1a5cc0)
        update=C.CFUNCTYPE(C.c_int,C.c_void_p,C.c_void_p,C.c_bool)(base+0x199fa0)
        getbounds=C.CFUNCTYPE(C.c_int,C.c_void_p,C.c_void_p)(base+0x19c6a0)
        rows=[]
        for case in cases() if input_cases is None else input_cases:
            keep=[]
            def buf(n):
                v=C.create_string_buffer(n);keep.append(v);return v
            def ptr(v,o,x):C.c_void_p.from_buffer(v,o).value=x
            def q(v,o,x):C.c_uint64.from_buffer(v,o).value=x
            def u(v,o,x):C.c_uint32.from_buffer(v,o).value=x
            model=buf(0x800);file=buf(0x1000);vt=buf(0x70)
            ptr(model,0,C.addressof(vt));ptr(vt,0x28,base+0x8a80)
            ptr(vt,0x68,base+(0x6570 if case['spatial'] else 0x8a80))
            ptr(model,0xa0,C.addressof(file));C.c_uint8.from_buffer(model,0x78).value=case['spatial']
            q(file,0x190,case['initial_counter'])
            if prepare_ids:
                # File-input service: original vtable +50 -> 192210, whose
                # constructor stores its file at +30. No host callback stub.
                service=buf(0xa0);ptr(service,0,base+0x534708)
                ptr(service,0x30,C.addressof(file));ptr(file,0xee0,C.addressof(service))
            # Exact pool configuration from 198300; allocation code is original.
            q(model,0xc8,0x60);q(model,0xd0,32);q(model,0xd8,32)
            ptr(model,0x610,base+0x533798)
            for off,value in [(0x630,0x1b8),(0x638,4),(0x640,32),(0x650,4),(0x670,0xd0),(0x678,4),(0x680,32),(0x690,4)]:q(model,off,value)
            storage=buf(0x28);page=buf(65536);ptr(storage,8,C.addressof(page));u(storage,0x10,65536)
            ptr(model,0x178,C.addressof(storage))
            listing=buf(0x38);block=buf(0x48);slots=buf(128*8)
            ptr(listing,0x20,C.addressof(block));ptr(listing,0x30,C.addressof(model))
            ptr(block,0,C.addressof(listing));ptr(block,0x18,C.addressof(slots));ptr(block,0x20,C.addressof(slots));ptr(block,0x28,C.addressof(slots)+128*8)
            ptr(model,0x158,C.addressof(block))
            children={i:[] for i in range(len(case['parents']))}
            for i,parent in enumerate(case['parents']):
                if parent>=0:children[parent].append(i)
            descendants={i:0 for i in children}
            for i in reversed(range(len(case['parents']))):
                descendants[i]=sum(1+descendants[c] for c in children[i])
            nodes=[];source_ranges=[];headers=[]
            for i,parent in enumerate(case['parents']):
                allocation=buf(0x20+0x48+128);address=C.addressof(allocation)+0x20;nodes.append(address)
                flags=0x20|(0x80 if parent>=0 else 0)|(0x40 if children[i] else 0)
                struct.pack_into('<HHIIIQ',allocation,0x68,14 if children[i] else 33,flags,64,64,0,case['ids'][i])
                struct.pack_into('<H',allocation,0x88,case['extended_flags'])
                size=1000*(i+1) if parent>=0 else 10+i
                r=[-size,-size-1,-size-2,size,size+1,size+2];source_ranges.append(r)
                struct.pack_into('<6q',allocation,0xa0,*r)
                if children[i]:struct.pack_into('<I',allocation,0xd0,descendants[i])
                headers.append(bytes(allocation)[0x68:0xe8])
            for parent,items in children.items():
                if not items:continue
                C.c_void_p.from_address(nodes[parent]+0x18).value=nodes[items[0]]
                C.c_uint32.from_address(nodes[parent]+0x30).value=1
                for a,b in zip(items,items[1:]):C.c_void_p.from_address(nodes[a]).value=nodes[b]
            roots=[i for i,parent in enumerate(case['parents']) if parent<0]
            calls=[]
            for index in roots:
                if prepare_ids:
                    assert prepare(listing,nodes[index],True,False,False)==0
                    prepared_ids=[C.c_uint64.from_address(n+0x58).value for n in nodes]
                    prepared_counter=C.c_uint64.from_buffer(file,0x190).value
                result=register(listing,nodes[index],None,False,True,False,0.,False)
                assert result==0
                entity=C.c_void_p.from_address(nodes[index]+0x28).value;assert entity
                calls.append(dict(root=index,register_return=result,update_return=update(model,entity,False)))
                if prepare_ids:
                    calls[-1].update(prepared_ids=prepared_ids,prepared_counter=prepared_counter)
            entities=[C.c_void_p.from_address(n+0x28).value for n in nodes];observed=[]
            for i,entity in enumerate(entities):
                header=C.c_void_p.from_address(entity+0x40).value
                parent=C.c_void_p.from_address(entity+0x20).value
                actual=C.string_at(header,128)
                assert actual[:16]+actual[24:]==headers[i][:16]+headers[i][24:]
                child_list=C.c_void_p.from_address(entity+0x38).value
                child_indices=[]
                if child_list:
                    begin=C.c_void_p.from_address(child_list).value;end=C.c_void_p.from_address(child_list+8).value
                    child_indices=[entities.index(C.c_void_p.from_address(p).value) for p in range(begin,end,8)]
                observed.append(dict(id=C.c_uint64.from_address(header+0x10).value,
                                     flags=C.c_uint32.from_address(entity+0x10).value,
                                     parent=entities.index(parent) if parent else None,children=child_indices,
                                     ordinal=C.c_uint32.from_address(entity+0x58).value))
            count=(C.c_void_p.from_buffer(block,0x20).value-C.addressof(slots))//8
            actual_roots=[entities.index(C.c_void_p.from_buffer(slots,8*i).value) for i in range(count)]
            output=(C.c_double*6)(11,22,33,44,55,66);code=getbounds(model,output)
            rows.append(dict(case,calls=calls,source_ranges=source_ranges,entities=observed,root_indices=actual_roots,
                             bounds_result=code,bounds=list(output),counter=C.c_uint64.from_buffer(file,0x190).value))
        scope=('R1.18_file_service_id_preparation_and_registration_without_file_callbacks' if prepare_ids
               else 'R1.18_prepared_tree_registration_and_cache_update_without_file_callbacks')
        return dict(scope=scope,dll_sha256=HASHES,cases=rows)
    finally:
        for d in dirs:d.close()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path);a=p.parse_args();result=probe(a.dll_root.resolve())
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    if a.fixture:a.fixture.write_text('#pragma once\n// Original prepared-tree entity allocation, registration and cache observations.\ninline constexpr const char* entity_registration_oracle = R"oracle('+json.dumps(result,separators=(',',':'))+')oracle";\n',encoding='utf8')
    print('Observed',len(result['cases']),'prepared entity forests')
if __name__=='__main__':main()
