#!/usr/bin/env python3
"""Original R1.18 cached model bounds + full recursive 1dd110/1dc9f0 +
default table fill. Default graphs have no filter, active clip, scale service
or provider callbacks. Optional graphs exercise reviewed inline clipping.
Only original leaf getters are used for synthetic reference/model contexts.
Native allocations remain until exit; no patched code or Python callbacks.
"""
import argparse,copy,ctypes as C,hashlib,json,os,struct,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_view_frame import HASHES

def probe(root,graphs=None):
    if os.name!='nt' or C.sizeof(C.c_void_p)!=8:raise RuntimeError('Requires Windows x64')
    for name,h in HASHES.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=h:raise ValueError(name)
    dirs=[os.add_dll_directory(str(root/d)) for d in ['ROOT','SHARE','SHARE/vcredist/X64','PLATFORM']]
    try:
        dll=C.WinDLL(str(root/'ROOT/P3DKJ.dll'));base=dll._handle
        bounds_fn=C.CFUNCTYPE(C.c_int32,C.c_void_p,C.c_void_p)(base+0x19c6a0)
        union_fn=C.CFUNCTYPE(None,C.c_void_p,C.c_void_p,C.c_void_p,C.c_bool,C.c_bool,C.c_void_p)(base+0x1dd110)
        init=C.CFUNCTYPE(C.c_void_p,C.c_void_p,C.c_void_p,C.c_bool)(base+0x4ac3c0)
        fill=C.CFUNCTYPE(None,C.c_void_p,C.c_void_p,C.c_bool)(base+0x4acc80)
        maximum=float.fromhex('0x1.fffffffffffffp+1023');empty=[maximum]*3+[-maximum]*3
        rows=[]
        def run(nodes,roots,root_bounds):
            keep=[]
            def buf(n):
                b=C.create_string_buffer(n);keep.append(b);return b
            def ptr(b,offset,value):C.c_void_p.from_buffer(b,offset).value=value
            def model(bounds,spatial=True):
                m=buf(0x300);vt=buf(0xc8);cache=buf(0x20);provider=buf(0x80);pvt=buf(0x18)
                ptr(m,0,C.addressof(vt));ptr(vt,0x68,base+0x6570);ptr(vt,0xc0,base+0x37c90)
                C.c_int32.from_buffer(m,0xcc).value=1;C.c_double.from_buffer(m,0xd8).value=100000
                C.c_uint32.from_buffer(m,0xe0).value=1;C.c_uint32.from_buffer(m,0x118).value=2
                C.c_uint8.from_buffer(m,0x78).value=spatial;C.c_int32.from_buffer(m,0x1b8).value=7
                ptr(m,0x1c8,C.addressof(cache));ptr(cache,0x18,C.addressof(provider));ptr(provider,0,C.addressof(pvt))
                ptr(pvt,0x10,base+(0x37c40 if bounds is not None else 0x8a80))
                if bounds is not None:struct.pack_into('<6q',provider,0x50,*bounds)
                return m
            root_model=model(root_bounds);refs=[buf(0x500) for _ in nodes]
            for index,node in enumerate(nodes):
                ref=refs[index];vt=buf(0x70);ptr(ref,0,C.addressof(vt))
                ptr(vt,0x28,base+0x8560);ptr(vt,0x18,base+0x4d850)
                ptr(vt,0x68,base+0x6570)
                target=model(node['bounds'],node['spatial']) if node['target_present'] else None
                ptr(ref,0x18,C.addressof(target) if target else 0)
                if target is None:
                    provider_cache=buf(0x18);ptr(ref,0x4a0,C.addressof(provider_cache))
                header=buf(0x150);ptr(ref,0x10,C.addressof(header))
                ptr(ref,0x88,C.addressof(target) if node['attached'] and target else 0)
                C.c_uint32.from_buffer(header,0x10).value=1
                struct.pack_into('<3d',header,0x118,*node['origin'])
                struct.pack_into('<3d',ref,0x100,*node['translation'])
                struct.pack_into('<3d',ref,0x118,*node['reference_point'])
                C.c_double.from_buffer(ref,0x138).value=node['scale']
                struct.pack_into('<9d',ref,0x140,*node['matrix'])
                C.c_uint32.from_buffer(ref,0x9c).value=0x1000 if node['perspective'] else 0
                struct.pack_into('<3d',ref,0x1b0,*node['eye'])
                C.c_double.from_buffer(ref,0x1c8).value=node['distance']
                if 'clip' in node:
                    clip=node['clip'];points=clip['points']
                    assert len(points)<=2500 and target is not None
                    # Force the reviewed direct-matrix path; dimensional gate
                    # false uses a separate owner with an original false getter.
                    owner=buf(0x98);ovt=buf(0x70);ptr(owner,0,C.addressof(ovt))
                    ptr(ovt,0x68,base+0x8a80);ptr(ref,0x90,C.addressof(owner))
                    C.c_int32.from_buffer(target,0xcc).value=2 if clip['depths_allowed'] else 1
                    C.c_uint32.from_buffer(ref,0x9c).value|=0x80|clip['depth_flags']
                    struct.pack_into('<9d',ref,0x290,*clip['matrix'])
                    struct.pack_into('<2d',ref,0x280,clip['upper'],clip['lower'])
                    C.c_uint32.from_buffer(ref,0x2d8).value=len(points)
                    if points:
                        pp=buf(16*len(points));struct.pack_into('<'+'d'*(2*len(points)),pp,0,*[v for p in points for v in p])
                        ptr(ref,0x2e0,C.addressof(pp))
            def children(owner,indices):
                if not indices:return
                listing=buf(0x18);values=buf(8*len(indices))
                for i,index in enumerate(indices):ptr(values,8*i,C.addressof(refs[index]))
                ptr(listing,8,C.addressof(values));ptr(listing,16,C.addressof(values)+8*len(indices))
                ptr(owner,0x20,C.addressof(listing))
            children(root_model,roots)
            for ref,node in zip(refs,nodes):children(ref,node['children'])
            before=[bytes(ref) for ref in refs]
            out=(C.c_double*6)(*empty);code=bounds_fn(root_model,out)
            union_fn(out,root_model,None,False,False,None)
            assert [bytes(ref) for ref in refs]==before
            table=buf(0x120);init(table,None,False);C.c_uint32.from_buffer(table,8).value=1
            fill(table,root_model,False)
            for i in range(8):assert C.c_void_p.from_buffer(table,0x50+8*i).value
            v=C.string_at(C.c_void_p.from_buffer(table,0x50).value,0x190)
            rows.append({'root_bounds':copy.deepcopy(root_bounds),'root_bounds_code':code,
                         'roots':list(roots),'nodes':copy.deepcopy(nodes),
                         'combined_range':list(out),'table_slot0':{
                             'origin':list(struct.unpack_from('<3d',v,0x20)),
                             'delta':list(struct.unpack_from('<3d',v,0x38)),
                             'flags':list(struct.unpack_from('<3I',v,0x10))}})
        if graphs is not None:
            for graph in graphs:run(graph['nodes'],graph['roots'],graph['root_bounds'])
            return {'scope':'R1.18_default_filter_null_recursive_reference_bounds_inline_clipping',
                    'dll_sha256':HASHES,'native_allocations':'retained_until_process_exit','cases':rows}
        identity=[1,0,0,0,1,0,0,0,1]
        def node(**changes):
            n={'bounds':[-10,-20,-30,10,20,30],'spatial':True,'target_present':True,
               'attached':False,'origin':[1,2,3],'translation':[100,200,300],
               'reference_point':[1,2,3],'scale':2.,'matrix':identity,'perspective':False,
               'eye':[0,0,100],'distance':25.,'children':[]}
            n.update(changes);return n
        matrices=[identity,[0,-1,0,1,0,0,0,0,1],[-1,0,0,0,1,0,0,0,1],
                  [1,.2,.1,.3,1,.4,.5,.6,1],[1,0,0,0,.8,-.6,0,.6,.8],[0]*9]
        for m in matrices:
            for scale in [0.,.5,1.,-2.]:
                for attached in [False,True]:
                    for spatial in [False,True]:
                        for eye in [None,[0,0,100],[5,-7,0],[0,0,-100]]:
                            run([node(matrix=m,scale=scale,attached=attached,spatial=spatial,
                                      perspective=eye is not None,eye=eye or [0,0,100])],[0],None)
        for mode in range(32):
            nodes=[node(children=[1,2],scale=-1 if mode&1 else 2,attached=bool(mode&2)),
                   node(children=[3],translation=[-100,40,15],scale=.5,perspective=bool(mode&4)),
                   node(translation=[20,-80,5],matrix=matrices[3],perspective=bool(mode&8)),
                   node(translation=[1,2,3],spatial=bool(mode&16),scale=3)]
            run(nodes,[0,2,0],[-1,-2,-3,1,2,3])
            nodes[0]['bounds']=None;nodes[1]['bounds']=[10,20,30,-10,-20,-30]
            run(nodes,[0],None)
        run([],[],None);run([],[],[-1,-2,-3,1,2,3])
        run([node(target_present=False,children=[1]),node()],[0],None)
        run([node(bounds=None)],[0],None)
        run([node(scale=1e100)],[0],[-1,-2,-3,1,2,3])
        return {'scope':'R1.18_default_filter_null_recursive_reference_bounds_no_active_clip',
                'dll_sha256':HASHES,'native_allocations':'retained_until_process_exit','cases':rows}
    finally:
        for d in dirs:d.close()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path);a=p.parse_args();r=probe(a.dll_root.resolve())
    a.output.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n',encoding='utf8')
    if a.fixture:a.fixture.write_text('#pragma once\n// Original R1.18 recursive reference bounds observations.\ninline constexpr const char* reference_bounds_oracle = R"oracle('+json.dumps(r,separators=(',',':'),allow_nan=False)+')oracle";\n',encoding='utf8')
    print('Observed',len(r['cases']),'recursive bounds and full default-table cases')
if __name__=='__main__':main()
