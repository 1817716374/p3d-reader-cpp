#!/usr/bin/env python3
"""Full original 4acc80 with cached integer bounds, no child models, and
native-created existing views/layouts. Original leaves provide bounded model
virtual slots. Native allocations remain until process exit; no host, patched
instructions, Python callbacks or synthetic object destructors are used.
"""
import argparse, ctypes as C, hashlib, json, math, os, struct, sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_view_frame import HASHES

def probe(root):
    if os.name!='nt' or C.sizeof(C.c_void_p)!=8:raise RuntimeError('Requires Windows x64')
    for name,h in HASHES.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=h:raise ValueError(name)
    dirs=[os.add_dll_directory(str(root/d)) for d in ['ROOT','SHARE','SHARE/vcredist/X64','PLATFORM']]
    try:
        dll=C.WinDLL(str(root/'ROOT/P3DKJ.dll'));base=dll._handle
        init=C.CFUNCTYPE(C.c_void_p,C.c_void_p,C.c_void_p,C.c_bool)(base+0x4ac3c0)
        fill=C.CFUNCTYPE(None,C.c_void_p,C.c_void_p,C.c_bool)(base+0x4acc80)
        bounds_fn=C.CFUNCTYPE(C.c_int32,C.c_void_p,C.c_void_p)(base+0x19c6a0)
        view_ctor=C.CFUNCTYPE(C.c_void_p,C.c_void_p,C.c_bool,C.c_void_p,C.c_int32,C.c_void_p,C.c_void_p,C.c_int8,C.c_bool)(base+0x4b8050)
        layout_ctor=C.CFUNCTYPE(C.c_void_p,C.c_void_p,C.c_void_p,C.c_void_p,C.c_int32,C.c_bool)(base+0x4b93e0)
        def observe(pointer,layout_pointer):
            data=C.string_at(pointer,0x190);layout=C.string_at(layout_pointer,0x40)
            return {'flags':list(struct.unpack_from('<3I',data,0x10)),
                    'origin':list(struct.unpack_from('<3d',data,0x20)),
                    'delta':list(struct.unpack_from('<3d',data,0x38)),
                    'orientation':list(struct.unpack_from('<9d',data,0x50)),
                    'half_depth':struct.unpack_from('<d',data,0x98)[0],
                    'values_a8_to_c0':list(struct.unpack_from('<4d',data,0xa8)),
                    'slot_index':struct.unpack_from('<I',data,0xc8)[0],
                    'state_188':bool(data[0x188]),
                    'layout_normalized':list(struct.unpack_from('<4d',layout,0x10)),
                    'layout_words':list(struct.unpack_from('<7H',layout,0x30))}
        rows=[]
        ordinary={'value_28':10.,'code_30':1,'code_68':2,'value_38':2.,'value_40':3.,'value_70':4.,'value_78':5.}
        def run(mask,query68,spatial,mode,special,raw_range,scale,model_present=True):
            model=C.create_string_buffer(0x300);vt=C.create_string_buffer(0xc8)
            C.c_void_p.from_buffer(model).value=C.addressof(vt)
            C.c_void_p.from_buffer(vt,0x68).value=base+(0x6570 if query68 else 0x8a80)
            C.c_void_p.from_buffer(vt,0xc0).value=base+(0x37c90 if scale is not None else 0x8a80)
            C.c_int32.from_buffer(model,0xcc).value=1
            if scale is not None:
                for field,at in [('value_28',0x28),('value_38',0x38),('value_40',0x40),('value_70',0x70),('value_78',0x78)]:
                    C.c_double.from_buffer(model,0xb0+at).value=scale[field]
                for field,at in [('code_30',0x30),('code_68',0x68)]:
                    C.c_uint32.from_buffer(model,0xb0+at).value=scale[field]
            C.c_uint8.from_buffer(model,0x78).value=spatial
            C.c_int32.from_buffer(model,0x1b8).value=7
            file=C.create_string_buffer(0x1000);directory=C.create_string_buffer(0x38);entry=C.create_string_buffer(0x88)
            if special:
                C.c_void_p.from_buffer(model,0xa0).value=C.addressof(file)
                C.c_void_p.from_buffer(file,0xec0).value=C.addressof(directory)
                C.c_void_p.from_buffer(directory,8).value=C.addressof(entry)
                C.c_void_p.from_buffer(directory,16).value=C.addressof(entry)+0x88
                C.c_int32.from_buffer(entry,0x28).value=7
                C.c_uint8.from_buffer(entry,0x54).value=1;C.c_uint8.from_buffer(entry,0x59).value=1
            cache=C.create_string_buffer(0x20);provider=C.create_string_buffer(0x80);pvt=C.create_string_buffer(0x18)
            C.c_void_p.from_buffer(model,0x1c8).value=C.addressof(cache)
            C.c_void_p.from_buffer(cache,0x18).value=C.addressof(provider)
            C.c_void_p.from_buffer(provider).value=C.addressof(pvt)
            C.c_void_p.from_buffer(pvt,0x10).value=base+(0x37c40 if raw_range is not None else 0x8a80)
            if raw_range is not None:struct.pack_into('<6q',provider,0x50,*raw_range)
            max_double=float.fromhex('0x1.fffffffffffffp+1023')
            bounds=(C.c_double*6)(*([max_double]*3+[-max_double]*3))
            bounds_result=bounds_fn(model,bounds)
            table=C.create_string_buffer(0x120);assert init(table,None,False)==C.addressof(table)
            C.c_int32.from_buffer(table,8).value=1
            parent=(C.c_int32*4)(0,0,1000,800);existing={};snapshots={}
            for i in range(8):
                if not (mask&(1<<i)):continue
                view=C.c_void_p();layout=C.c_void_p()
                r=(C.c_double*6)(1,2,3,11,22,33);v=(C.c_int32*4)(10+i,20+i,100+i,200+i)
                view_ctor(C.byref(view),False,model,i,r,v,1,True)
                C.c_uint32.from_address(view.value+0x10).value|=0x84
                layout_ctor(C.byref(layout),v,parent,i,bool(i%2))
                C.c_uint16.from_address(layout.value+0x3c).value=0x100+i
                C.c_void_p.from_buffer(table,0x50+8*i).value=view.value
                C.c_void_p.from_buffer(table,0x90+8*i).value=layout.value
                existing[str(i)]=observe(view.value,layout.value)
                snapshots[i]=(view.value,layout.value,C.string_at(view.value,0x190),C.string_at(layout.value,0x40))
            fill(table,model if model_present else None,mode)
            slots=[]
            for i in range(8):
                view=C.c_void_p.from_buffer(table,0x50+8*i).value
                layout=C.c_void_p.from_buffer(table,0x90+8*i).value
                assert view and layout
                if i in snapshots:
                    old_v,old_l,old_vdata,old_ldata=snapshots[i]
                    assert view==old_v and layout==old_l
                    assert C.string_at(view,0x190)==old_vdata and C.string_at(layout,0x40)==old_ldata
                else:
                    assert C.c_void_p.from_address(view+0x100).value
                    assert C.c_uint32.from_address(view+8).value==1
                slots.append(observe(view,layout))
            rows.append({'existing_mask':mask,'model_present':model_present,'model_query_68':query68,
                         'spatial_byte_78':spatial,'suppress_selection':mode,'special_directory':special,
                         'integer_bounds':raw_range,'bounds_result':bounds_result,'combined_range':list(bounds),
                         'fallback_scale':scale,'existing':existing,'slots':slots})
        good=[-10,-20,-30,10,20,30]
        for mask in range(256):
            for q in [False,True]:run(mask,q,bool(mask&4),bool(mask&1),bool(mask&2),good,ordinary)
        ranges=[None,[4,5,6,4,5,6],[10,20,30,-10,-20,-30],
                [1,2,3,1,4,6],[1,2,30,11,22,-30],
                [-9223372036854775808,0,0,9223372036854775807,1,1],
                [9223372036854775807,0,0,9223372036854775807,1,1],
                [0,0,-9223372036854775808,1,1,-9223372036854775808],
                [9223372036854775805]*3+[9223372036854775806]*3]
        scales=[ordinary,dict(ordinary,code_68=1),dict(ordinary,value_28=-30.),
                dict(ordinary,code_68=1,value_40=0.,value_28=123.),dict(ordinary,value_28=0.)]
        for bounds in ranges:
            for scale in scales:
                for q in [False,True]:
                    for spatial in [False,True]:
                        run(0,q,spatial,False,False,bounds,scale)
        for q in [False,True]:
            for mode in [False,True]:
                for special in [False,True]:run(0,q,True,mode,special,good,ordinary)
        for bounds in [None,[4,5,6,4,5,6]]:run(0,True,True,False,False,bounds,None)
        for mask in [1,2,128,85,170,254,255]:
            for mode in [False,True]:run(mask,False,True,mode,False,good,ordinary,False)
        geom=C.WinDLL(str(root/'ROOT/P3DGeomBase.dll'))
        invalid=C.CFUNCTYPE(C.c_bool,C.c_void_p)(geom._handle+0x80ed0)
        point=C.CFUNCTYPE(C.c_bool,C.c_void_p)(geom._handle+0x81030)
        predicate_ranges=[[0.,0.,0.,1.,1.,1.],[0.]*6,[1.,2.,3.,-1.,-2.,-3.],
                          [0.,0.,0.,0.,0.,1.]]
        for value in [1e100,-1e100,math.nextafter(1e100,0.),math.nextafter(-1e100,0.),
                      math.nextafter(1e100,math.inf),float('nan'),float('inf'),-float('inf')]:
            for axis in range(6):
                r=[0.,0.,0.,1.,1.,1.];r[axis]=value;predicate_ranges.append(r)
        predicates=[]
        for r in predicate_ranges:
            data=(C.c_double*6)(*r)
            predicates.append({'range_bits':[struct.pack('<d',v).hex() for v in r],
                               'invalid':bool(invalid(data)),'point':bool(point(data))})
        return {'scope':'R1.18_full_4acc80_cached_integer_bounds_no_child_models',
                'dll_sha256':HASHES,'native_allocations':'retained_until_process_exit',
                'range_predicates':predicates,'cases':rows}
    finally:
        for d in dirs:d.close()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path);a=p.parse_args();result=probe(a.dll_root.resolve())
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    if a.fixture:a.fixture.write_text('#pragma once\n// Original R1.18 full default-table observations.\ninline constexpr const char* default_view_table_oracle = R"oracle('+json.dumps(result,separators=(',',':'))+')oracle";\n',encoding='utf8')
    print('Observed',len(result['cases']),'complete default tables')
if __name__=='__main__':main()
