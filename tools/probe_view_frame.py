#!/usr/bin/env python3
"""Observe full R1.18 4b7550 framing with bounded explicit model inputs.

The view starts with empty resources; flags and range are explicit. The model
has an explicit ID and either no file or a cached directory entry permitting
the spatial matrix. No model virtual callback, host or destructor is invoked.
"""
import argparse, ctypes as C, hashlib, json, math, os, struct
from pathlib import Path
HASHES = {
    'ROOT/P3DKJ.dll':'37fc96b97d230ba31619af4fba09ee11b2a38bfffca616621f11b240079f5901',
    'ROOT/P3DKJJC.dll':'d5735b0f6c706beafb5feb2a21933404c70eb0371d8d208bf48867a4c43a313c',
    'ROOT/P3DDC.dll':'1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03',
    'ROOT/P3DGeomBase.dll':'fbc5795841acf724ceaa5e7a68107401581a6a5b8ac8acdbd966945ce34f7313',
    'SHARE/p3dlibxml2.dll':'ca24cc124168dd90958f79e72d2af9dbca1dd937338ce6dd07fb0500cf21270a',
}
def probe(root):
    if os.name!='nt' or C.sizeof(C.c_void_p)!=8:raise RuntimeError('Requires Windows x64')
    for name,expected in HASHES.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=expected:raise ValueError(name)
    directories=[os.add_dll_directory(str(root/d)) for d in ['ROOT','SHARE','SHARE/vcredist/X64','PLATFORM']]
    try:
        dll=C.WinDLL(str(root/'ROOT/P3DKJ.dll'));base=dll._handle
        fn=C.CFUNCTYPE(None,C.c_void_p,C.c_int32,C.c_void_p,C.c_void_p,C.c_void_p,C.c_void_p,C.c_void_p,C.c_bool)(base+0x4b7550)
        def run(bounds,matrix,rect,preserve,slot,selected):
            view=C.create_string_buffer(0x190);model=C.create_string_buffer(0x200)
            C.c_int32.from_buffer(model,0x1b8).value=7
            file,directory,entry=C.create_string_buffer(0x1000),C.create_string_buffer(0x38),C.create_string_buffer(0x88)
            if preserve:
                C.c_void_p.from_buffer(model,0xa0).value=C.addressof(file)
                C.c_void_p.from_buffer(file,0xec0).value=C.addressof(directory)
                C.c_void_p.from_buffer(directory,8).value=C.addressof(entry)
                C.c_void_p.from_buffer(directory,16).value=C.addressof(entry)+0x88
                C.c_int32.from_buffer(entry,0x28).value=7
                C.c_uint8.from_buffer(entry,0x54).value=1
                C.c_uint8.from_buffer(entry,0x59).value=1
            r=(C.c_double*6)(*bounds);m=(C.c_double*9)(*matrix) if matrix is not None else None
            v=(C.c_int32*4)(*rect) if rect is not None else None
            flags=(C.c_uint32*3)(0x12345678,0x87654321,0xabcdef01)
            fn(view,slot,model,r,v,m,flags,selected)
            assert list(r)==bounds and (m is None or list(m)==matrix)
            assert C.c_void_p.from_buffer(view,0xd0).value==C.addressof(model)
            assert C.c_int32.from_buffer(view,0xe8).value==7
            assert list(struct.unpack_from('<3I',view,0x10))==[(flags[0]&~0x80)|(int(selected)<<7),flags[1],flags[2]]
            return {'range':bounds,'orientation':matrix,'viewport':rect,'preserve_spatial_orientation':preserve,
                    'slot':slot,'selected':selected,'origin':list(struct.unpack_from('<3d',view,0x20)),
                    'delta':list(struct.unpack_from('<3d',view,0x38)),
                    'result_orientation':list(struct.unpack_from('<9d',view,0x50)),
                    'half_depth':C.c_double.from_buffer(view,0x98).value,
                    'slot_index':C.c_uint32.from_buffer(view,0xc8).value}
        c,s=math.cos(.37),math.sin(.37)
        matrices=[None,[1,0,0,0,1,0,0,0,1],[0]*9,[2,0,0,0,3,0,0,0,4],
                  [c,-s,0,s,c,0,0,0,1],[-1,0,0,0,1,0,0,0,1],
                  [1,0,0,0,.8,-.6,0,.6,.8],[1,.2,.1,.3,1,.4,.5,.6,1],
                  [1+5e-13,0,0,0,1,0,0,0,1],[1,1e-12,0,0,1,0,0,0,1],
                  [1,1.01e-12,0,0,1,0,0,0,1],[-0.,2,3,-0.,4,5,6,7,8]]
        ranges=[[1,2,3,11,22,33],[-10,-20,-30,10,20,30],[4,5,6,4,5,6],
                [11,22,33,1,2,3],[-1e8,3e7,-2e6,9e8,4e7,2e6]]
        rectangles=[None,[0,0,100,100],[10,20,410,220],[10,20,210,420],[5,8,5,8],
                    [100,200,-100,-200],[2147483647,2147483647,2147483647,2147483647],
                    [-2147483648,-2147483648,2147483647,2147483647]]
        rows=[];slots=[-2,-1,0,7,8,-2147483648,2147483647,4]
        for matrix in matrices:
            for bounds in ranges:
                for rect in rectangles:
                    for preserve in [False,True]:
                        i=len(rows);rows.append(run(bounds,matrix,rect,preserve,slots[(i//2)%8],bool(i%2)))
        return {'scope':'R1.18_full_4b7550_with_explicit_flags_range_and_bounded_model_directory',
                'dll_sha256':HASHES,'constructor_rva':'0x4b7550','model_loading':'not_executed','cases':rows}
    finally:
        for d in directories:d.close()
def main():
    p=argparse.ArgumentParser(description=__doc__)
    for n in ['dll-root','output']:p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--fixture',type=Path);a=p.parse_args();r=probe(a.dll_root.resolve())
    a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf8')
    if a.fixture:a.fixture.write_text('#pragma once\n// Original R1.18 numeric view framing observations.\ninline constexpr const char* view_frame_oracle = R"oracle('+json.dumps(r,separators=(',',':'))+')oracle";\n',encoding='utf8')
    print('Observed',len(r['cases']),'full constructor frame cases')
if __name__=='__main__':main()
