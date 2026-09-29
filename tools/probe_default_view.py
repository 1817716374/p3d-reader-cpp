#!/usr/bin/env python3
"""Observe complete original R1.18 4b8050 default view construction.

Explicit model/directory/range inputs. Model virtual +0x68 uses original
constant leaf functions; +0xc0 uses original 37c40 returning model+0x50.
Native allocated views are retained until process exit, never destroyed as
synthetic objects. No patched instructions, Python callbacks or host calls.
"""
import argparse, ctypes as C, hashlib, json, os, struct, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_view_frame import HASHES

def probe(root):
    if os.name != 'nt' or C.sizeof(C.c_void_p) != 8:
        raise RuntimeError('Requires Windows x64')
    for name, expected in HASHES.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != expected:
            raise ValueError(name)
    directories = [os.add_dll_directory(str(root/d)) for d in
                   ['ROOT', 'SHARE', 'SHARE/vcredist/X64', 'PLATFORM']]
    try:
        dll = C.WinDLL(str(root/'ROOT/P3DKJ.dll')); base = dll._handle
        ctor = C.CFUNCTYPE(C.c_void_p, C.c_void_p, C.c_bool, C.c_void_p,
                          C.c_int32, C.c_void_p, C.c_void_p, C.c_int8, C.c_bool)(base+0x4b8050)
        rows = []
        slots = [-2147483648,-2,-1,0,1,7,8,2147483647]
        ranges = [[1,2,3,11,22,33], [-10,-20,-30,10,20,30],
                  [4,5,6,4,5,6], [11,22,33,1,2,3]]
        rects = [None,[0,0,100,100],[2,2,998,798],[5,8,5,8],
                 [-2147483648,-2147483648,2147483647,2147483647]]
        def run(entries, query68, record_value, preset, model_id=7):
            i = len(rows); selected = bool(i%2); state188 = bool((i//2)%2)
            model = C.create_string_buffer(0x200); vt = C.create_string_buffer(0xc8)
            C.c_void_p.from_buffer(model).value = C.addressof(vt)
            C.c_void_p.from_buffer(vt,0x68).value = base+(0x6570 if query68 else 0x8a80)
            C.c_void_p.from_buffer(vt,0xc0).value = base+0x37c40
            C.c_int32.from_buffer(model,0x6c).value = record_value
            C.c_int32.from_buffer(model,0x1b8).value = model_id
            file = C.create_string_buffer(0x1000); directory = C.create_string_buffer(0x38)
            storage = C.create_string_buffer(max(1, len(entries or []))*0x88)
            if entries is not None:
                C.c_void_p.from_buffer(model,0xa0).value = C.addressof(file)
                C.c_void_p.from_buffer(file,0xec0).value = C.addressof(directory)
                C.c_void_p.from_buffer(directory,8).value = C.addressof(storage)
                C.c_void_p.from_buffer(directory,16).value = C.addressof(storage)+len(entries)*0x88
                for j,e in enumerate(entries):
                    at = j*0x88
                    C.c_int32.from_buffer(storage,at+0x28).value = e['model_id']
                    C.c_int32.from_buffer(storage,at+0x50).value = e['value_50']
                    for field, offset in [('state_54',0x54),('excluded_55',0x55),('state_59',0x59)]:
                        C.c_uint8.from_buffer(storage,at+offset).value = e[field]
            bounds = ranges[i%len(ranges)]; rect = rects[i%len(rects)]; slot = slots[i%len(slots)]
            r = (C.c_double*6)(*bounds); v = (C.c_int32*4)(*rect) if rect else None
            result = C.c_void_p()
            assert ctor(C.byref(result),state188,model,slot,r,v,preset,selected) == C.addressof(result)
            assert result.value
            data = C.string_at(result.value,0x190)
            assert struct.unpack_from('<I',data,8)[0] == 1
            assert struct.unpack_from('<Q',data,0xd0)[0] == C.addressof(model)
            assert struct.unpack_from('<Q',data,0xd8)[0] == 0
            assert struct.unpack_from('<Q',data,0xe0)[0] == (C.addressof(file) if entries is not None else 0)
            assert struct.unpack_from('<i',data,0xe8)[0] == model_id
            assert list(r) == bounds and (v is None or list(v) == rect)
            rows.append({'entries':entries, 'model_id':model_id, 'model_query_68':query68,
                         'model_record_value_1c':record_value, 'preset':preset,
                         'range':bounds, 'viewport':rect, 'slot':slot,
                         'selected':selected, 'state_188':state188,
                         'result_state_188':bool(data[0x188]),
                         'flags':list(struct.unpack_from('<3I',data,0x10)),
                         'origin':list(struct.unpack_from('<3d',data,0x20)),
                         'delta':list(struct.unpack_from('<3d',data,0x38)),
                         'orientation':list(struct.unpack_from('<9d',data,0x50)),
                         'half_depth':struct.unpack_from('<d',data,0x98)[0],
                         'values_a8_to_c0':list(struct.unpack_from('<4d',data,0xa8)),
                         'slot_index':struct.unpack_from('<I',data,0xc8)[0]})
        def entry(model_id=7, value=0, s54=1, excluded=0, s59=1):
            return {'model_id':model_id,'value_50':value,'state_54':s54,
                    'excluded_55':excluded,'state_59':s59}
        for s54 in [0,1,128,255]:
            for s59 in [0,1,128,255]:
                for value in [-2147483648,0,1,2,2147483647]:
                    for excluded in [0,1,128,255]:
                        for query68, record in [(False,1),(False,0),(True,0)]:
                            run([entry(value=value,s54=s54,s59=s59,excluded=excluded)],query68,record,7)
        for preset in range(-128,128):
            run(None,False,1,preset)
            run([],True,2,preset)
            run([entry()],False,0,preset)
        for model_id in [-2147483648,-3,-2,-1,0,7,2147483647]:
            yes=entry(model_id); no=entry(model_id,s54=0); excluded=entry(model_id,excluded=255)
            other=entry(6 if model_id!=6 else 5)
            for entries in [[],[other],[excluded],[no,yes],[yes,no],
                            [excluded,yes],[other,yes],[excluded,no,yes],[other,excluded,yes]]:
                run(entries,False,-2147483648,8,model_id)
        return {'scope':'R1.18_full_4b8050_with_bounded_model_directory_and_explicit_range',
                'dll_sha256':HASHES,'constructor_rva':'0x4b8050',
                'model_loading':'not_executed','native_allocations':'retained_until_process_exit',
                'cases':rows}
    finally:
        for d in directories:d.close()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path)
    a=p.parse_args(); result=probe(a.dll_root.resolve())
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    if a.fixture:
        a.fixture.write_text('#pragma once\n// Original R1.18 allocated default view observations.\ninline constexpr const char* default_view_oracle = R"oracle('+json.dumps(result,separators=(',',':'))+')oracle";\n',encoding='utf8')
    print('Observed',len(result['cases']),'full default view constructor cases')
if __name__=='__main__':main()
