#!/usr/bin/env python3
"""Observe original R1.18 list-header validation and minimum base lengths."""
import argparse,ctypes as C,hashlib,json,os,struct,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_view_frame import HASHES

def cases():
    return [dict(type=t,subtype=s,extended=e,dimension=d,model_dimension=m)
            for t in range(0,101) for s in (range(12) if t in (10,49) else [0])
            for e,d in [(False,False),(True,False),(True,True)] for m in [False,True]]

def probe(root):
    if os.name!='nt' or C.sizeof(C.c_void_p)!=8:raise RuntimeError('Requires Windows x64')
    for name,expected in HASHES.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=expected:raise ValueError(name)
    dirs=[os.add_dll_directory(str(root/d)) for d in ['ROOT','SHARE','SHARE/vcredist/X64','PLATFORM']]
    try:
        dll=C.WinDLL(str(root/'ROOT/P3DKJ.dll'));base=dll._handle
        validate=C.CFUNCTYPE(C.c_int,C.c_void_p,C.c_void_p)(base+0x1a6300)
        rows=[]
        for case in cases():
            model=C.create_string_buffer(0x800);vt=C.create_string_buffer(0x70)
            listing=C.create_string_buffer(0x38);header=C.create_string_buffer(4096)
            C.c_void_p.from_buffer(model).value=C.addressof(vt)
            C.c_void_p.from_buffer(vt,0x48).value=base+0x8a80
            C.c_void_p.from_buffer(vt,0x68).value=base+(0x6570 if case['model_dimension'] else 0x8a80)
            C.c_void_p.from_buffer(listing).value=base+0x534050
            C.c_void_p.from_buffer(listing,0x30).value=C.addressof(model)
            struct.pack_into('<HHIII',header,0,case['type'],0x20 if case['extended'] else 0,2048,0,case['subtype'])
            struct.pack_into('<H',header,0x20,0x1000 if case['dimension'] else 0)
            before=bytes(header);code=validate(listing,header);after=bytes(header)
            assert before[:8]+before[12:]==after[:8]+after[12:]
            rows.append(dict(case,return_code=code,base_word_count=struct.unpack_from('<I',header,8)[0]))
        return dict(scope='R1.18_original_list_header_validator',dll_sha256=HASHES,cases=rows)
    finally:
        for d in dirs:d.close()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path);a=p.parse_args();result=probe(a.dll_root.resolve())
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    if a.fixture:a.fixture.write_text('#pragma once\n// Original R1.18 minimum base-length and validator observations.\ninline constexpr const char* list_header_oracle = R"oracle('+json.dumps(result,separators=(',',':'))+')oracle";\n',encoding='utf8')
    print('Observed',len(result['cases']),'list headers')
if __name__=='__main__':main()
