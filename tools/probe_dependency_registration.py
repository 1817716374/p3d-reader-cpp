#!/usr/bin/env python3
"""Original file-entry callbacks with direct-ID 56d0 dependency links.

Runs complete 163a90 initialization and native model allocation-pool setup,
then complete 199e40 for each root. Current model only; system registry empty,
file fallback disabled by original false getter; no application plugins.
"""
import argparse,json,struct,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_entity_registration import probe

def payload(ids,format_=0,flags=0,owner=999,relation=1):
    data=struct.pack('<4H',owner,relation,flags|(format_<<10),len(ids))
    for i,id_ in enumerate(ids):
        data+=struct.pack('<Q',id_)
        if format_==1:data+=struct.pack('<Q',0x1234567800000000+i)
    return data.hex()

def cases():
    maximum=(1<<64)-1;rows=[]
    for parents in [[-1],[-1,-1,-1,-1],[-1,0,0,2],[-1,0,1,1,0,-1,5,6],[-1]*33]:
        n=len(parents)
        for ids in [list(range(1,n+1)),[0]*n,[1]*n,[maximum if i%2 else 0 for i in range(n)]]:
            for format_ in (0,1):
                for flags in (0,1,2,0x200):
                    links=[]
                    for i in range(n):
                        targets=[i+1,1,n,0,maximum,1]
                        links.append([payload(targets,format_,flags)])
                    rows.append(dict(parents=parents,ids=ids,initial_counter=0,extended_flags=0,spatial=False,dependency_payloads=links))
    # Source-order across multiple links, empty links, and a larger pointer list.
    for format_ in (0,1):
        for owner,relation in [(0,0),(10000,17),(65535,65535),(10000,4)]:
            if format_==0 and (owner,relation)==(10000,4):continue
            rows.append(dict(parents=[-1,0,0,-1],ids=[1,2,3,4],initial_counter=100,
                extended_flags=0x1000,spatial=True,dependency_payloads=[
                    [payload([],format_,owner=owner,relation=relation),payload([3,3,4,2],format_,owner=owner,relation=relation)],
                    [payload([1]*24,format_,owner=owner,relation=relation)],
                    [payload([1,2],format_,1,owner,relation),payload([2,1],format_,owner=owner,relation=relation)],[]]))
    for flags in (0,8,0x20000,0x20008,0x100000):
        rows.append(dict(parents=[-1,-1],ids=[1,2],initial_counter=0,extended_flags=0,spatial=False,
                         dependency_payloads=[[],[payload([1,1,0,999])]],post_root_flags={0:flags}))
    return rows

def fixture(result):
    return '#pragma once\n// Original file-input direct-ID dependency registration and per-root snapshots.\ninline constexpr const char* dependency_registration_oracle = R"oracle('+json.dumps(result,separators=(',',':'))+')oracle";\n'

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path);a=p.parse_args()
    result=probe(a.dll_root.resolve(),cases(),file_entry=True,dependencies=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    if a.fixture:a.fixture.write_text(fixture(result),encoding='utf8')
    print('Observed',len(result['cases']),'direct-ID dependency forests')

if __name__=='__main__':main()
