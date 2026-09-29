#!/usr/bin/env python3
"""Original file-service ID preparation followed by full entity registration.

File block input 105f70 -> 199e40 selects list flags (false,true,false),
therefore 1a5cc0 invokes file input-service +50 (original 192210). This probe
executes that entire preparation and registration for each root. It does not
execute header conversion, file callbacks, TLS transactions or complete load.
"""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_entity_registration import probe

def cases():
    maximum=(1<<64)-1
    rows=[]
    for parents in [[-1],[-1,0,0,-1],[-1,0,1,1,0,-1,5,6],[-1]*33]:
        n=len(parents)
        profiles=[[0]*n,[1]*n,[0 if i%2 else 1 for i in range(n)],
                  [maximum if i%3==2 else i%2 for i in range(n)],
                  list(reversed(range(1,n+1)))]
        for ids in profiles:
            for counter in [0,2,100,maximum]:
                for spatial in [False,True]:
                    rows.append(dict(parents=parents,ids=ids,initial_counter=counter,
                                     extended_flags=0x1000 if spatial else 0,spatial=spatial))
    return rows

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path)
    a=p.parse_args();result=probe(a.dll_root.resolve(),cases(),prepare_ids=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    if a.fixture:
        a.fixture.write_text('#pragma once\n// Original file-service ID preparation and full native registration.\ninline constexpr const char* entity_input_ids_oracle = R"oracle('+json.dumps(result,separators=(',',':'))+')oracle";\n',encoding='utf8')
    print('Observed',len(result['cases']),'file-service ID preparation forests')

if __name__=='__main__':main()
