#!/usr/bin/env python3
"""Original model notice and pending retry core after complete root input.

Executes 1efe40 and the complete 1e9640 retry phase, not the outer service
flush/geometry transaction. The initial monitored set and auxiliary queues
are empty. Synthetic config owns a real initialized Windows critical section.
"""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_dependency_registration import cases as load_cases,payload
from probe_entity_registration import probe

def cases():
    rows=[]
    # Omit the attribute-provider flag: its separate retry traversal is unknown.
    for row in load_cases()[:-1]:
        for a,b in ((0,0),(1,0),(0,1),(2,2)):
            rows.append(dict(row,notice_list_flag=a,notice_model_flag=b))
    for format_ in (0,1):
        for targets in ([2],[1,2,1,999],[2,2,0],[999],[]):
            for target_flags in (0,8,0x20000,0x20008):
                for dependent_flags in (0,8,0x20000):
                    rows.append(dict(parents=[-1,-1],ids=[1,2],initial_counter=0,
                        extended_flags=0,spatial=False,dependency_payloads=[[payload(targets,format_)],[]],
                        notice_list_flag=1,notice_model_flag=0,
                        pre_retry_flags={0:dependent_flags,1:target_flags}))
    return rows

def fixture(result):
    return '#pragma once\n// Original model notice and complete pending retry core, not a service flush.\ninline constexpr const char* dependency_retry_oracle = R"oracle('+json.dumps(result,separators=(',',':'))+')oracle";\n'

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path);a=p.parse_args()
    result=probe(a.dll_root.resolve(),cases(),file_entry=True,dependencies=True,dependency_retry=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    if a.fixture:a.fixture.write_text(fixture(result),encoding='utf8')
    print('Observed',len(result['cases']),'pending retry cases')

if __name__=='__main__':main()
