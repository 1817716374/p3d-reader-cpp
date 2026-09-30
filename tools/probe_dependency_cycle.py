#!/usr/bin/env python3
"""One complete original dependency iteration after initial input.

The model notification and entire 1eb240 run, including list normalization and
work-set cleanup. Cases have no surviving missing/excluded targets and no
other handler queues; they do not require later geometry callbacks. The outer
host transaction/notification loop 1f1a90 is not executed.
"""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_dependency_registration import payload
from probe_entity_registration import probe

def cases():
    rows=[]
    for parents in ([-1],[-1]*4,[-1,0,0,2],[-1,0,1,1,0,-1,5,6],[-1]*33):
        n=len(parents)
        for format_ in (0,1):
            for flags in (0,1,2,0x200):
                for notice in ((0,0),(1,0),(0,1),(2,2)):
                    links=[[payload([i+1,1,n,0,1,n],format_,flags)] for i in range(n)]
                    rows.append(dict(parents=parents,ids=list(range(1,n+1)),initial_counter=0,
                        extended_flags=0,spatial=False,dependency_payloads=links,
                        notice_list_flag=notice[0],notice_model_flag=notice[1]))
    # Multiple links, nonadjacent duplicates, unaffected existing dependents.
    for format_ in (0,1):
        for count in (1,2,8,24):
            for notify in (0,1):
                rows.append(dict(parents=[-1]*4,ids=[1,2,3,4],initial_counter=0,
                    extended_flags=0,spatial=False,notice_list_flag=notify,notice_model_flag=0,
                    dependency_payloads=[[payload([1]*count+[4,1,4],format_),payload([2,1,4,2],format_)],
                                         [payload([1,1],format_)],[payload([1,1],format_)],[]]))
    return rows

def fixture(result):
    return '#pragma once\n// Complete original dependency iteration in a bounded initial-input context.\ninline constexpr const char* dependency_cycle_oracle = R"oracle('+json.dumps(result,separators=(',',':'))+')oracle";\n'

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path);a=p.parse_args()
    r=probe(a.dll_root.resolve(),cases(),file_entry=True,dependencies=True,dependency_retry=True,dependency_cycle=True)
    a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf8')
    if a.fixture:a.fixture.write_text(fixture(r),encoding='utf8')
    print('Observed',len(r['cases']),'complete dependency iterations')

if __name__=='__main__':main()
