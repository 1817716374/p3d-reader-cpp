#!/usr/bin/env python3
"""Complete dependency iteration in an original constructed, caller-held model.

Includes surviving missing/excluded targets and the original model acquisition,
link callback and release paths. No application handlers, attribute providers,
final model unload or outer host transaction/flush are exercised.
"""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_dependency_retry import cases
from probe_entity_registration import probe

def fixture(result):
    return '#pragma once\n// Original constructed file/model, caller-held complete dependency iteration.\ninline constexpr const char* dependency_held_cycle_oracle = R"oracle('+json.dumps(result,separators=(',',':'))+')oracle";\n'

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path);a=p.parse_args()
    r=probe(a.dll_root.resolve(),cases(),file_entry=True,dependencies=True,dependency_retry=True,dependency_cycle=True,constructed=True)
    a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf8')
    if a.fixture:a.fixture.write_text(fixture(r),encoding='utf8')
    print('Observed',len(r['cases']),'complete dependency iterations in original constructed models')

if __name__=='__main__':main()
