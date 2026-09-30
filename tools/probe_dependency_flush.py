#!/usr/bin/env python3
"""Original outer dependency flush with constructed config and default host.

Runs complete 1f1a90 including original config stack operations, default host
notifications and registry replacement. Caller-held model, no application
handlers, no attribute providers and no preexisting transaction history.
"""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_dependency_retry import cases
from probe_entity_registration import probe

def fixture(result):
    return '#pragma once\n// Complete original outer flush with constructed config and default host notifications.\ninline constexpr const char* dependency_flush_oracle = R"oracle('+json.dumps(result,separators=(',',':'))+')oracle";\n'

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path);a=p.parse_args()
    r=probe(a.dll_root.resolve(),cases(),file_entry=True,dependencies=True,dependency_retry=True,
            dependency_cycle=True,constructed=True,dependency_flush=True)
    a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf8')
    if a.fixture:a.fixture.write_text(fixture(r),encoding='utf8')
    print('Observed',len(r['cases']),'complete outer dependency flushes')

if __name__=='__main__':main()
