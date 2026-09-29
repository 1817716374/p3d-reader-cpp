#!/usr/bin/env python3
"""Run original file-input entry 199e40, including original file callbacks.

Synthetic bounded host/model/storage, actual TLS service registries, native
entity allocation and native lazy bounds. No dependency linkages, geometry
regeneration, whole-file open, patched DLLs or Python native callbacks.
"""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_entity_input_ids import cases as id_cases
from probe_entity_registration import probe


def cases():
    rows=[]
    for case in id_cases():
        for flags in (0x20,0xe8):
            rows.append(dict(case,source_flags=flags,source_descendants=0xffffffff))
    for parents in ([-1,0,1,1,0,-1,5,6],[-1]*33):
        for flags in (0,0x400,0x800,0x1000):
            for spatial in (False,True):
                for source in (0x20,0xe8):
                    rows.append(dict(parents=parents,ids=[0]*len(parents),initial_counter=0,
                                     extended_flags=flags,spatial=spatial,source_flags=source,
                                     source_descendants=0xffffffff))
    return rows


def fixture(result):
    return '#pragma once\n// Original file-input entry, header preparation and file callbacks; bounded context.\ninline constexpr const char* entity_file_input_oracle = R"oracle('+json.dumps(result,separators=(',',':'))+')oracle";\n'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path)
    a=p.parse_args();result=probe(a.dll_root.resolve(),cases(),file_entry=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    if a.fixture:a.fixture.write_text(fixture(result),encoding='utf8')
    print('Observed',len(result['cases']),'complete file-input entry forests')


if __name__=='__main__':main()
