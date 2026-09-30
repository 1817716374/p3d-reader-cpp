#!/usr/bin/env python3
"""Original type-62 input, collector terminal bounds and reference composition.

Original 1076b0 matrix preparation, 199e40 entities, 1017b0 collection,
231ca0 terminal setter and 101dc0 transform; no replacement handlers,
patched parent links or external loading.
"""
import argparse
import json
import struct
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_dependency_reference_owners import probe
from probe_dependency_reference_affines import cases as affine_cases
from probe_dependency_paths import payload


def cases():
    profiles = {
        'translation':dict(matrix=[1,0,0,0,1,0,0,0,1],translation=[10,20,30]),
        'mirror_shear':dict(matrix=[-2,3,0,0,1,0.5,0,0,4],translation=[-7,5,11]),
        'zero_repair':dict(matrix=[0]*9,translation=[3,-5,7]),
        'short_column_repair':dict(matrix=[0,0,0,0,2,0,0,0,2],translation=[-2,4,9]),
    }
    references = {None:None}
    for row in affine_cases():references.setdefault(row['transform_case'],row['source_transforms'])
    paths = [(0,[]),(0,[50]),(0,[51,50]),(0,[50,51]),(0,[50,41,51]),
             (0,[42,50]),(0,[43,42,51,50]),(6,[50]),(6,[50,51]),
             (6,[41,50]),(6,[50,45]),(0,[45])]
    for profile, block in profiles.items():
        for transform, reference in references.items():
            for format_, path in paths:
                for flags in (0,8,0x20000):
                    row=dict(format=4,path=[41,44],iterations=1,disabled=False,
                        payload_hex=struct.pack('<4H2Q',999,1,4<<10,1,41,44).hex(),
                        target_flags=0,owner_flags=0,owner_lookup_includes_deleted=False,
                        block_profile=profile,block_sources=[dict(id=50,runtime_flags=flags,**block),
                            dict(id=51,runtime_flags=0,matrix=[0,-1,0,1,0,0,0,0,0.5],translation=[2,3,5])],
                        path_program=dict(format=format_,path=path),path_owners=[
                            dict(id=44,runtime_flags=0,links=[payload(format_,path,1,True)]),
                            dict(id=45,runtime_flags=0,links=[payload(0,[],1,True)])],
                        terminal_positions=[-1,0,1,2,9])
                    if reference is not None:row.update(transform_case=transform,source_transforms=reference)
                    yield row


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fixture',type=Path)
    args=parser.parse_args()
    result=probe(args.dll_root.resolve(),cases())
    result['scope']='R1.18_original_type62_collector_transforms'
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    if args.fixture:
        # Retain every distinct source/collector/transform observation, but
        # remove duplicate query sets caused only by graph runtime flags.
        catalog=[];indices={};rows=[];seen=set()
        for row in result['cases']:
            compact={k:row[k] for k in ('path_queries','source_headers','block_profile','path_program')}
            key=json.dumps(compact,sort_keys=True,separators=(',',':'))
            if key in seen:continue
            seen.add(key)
            compact.pop('source_headers');compact['source_indices']=[]
            for source in row['source_headers']:
                key=json.dumps(source,sort_keys=True,separators=(',',':'))
                if key not in indices:indices[key]=len(catalog);catalog.append(source)
                compact['source_indices'].append(indices[key])
            rows.append(compact)
        fixture=dict(scope=result['scope'],dll_sha256=result['dll_sha256'],
                     native_case_count=len(result['cases']),source_catalog=catalog,cases=rows)
        args.fixture.write_text('#pragma once\n// Pure synthetic original type-62 collector observations.\n'
            'inline constexpr const char* dependency_blocks_oracle = R"oracle('
            +json.dumps(fixture,separators=(',',':'))+')oracle";\n',encoding='utf8')
    print('Observed',len(result['cases']),'original type-62 cases')


if __name__=='__main__':main()
