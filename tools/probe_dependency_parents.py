#!/usr/bin/env python3
"""Original input subtrees, actual entity parents and owner-path collection.

Only input nodes are linked before 199e40. Native entity parent/child pointers
are observed after creation, never patched. Blocks use original 1076b0 repair.
"""
import argparse
import copy
import json
import struct
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_dependency_reference_owners import probe
from probe_dependency_reference_affines import cases as affine_cases
from probe_dependency_paths import payload
from probe_dependency_registration import payload as direct_payload


def cases():
    layouts={
        'flat':[(60,14,None),(50,62,0),(61,33,0),(51,62,0)],
        'deep':[(60,14,None),(63,14,0),(50,62,1),(61,33,1),(51,62,0)],
        'missing_child_flag':[(60,14,None),(63,14,0),(50,62,1),(61,33,1),(51,62,0)],
        'two_roots':[(60,14,None),(50,62,0),(61,33,0),(63,14,None),(51,62,3)],
    }
    matrices={'flat':[1,0,0,0,1,0,0,0,1],'deep':[0,0,0,0,2,0,0,0,2],
              'missing_child_flag':[0]*9,'two_roots':[-2,3,0,0,1,0.5,0,0,4]}
    rotation=next(c['source_transforms'] for c in affine_cases() if c['transform_case']=='rotation')
    paths=[(0,[]),(0,[50]),(0,[51,50]),(0,[50,51]),(0,[50,50]),(0,[60,50]),
           (0,[42,50]),(0,[43,42,51,50]),(6,[50]),(6,[50,45]),(6,[50,51]),(6,[61,50])]
    for layout,shape in layouts.items():
        for format_,path in paths:
            for state in ('normal','deleted_ancestor','deleted_child'):
                for disabled in (False,True):
                    for affine in (False,True):
                        tree=[]
                        for id_,type_,parent in shape:
                            spec=dict(id=id_,type=type_,parent=parent,runtime_flags=8 if
                                (state=='deleted_ancestor' and id_==60) or (state=='deleted_child' and id_==50) else 0,
                                links=[direct_payload([61,51,61],flags=int(disabled))])
                            if type_==62:
                                spec.update(matrix=matrices[layout] if id_==50 else [0,-1,0,1,0,0,0,0,0.5],
                                            translation=[-7,5,11] if id_==50 else [2,3,5])
                            if layout=='missing_child_flag' and id_==63:spec['source_flags']=0x60
                            tree.append(spec)
                        row=dict(format=4,path=[41,44],iterations=1,disabled=False,
                            payload_hex=struct.pack('<4H2Q',999,1,4<<10,1,41,44).hex(),
                            target_flags=0,owner_flags=0,owner_lookup_includes_deleted=False,
                            tree_layout=layout,tree_state=state,tree_links_disabled=disabled,tree_sources=tree,
                            path_program=dict(format=format_,path=path),path_owners=[
                                dict(id=44,runtime_flags=0,links=[payload(format_,path,1,True)]),
                                dict(id=45,runtime_flags=0,links=[payload(0,[],1,True)])],
                            terminal_positions=[-1,0,1,2,9])
                        if affine:row.update(transform_case='rotation',source_transforms=copy.deepcopy(rotation))
                        yield row


def fixture(result):
    catalog=[];indices={};rows=[]
    for original in result['cases']:
        row={k:original[k] for k in ('tree_layout','tree_state','tree_sources',
            'tree_entities','input_batches','path_program','path_queries')}
        row['source_indices']=[]
        for source in original['source_headers']:
            key=json.dumps(source,sort_keys=True,separators=(',',':'))
            if key not in indices:indices[key]=len(catalog);catalog.append(source)
            row['source_indices'].append(indices[key])
        assert [catalog[i] for i in row['source_indices']]==original['source_headers']
        rows.append(row)
    compact=dict(scope=result['scope'],dll_sha256=result['dll_sha256'],
                 native_case_count=len(rows),source_catalog=catalog,cases=rows)
    compact['dependency_cases']=[]
    for original,row in zip(result['cases'],rows,strict=True):
        graph={k:original[k] for k in ('format','disabled','payload_hex','target_flags','owner_flags',
            'owner_lookup_includes_deleted','tree_layout','tree_state','tree_sources',
            'input_batches','path_program','path_owners','calls')}
        if 'transform_case' in original:graph['transform_case']=original['transform_case']
        graph['source_indices']=row['source_indices']
        compact['dependency_cases'].append(graph)
    return ('#pragma once\n// Synthetic original input trees and collected transforms.\n'
        'inline constexpr const char* dependency_parents_oracle = R"oracle('
        +json.dumps(compact,separators=(',',':'))+')oracle";\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fixture',type=Path)
    args=parser.parse_args()
    result=probe(args.dll_root.resolve(),cases())
    result['scope']='R1.18_original_parented_owner_path_callbacks'
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    if args.fixture:args.fixture.write_text(fixture(result),encoding='utf8')
    print('Observed',len(result['cases']),'original parented owner cases')


if __name__=='__main__':main()
