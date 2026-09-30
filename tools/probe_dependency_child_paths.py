#!/usr/bin/env python3
"""Original parented type-47 expansion versus terminal append, without patches."""
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
from probe_dependency_parents import fixture


def cases():
    rotation=next(c['source_transforms'] for c in affine_cases() if c['transform_case']=='rotation')
    layouts={
        'flat':[(60,14,None),(50,62,0),(61,33,0),(46,47,0),(48,47,0),(51,62,0)],
        'deep':[(60,14,None),(63,14,0),(50,62,1),(61,33,1),(46,47,1),(48,47,0),(51,62,0)]}
    programs={
        'empty':dict(links=[payload(0,[],1,True)]),
        'blocks':dict(links=[payload(0,[50,51],1,True)]),
        'terminal_block':dict(links=[payload(6,[50],1,True)]),
        'wrong_signature':dict(links=[payload(0,[50],1,True)],signature=0x56e7),
        'wrong_subtype':dict(links=[payload(0,[50],1,True)],subtype=19),
        'missing_link':dict(links=[])}
    paths=[(0,[46]),(0,[48]),(0,[46,46]),(0,[42,46]),(0,[43,42,46]),
           (6,[46]),(6,[46,45]),(6,[46,50]),(6,[48,45]),(0,[48,45]),(0,[46,45]),(0,[42,48])]
    for layout,shape in layouts.items():
        for child_program,program in programs.items():
            for format_,path in paths:
                for state in ('normal','deleted_ancestor','deleted_child'):
                    for affine in (False,True):
                        tree=[]
                        for id_,type_,parent in shape:
                            spec=dict(id=id_,type=type_,parent=parent,runtime_flags=8 if
                                (state=='deleted_ancestor' and id_==60) or (state=='deleted_child' and id_==46) else 0,
                                links=[direct_payload([61,51,61],flags=1)])
                            if type_==62:spec.update(matrix=[1,0,0,0,2,0,0,0,1] if id_==50 else [0,-1,0,1,0,0,0,0,0.5],
                                                     translation=[-7,5,11] if id_==50 else [2,3,5])
                            if id_==46:spec.update(copy.deepcopy(program))
                            if id_==48:spec['links']=[payload(0,[46],1,True)]
                            tree.append(spec)
                        row=dict(format=4,path=[41,44],iterations=1,disabled=False,
                            payload_hex=struct.pack('<4H2Q',999,1,4<<10,1,41,44).hex(),
                            target_flags=0,owner_flags=0,owner_lookup_includes_deleted=False,
                            tree_layout=layout,tree_state=state,tree_sources=tree,child_program=child_program,
                            path_program=dict(format=format_,path=path),path_owners=[
                                dict(id=44,runtime_flags=0,links=[payload(format_,path,1,True)]),
                                dict(id=45,runtime_flags=0,links=[payload(0,[],1,True)])],
                            terminal_positions=[-1,0,1,2,9])
                        if affine:row.update(transform_case='rotation',source_transforms=copy.deepcopy(rotation))
                        yield row


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fixture',type=Path)
    args=parser.parse_args()
    result=probe(args.dll_root.resolve(),cases())
    result['scope']='R1.18_original_parented_type47_expansion_callbacks'
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    if args.fixture:args.fixture.write_text(fixture(result).replace('dependency_parents_oracle','dependency_child_paths_oracle'),encoding='utf8')
    print('Observed',len(result['cases']),'original type47 child cases')


if __name__=='__main__':main()
