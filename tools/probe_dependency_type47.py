#!/usr/bin/env python3
"""Original type-47 owner expansion, transform queries and initial callbacks.

Uses real constructed entities and bound type-13 references. Collector storage
follows the original inline constructor and uses its original vtable/vector
initializer. No replacement methods, binary patches, external loading or retry.
"""
import argparse
import json
import struct
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_dependency_reference_owners import probe
from probe_dependency_paths import payload
from probe_dependency_reference_affines import cases as affine_cases


def identity_cases():
    programs = [(f'path{format_}_{path}', [dict(id=44, links=[payload(format_,path,1,True)])])
                for format_ in (0,6) for path in ([],[41],[42],[41,42],[43,42],[41,43,42],[500],[0])]
    programs += [('nested_reference', [dict(id=44,links=[payload(0,[45],1,True)]),
                                      dict(id=45,links=[payload(0,[42],1,True)])]),
                 ('nested_deleted', [dict(id=44,links=[payload(0,[45],1,True)]),
                                    dict(id=45,links=[payload(0,[42],1,True)],runtime_flags=8)]),
                 ('nested_empty', [dict(id=44,links=[payload(0,[42,45],1,True)]),
                                  dict(id=45,links=[payload(0,[],1,True)])]),
                 ('nested_terminal', [dict(id=44,links=[payload(0,[45],1,True)]),
                                     dict(id=45,links=[payload(6,[42],1,True)])]),
                 ('first_match', [dict(id=44,links=[payload(6,[41],1,True),payload(0,[42],1,True)])]),
                 ('unrelated_key', [dict(id=44,links=[struct.pack('<4HQ',999,1,1,1,41).hex(),payload(0,[42],1,True)])]),
                 ('wrong_signature', [dict(id=44,links=[payload(0,[42],1,True)],signature=0x56e7)]),
                 ('wrong_subtype', [dict(id=44,links=[payload(0,[42],1,True)],subtype=19)]),
                 ('no_link', [dict(id=44,links=[])]),
                 ('wrong_format', [dict(id=44,links=[struct.pack('<4H2Q',10000,4,(4<<10)|1,1,41,42).hex()])]),
                 ('wrong_count', [dict(id=44,links=[payload(6,[41,42],2,True)])])]
    states = [(0,0,0,False),(8,0,0,False),(0x20000,0,0,False),(0,8,0,False),
              (0,8,0,True),(0,0x20000,0,False),(0,0,8,False),(0,0,8,True),(0,0,0x20000,False)]
    for label, owners in programs:
        for format_ in (4,6,0):
            outer = (struct.pack('<4H2Q',999,1,4<<10,1,41,44).hex()
                     if format_==4 else payload(format_,[41,44],1,False))
            for target_flags, owner_flags, path_flags, policy in states:
                for embedded_disabled in (False,True):
                    sources = json.loads(json.dumps(owners))
                    for source in sources:
                        source.setdefault('runtime_flags',path_flags if source['id']==44 else 0)
                        for i,link in enumerate(source['links']):
                            raw=bytearray.fromhex(link)
                            raw[4]=(raw[4]&~1)|int(embedded_disabled)
                            source['links'][i]=raw.hex()
                    yield dict(format=format_,path=[41,44],iterations=1,disabled=False,payload_hex=outer,
                               target_flags=target_flags,owner_flags=owner_flags,owner_lookup_includes_deleted=policy,
                               path_program=label,path_owners=sources,embedded_disabled=embedded_disabled)


def cases():
    baseline=list(identity_cases())
    yield from baseline
    profiles={}
    for row in affine_cases():profiles.setdefault(row['transform_case'],row['source_transforms'])
    for label,transforms in profiles.items():
        for row in baseline:
            if row['format']==4 and row['target_flags']==row['owner_flags']==0 and not row['owner_lookup_includes_deleted'] and row['embedded_disabled'] and row['path_owners'][0]['runtime_flags']==0:
                yield dict(row,transform_case=label,source_transforms=transforms)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dll-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fixture',type=Path)
    args=parser.parse_args()
    result=probe(args.dll_root.resolve(),cases())
    result['scope']='R1.18_original_type47_owner_expansion_and_callbacks'
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    if args.fixture:
        catalog=[];indices={};rows=[]
        for original in result['cases']:
            row={k:original[k] for k in ('format','disabled','payload_hex','target_flags','owner_flags',
                 'owner_lookup_includes_deleted','path_program','path_owners','calls')}
            if 'transform_case' in original:row['transform_case']=original['transform_case']
            row['source_indices']=[]
            for source in original['source_headers']:
                key=json.dumps(source,sort_keys=True,separators=(',',':'))
                if key not in indices:indices[key]=len(catalog);catalog.append(source)
                row['source_indices'].append(indices[key])
            assert [catalog[i] for i in row['source_indices']]==original['source_headers']
            rows.append(row)
        compact=dict(scope='R1.18_type47_initial_dependency_graph_fixture',
                     dll_sha256=result['dll_sha256'],source_catalog=catalog,cases=rows)
        args.fixture.write_text('#pragma once\n// Pure synthetic original type-47 owner path observations.\n'
            'inline constexpr const char* dependency_type47_oracle = R"oracle('
            +json.dumps(compact,separators=(',',':'))+')oracle";\n',encoding='utf8')
    print('Observed',len(result['cases']),'original type-47 cases')


if __name__=='__main__':main()
