#!/usr/bin/env python3
"""Independent type-47 collector and complete initial dependency graph replay."""
import argparse
import copy
import json
import math
import struct
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_dependency_type47 import cases
from probe_view_frame import HASHES


def verify(report, expected_cases=None, expected_scope='R1.18_original_type47_owner_expansion_and_callbacks', expected_count=2030):
    assert report['scope']==expected_scope
    assert report['dll_sha256']==HASHES
    totals=dict(cases=0,root_callbacks=0,path_queries=0,successful_expansions=0,
                transform_failures=0,local_edges=0,other_model_edges=0,pending_entities=0,affine_queries=0,
                single_type47_transforms=0,single_type47_reference_transforms=0,terminal_queries=0)
    maximum=(1<<64)-1
    identity=[1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.]
    names={None:'null',0:'current',1:'reference_42',2:'nested_reference_43'}
    identity4=[identity[i*4:i*4+4] for i in range(3)]+[[0.,0.,0.,1.]]
    def multiply(a,b):return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
    def flat(matrix):return [v for line in matrix[:3] for v in line]
    def equal_matrix(actual,expected):
        return len(actual)==len(expected) and all(math.isclose(a,b,rel_tol=2e-14,abs_tol=2e-13) for a,b in zip(actual,expected))
    for supplied,row in zip(cases() if expected_cases is None else expected_cases,report['cases'],strict=True):
        assert all(row[k]==v for k,v in supplied.items())
        assert row['reference_ids']==[42,43,43]
        assert row['reference_parent_contexts']==['current','model_9','reference_42']
        assert row['target_model_ids']==[9,7,10]
        assert len(row['transitions'])==4
        for i,item in enumerate(row['transitions']):
            flags=row['owner_flags'] if i==0 else row['target_flags'] if i==3 else 0
            accepted=not (flags&8 and not row['owner_lookup_includes_deleted'])
            assert item==dict(owner_id=(42,43,43,41)[i],accepted=accepted,
                              result=('same_reference' if i==3 else 'child_reference') if accepted else 'null')
        local={}
        programs={s['id']:s for s in row['path_owners']}
        blocks={s['id']:s for s in row.get('block_sources',[])}
        block_matrices={}
        for id_,spec in blocks.items():
            linear=spec['matrix']
            # Independent closed-form results for these two singular profiles.
            if linear==[0]*9:linear=[1,0,0,0,1,0,0,0,1]
            elif linear==[0,0,0,0,2,0,0,0,2]:linear=[2,0,0,0,2,0,0,0,2]
            block_matrices[id_]=[linear[i*3:i*3+3]+[spec['translation'][i]] for i in range(3)]+[[0.,0.,0.,1.]]
        def entity(model,id_,type_,flags,index):
            return dict(model=model,id=id_,type=type_,flags=flags,index=index)
        models={1:{41:entity(9,41,33,row['target_flags'],0),43:entity(9,43,13,0,1)},
                2:{41:entity(10,41,33,row['target_flags'],0)}}
        def lookup(context,id_):
            return None if context is None else (local if context==0 else models[context]).get(id_)
        def decode(raw):
            owner,relation,flags,count=struct.unpack_from('<4H',raw)
            format_=(flags>>10)&15
            if format_==6:
                n=struct.unpack_from('<I',raw,8)[0]
                path=list(struct.unpack_from('<'+'Q'*n,raw,24))
            else:
                path=list(struct.unpack_from('<'+'Q'*count,raw,8)) if format_==0 else []
            return owner,relation,flags,count,format_,path
        def expand(target,context,collector):
            if target is None:return False
            if target['type'] in (33,62):
                collector['collected'].append(dict(model_id=target['model'],id=target['id']))
                if context is not None:collector['context']=context
                return True
            if target['type']==13:
                if context==0 and target['id']==42:collector['context']=1
                elif context==1 and target['id']==43:collector['context']=2
                else:raise AssertionError('unmodelled reference context')
                return True
            program=programs[target['id']]
            if program.get('signature',0x56e6)!=0x56e6 or program.get('subtype',20)!=20:return False
            selected=next((bytes.fromhex(s) for s in program['links']
                           if struct.unpack_from('<2H',bytes.fromhex(s))==(10000,4)),None)
            if selected is None:return False
            _,_,_,count,format_,path=decode(selected)
            if format_ not in (0,6) or (format_==6 and (count!=1 or not path)):return False
            if format_==0 and not path:
                if collector['context'] is None:collector['context']=context
                return True
            for id_ in reversed(path if format_==0 else path[1:]):
                node=lookup(context,id_)
                # This inner lookup always rejects deletion, independently of
                # the top-level service's include-deleted policy.
                if node is None or node['flags']&8:return False
                if not expand(node,context,collector):return False
                context=collector['context']
            if format_==6:
                node=lookup(context,path[0])
                if node is None or node['flags']&8:return False
                collector['collected'].append(dict(model_id=node['model'],id=node['id']))
            return True
        def collect(id_,context):
            node=lookup(context,id_)
            result=dict(context=None,collected=[])
            if node is None or (node['flags']&8 and not row['owner_lookup_includes_deleted']):return False,result
            return expand(node,context,result),result
        def transition(id_,context):
            success,result=collect(id_,context)
            return result['context'] if success and result['context'] in (1,2) else None
        def references(raw):
            owner,relation,flags,count,format_,path=decode(raw)
            if flags&1 or count==0:return []
            if format_==4:
                id_,owner_id=struct.unpack_from('<2Q',raw,8)
                success,collected=collect(owner_id,0)
                context=collected['context'] if success else None
                return [(id_,lookup(context,id_)),(owner_id,lookup(0,owner_id))]
            if format_==0 and (owner,relation)!=(10000,4):
                return [(id_,lookup(0,id_)) for id_ in path]
            if format_==6:
                if not path:return [(maximum,None)]*(count*2)
                context=0
                for id_ in reversed(path[1:]):
                    context=transition(id_,context)
                    if context is None:break
                first=(maximum,None) if context is None else (path[0],lookup(context,path[0]))
                return [first,(path[-1],lookup(0,path[-1]))]+[(maximum,None)]*(2*(count-1))
            assert format_==0
            result=[]
            for i in range(len(path)):
                context=0
                for id_ in reversed(path[i:]):
                    previous=context;context=transition(id_,context)
                    if context is None:break
                result.append((maximum,None) if context is None else (path[i],lookup(previous,path[i])))
            return result
        source_order=[(9,41,33),(10,41,33),(9,43,13),(7,77,33),(7,42,13),(7,41,33)]
        source_order += [(7,id_,62) for id_ in blocks]+[(7,s['id'],47) for s in row['path_owners']]+[(7,78,33)]
        assert [(s['model_id'],s['id'],s['type']) for s in row['source_headers']]==source_order
        affines={}
        for source in row['source_headers']:
            type_,id_=source['type'],source['id']
            size=368 if type_==13 else 34 if type_==47 else 256 if type_==62 else 128
            expected=bytearray(size)
            struct.pack_into('<HHIIIQ',expected,0,type_,4 if type_==47 else 0x20,size//2,size//2,
                             programs[id_].get('subtype',20) if type_==47 else 0,id_)
            if type_==13:
                transform=row.get('source_transforms',{}).get(str(id_),{})
                linear=transform.get('matrix',[1,0,0,0,1,0,0,0,1])
                translation=transform.get('translation',[0,0,0]);point=transform.get('reference_point',[0,0,0])
                scale=transform.get('scale',1)
                struct.pack_into('<3d',expected,168,*point);struct.pack_into('<3d',expected,192,*translation)
                struct.pack_into('<9d',expected,216,*linear);struct.pack_into('<d',expected,288,scale)
                # Four reviewed profiles: unit orthogonal or nonrigid source
                # bases, so no extra rigid basis scale extraction is selected.
                lengths=[math.sqrt(sum(linear[i*3+j]**2 for i in range(3))) for j in range(3)]
                matrix=[[(linear[i*3+j]/lengths[j] if lengths[j] else float(i==0))*(scale or 1.) for j in range(3)] for i in range(3)]
                for i in range(3):matrix[i].append(translation[i]-sum(matrix[i][j]*point[j] for j in range(3)))
                affines[id_]=matrix+[[0.,0.,0.,1.]]
            elif type_==62:
                struct.pack_into('<9d',expected,160,*blocks[id_]['matrix'])
                struct.pack_into('<3d',expected,232,*blocks[id_]['translation'])
            elif type_==47:struct.pack_into('<H',expected,32,programs[id_].get('signature',0x56e6))
            else:struct.pack_into('<6q',expected,56,-1,-2,-3,1,2,3)
            links=programs[id_]['links'] if type_==47 else [row['payload_hex']] if id_ in (77,78) else []
            for link in links:
                data=bytes.fromhex(link);expected+=struct.pack('<HH',0x1000+(len(data)+4)//2-1,0x56d0)+data
            struct.pack_into('<I',expected,4,len(expected)//2)
            assert source['header_hex']==expected.hex()
            if type_==13:expected[2]|=0x40
            if type_==62:
                struct.pack_into('<9d',expected,160,*[v for line in block_matrices[id_][:3] for v in line[:3]])
                assert source['prepared_header_hex']==expected.hex()
            assert source['loaded_header_hex']==expected.hex()
        expected=dict(local_dependents=[],file_dependents=[[[],[]],[[]]],pending_entities=[])
        ids=[77,42,41]+list(blocks)+[s['id'] for s in row['path_owners']]+[78]
        assert len(row['calls'])==len(ids)
        for index,id_ in enumerate(ids):
            local[id_]=entity(7,id_,62 if id_ in blocks else 47 if id_ in programs else 13 if id_==42 else 33,0,index)
            expected['local_dependents'].append([])
            links=programs[id_]['links'] if id_ in programs else [row['payload_hex']] if id_ in (77,78) else []
            for link in links:
                for target_id,target in references(bytes.fromhex(link)):
                    if target is None:
                        if target_id and index not in expected['pending_entities']:expected['pending_entities'].append(index)
                    elif not target['flags']&0x20008:
                        lists=expected['local_dependents'] if target['model']==7 else expected['file_dependents'][0 if target['model']==9 else 1]
                        lists[target['index']].insert(0,index)
            local[id_]['flags']=row['owner_flags'] if id_==42 else blocks[id_]['runtime_flags'] if id_ in blocks else programs[id_]['runtime_flags'] if id_ in programs else 0
            assert expected==row['calls'][index],(supplied,index,expected,row['calls'][index])
        def path_matrix(collector,terminal):
            chain={0:identity4,1:affines[42],2:multiply(affines[42],affines[43])}.get(collector['context'])
            accumulated=identity4
            collected=collector['collected']
            start=terminal-1 if terminal<len(collected)-1 else terminal
            # A negative start queries the standard object's handler, which
            # is absent in this constructed profile. Nonnegative start scans
            # blocks, including the terminal when it is the final entry.
            for index in range(start,-1,-1):
                if index>=len(collected):break
                node=collected[index]
                if node['model_id']==7 and node['id'] in blocks:
                    accumulated=multiply(block_matrices[node['id']],accumulated)
            return multiply(chain,accumulated) if chain is not None else None
        for spec,observed in zip(row.get('block_sources',[])+row['path_owners'],row['path_queries'],strict=True):
            collector=dict(context=None,collected=[])
            success=expand(local[spec['id']],0,collector)
            transform_code=(0 if collector['context'] is not None else 0x11006) if success else None
            matrix=path_matrix(collector,len(collector['collected'])-1)
            wanted=dict(id=spec['id'],native_code=0 if success else 1,owner_context=names[collector['context']],
                        collected=collector['collected'],terminal_index=len(collector['collected'])-1,
                        transform_code=transform_code)
            assert all(observed[k]==v for k,v in wanted.items()),(supplied,observed,wanted)
            if blocks:assert observed['owner_kind']==(None if collector['context'] is None else 1 if collector['context']==0 else 2)
            assert equal_matrix(observed['native_matrix'],flat(matrix)) if transform_code==0 else observed['native_matrix'] is None
            if 'terminal_positions' in row and success:
                for position,query in zip(row['terminal_positions'],observed['terminal_queries'],strict=True):
                    terminal=len(collector['collected'])-1 if position==-1 else position
                    assert query['requested_position']==position and query['terminal_index']==terminal
                    assert query['transform_code']==transform_code
                    matrix=path_matrix(collector,terminal)
                    assert equal_matrix(query['native_matrix'],flat(matrix)) if transform_code==0 else query['native_matrix'] is None
                    totals['terminal_queries']+=1
            totals['path_queries']+=1;totals['successful_expansions']+=success
            totals['transform_failures']+=transform_code==0x11006
            if transform_code==0 and len(collector['collected'])==1:
                terminal=collector['collected'][0]
                if terminal['model_id']==7 and terminal['id'] in programs:
                    totals['single_type47_transforms']+=1
                    totals['single_type47_reference_transforms']+=collector['context'] in (1,2)
        if 'source_transforms' in row:
            queries=row['affine_queries']
            assert [(q['reference_index'],q['stop_reference_index']) for q in queries]==[(0,None),(1,None),(2,None),(2,0),(2,2),(0,2)]
            matrices=[affines[42],affines[43],multiply(affines[42],affines[43]),affines[43],identity4,affines[42]]
            assert all(equal_matrix(q['native_matrix'],flat(m)) for q,m in zip(queries,matrices,strict=True))
            totals['affine_queries']+=len(queries)
        totals['cases']+=1;totals['root_callbacks']+=len(ids)
        totals['local_edges']+=sum(map(len,expected['local_dependents']))
        totals['other_model_edges']+=sum(len(ids) for group in expected['file_dependents'] for ids in group)
        totals['pending_entities']+=len(expected['pending_entities'])
    assert totals['cases']==expected_count
    return totals


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();report=json.loads(args.report.read_text(encoding='utf8'))
    totals=verify(report);caught=[]
    for kind in ('owner','terminal','transform','edge','source','matrix','terminal47_matrix'):
        changed=copy.deepcopy(report)
        row=changed['cases'][0]
        if kind=='owner':row['path_queries'][0]['owner_context']='reference_42'
        elif kind=='terminal':row['path_queries'][0]['terminal_index']=0
        elif kind=='transform':row['path_queries'][0]['transform_code']=1
        elif kind=='edge':next(ids for r in changed['cases'] for g in r['calls'][-1]['file_dependents'] for ids in g if ids).pop()
        elif kind=='source':next(s for s in row['source_headers'] if s['type']==47)['header_hex']=''
        elif kind=='matrix':next(q for r in changed['cases'] if 'source_transforms' in r for q in r['path_queries'] if q['owner_context']=='nested_reference_43' and q['transform_code']==0)['native_matrix'][3]+=1
        else:next(r for r in changed['cases'] if 'source_transforms' in r and r['path_program']=='single_type47_nested_reference')['path_queries'][0]['native_matrix'][3]+=1
        try:verify(changed)
        except AssertionError:caught.append(kind)
        else:raise AssertionError('Negative control not detected: '+kind)
    totals['negative_controls_detected']=caught
    args.output.write_text(json.dumps(totals,indent=2)+'\n',encoding='utf8');print(totals)


if __name__=='__main__':main()
