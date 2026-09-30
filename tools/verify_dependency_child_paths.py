#!/usr/bin/env python3
"""Independent full graph, source, ancestry and transform replay for child paths."""
import argparse
import copy
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_dependency_child_paths import cases
from verify_dependency_type47 import verify as replay


def verify(report):
    return replay(report,cases(),'R1.18_original_parented_type47_expansion_callbacks',864)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();report=json.loads(args.report.read_text(encoding='utf8'))
    totals=verify(report);caught=[]
    for kind in ('source','loaded_flag','expanded_ancestor','terminal_ancestor','parent','owner','edge','matrix'):
        changed=copy.deepcopy(report)
        row=next(r for r in changed['cases'] if r['child_program']=='empty' and r['tree_state']=='normal' and
                 r['path_program']==dict(format=6,path=[46,45]))
        query=next(q for q in row['path_queries'] if q['id']==44)
        source=next(s for s in row['source_headers'] if s['id']==46)
        if kind=='source':source['header_hex']=''
        elif kind=='loaded_flag':
            data=bytearray.fromhex(source['loaded_header_hex']);data[2]&=0x7f
            source['loaded_header_hex']=data.hex()
        elif kind=='expanded_ancestor':next(q for q in row['path_queries'] if q['id']==46)['collected']=[dict(model_id=7,id=60)]
        elif kind=='terminal_ancestor':query['collected'].pop(0)
        elif kind=='parent':row['tree_entities'][3]['parent']=None
        elif kind=='owner':query['owner_context']='null'
        elif kind=='edge':next(v for v in row['calls'][-1]['local_dependents'] if v).pop()
        else:query['native_matrix'][3]+=1
        try:verify(changed)
        except AssertionError:caught.append(kind)
        else:raise AssertionError('Undetected negative: '+kind)
    totals['negative_controls_detected']=caught
    args.output.write_text(json.dumps(totals,indent=2)+'\n',encoding='utf8');print(totals)


if __name__=='__main__':main()
