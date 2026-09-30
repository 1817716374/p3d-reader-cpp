#!/usr/bin/env python3
"""Independently replay prepared block sources, graph and collector matrices."""
import argparse
import copy
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_dependency_blocks import cases
from verify_dependency_type47 import verify as replay


def verify(report):
    return replay(report,cases(),'R1.18_original_type62_collector_transforms',720)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=json.loads(args.report.read_text(encoding='utf8'))
    totals=verify(report);caught=[]
    for kind in ('source','prepared','loaded','terminal','matrix','boundary','edge','owner'):
        changed=copy.deepcopy(report);row=changed['cases'][0]
        if kind in ('source','prepared','loaded'):
            source=next(s for s in row['source_headers'] if s['type']==62)
            source[{'source':'header_hex','prepared':'prepared_header_hex','loaded':'loaded_header_hex'}[kind]]=''
        elif kind=='terminal':row['path_queries'][0]['terminal_index']=1
        elif kind=='matrix':row['path_queries'][0]['native_matrix'][3]+=1
        elif kind=='boundary':row['path_queries'][0]['terminal_queries'][2]['native_matrix'][3]+=1
        elif kind=='edge':next(ids for r in changed['cases'] for ids in r['calls'][-1]['local_dependents'] if ids).pop()
        else:row['path_queries'][0]['owner_context']='null'
        try:verify(changed)
        except AssertionError:caught.append(kind)
        else:raise AssertionError('Undetected negative: '+kind)
    totals['negative_controls_detected']=caught
    args.output.write_text(json.dumps(totals,indent=2)+'\n',encoding='utf8')
    print(totals)


if __name__=='__main__':main()
