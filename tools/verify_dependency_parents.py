#!/usr/bin/env python3
"""Independently replay original entity trees, subtree callbacks and collectors."""
import argparse
import copy
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_dependency_parents import cases
from verify_dependency_type47 import verify as replay


def verify(report):
    return replay(report,cases(),'R1.18_original_parented_owner_path_callbacks',576)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();report=json.loads(args.report.read_text(encoding='utf8'))
    totals=verify(report);caught=[]
    for kind in ('parent','children','batch','ancestor','source','prepared','loaded_child_flag','edge','matrix','terminal'):
        changed=copy.deepcopy(report);row=changed['cases'][0]
        if kind=='parent':row['tree_entities'][1]['parent']=None
        elif kind=='children':row['tree_entities'][0]['children'].pop()
        elif kind=='batch':row['input_batches'][3].pop()
        elif kind=='ancestor':row['path_queries'][1]['collected'].pop(0)
        elif kind=='source':next(s for s in row['source_headers'] if s['type']==14)['header_hex']=''
        elif kind=='prepared':next(s for s in row['source_headers'] if s['type']==62)['prepared_header_hex']=''
        elif kind=='loaded_child_flag':
            source=next(s for s in row['source_headers'] if s['type']==62)
            data=bytearray.fromhex(source['loaded_header_hex']);data[2]&=0x7f
            source['loaded_header_hex']=data.hex()
        elif kind=='edge':next(ids for ids in row['calls'][-1]['local_dependents'] if ids).pop()
        elif kind=='matrix':row['path_queries'][1]['native_matrix'][3]+=1
        else:row['path_queries'][1]['terminal_queries'][1]['terminal_index']=1
        try:verify(changed)
        except AssertionError:caught.append(kind)
        else:raise AssertionError('Undetected negative: '+kind)
    totals['negative_controls_detected']=caught
    args.output.write_text(json.dumps(totals,indent=2)+'\n',encoding='utf8');print(totals)


if __name__=='__main__':main()
