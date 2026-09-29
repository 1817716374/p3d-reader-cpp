#!/usr/bin/env python3
"""Full original recursive bounds and default tables with inline clip polygons.
Reuses SHA-gated original-only traversal; no clip-object lookup, host or callbacks.
"""
import argparse,copy,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_reference_bounds import probe

def graphs():
    identity=[1,0,0,0,1,0,0,0,1]
    matrices=[identity,[0,-1,0,1,0,0,0,0,1],[-1,0,0,0,1,0,0,0,1],
              [1,.2,.1,.3,1,.4,.5,.6,1],[1,0,0,0,.8,-.6,0,.6,.8]]
    maximum=float.fromhex('0x1.fffffffffffffp+1023');separator=[maximum,maximum]
    rectangle=[[-5,-6],[5,-6],[5,6],[-5,6]]
    polygons=[rectangle,rectangle+[rectangle[0]],list(reversed(rectangle)),
              [[-8,-3],[4,-7],[9,6]], [[-8,-8],[8,-8],[8,-2],[-2,-2],[-2,8],[-8,8]],
              [[20,20],[30,20],[30,30],[20,30]],[[10,-5],[20,-5],[20,5],[10,5]],
              [[0,-5],[0,0],[0,5],[0,10]],
              rectangle+[separator]+[[-2,-2],[2,-2],[2,2],[-2,2]],
              [separator]+rectangle,[[1,1],[2,2],separator]+rectangle,
              rectangle+[separator,[1,1],[2,2]],[],[[1,1]],[[1,1],[2,2]],[[1,1],[2,2],[3,3]]]
    def node(points,matrix=identity,flags=0xc00,allowed=True,lower=-7,upper=8,**kw):
        n={'bounds':[-10,-20,-30,10,20,30],'spatial':True,'target_present':True,
           'attached':False,'origin':[1,2,3],'translation':[0,0,0],'reference_point':[0,0,0],
           'scale':1.,'matrix':identity,'perspective':False,'eye':[0,0,100],'distance':25.,'children':[],
           'clip':{'points':points,'matrix':matrix,'depth_flags':flags,'depths_allowed':allowed,
                   'lower':lower,'upper':upper}}
        n.update(kw);return n
    out=[]
    def add(nodes,roots=[0],root=None):out.append(copy.deepcopy({'nodes':nodes,'roots':roots,'root_bounds':root}))
    for p in polygons:
        for m in matrices:
            for flags in [0,0x400,0x800,0xc00]:
                for allowed in [False,True]:add([node(p,m,flags,allowed)])
    for low,high in [(-7,8),(0,0),(8,-7),(40,50),(-50,-40),(-30,30),(-31,-30),(30,31)]:
        for scale in [.5,1,-2]:
            for attached in [False,True]:
                for perspective in [False,True]:
                    add([node(rectangle,lower=low,upper=high,scale=scale,attached=attached,
                              translation=[3,4,5],reference_point=[1,2,3],perspective=perspective)])
    for i in range(16):
        parent=node(rectangle,matrix=matrices[i%5],children=[1],scale=2,translation=[10,20,30])
        child=node(rectangle,flags=(i%4)*0x400,translation=[-5,3,1],perspective=bool(i&4),attached=bool(i&8))
        add([parent,child],[0,1,0],[-1,-2,-3,1,2,3])
    plane_matrices=[identity,[2,0,0,0,3,0,0,0,4],[-2,0,0,0,3,0,0,0,-4],
                    [0,0,1,0,1,0,-1,0,0],[1,0,0,0,0,-1,0,1,0],
                    [1,0,0,0,1,0,1e-13,0,1],[1,0,0,0,1,0,2e-12,0,1],
                    [0]*9,[1,0,0,0,1,0,0,0,0],matrices[3]]
    for m in plane_matrices:
        for flags in [0,0x400,0x800,0xc00]:
            for lower,upper in [(-7,8),(0,0),(8,-7)]:
                for attached in [False,True]:
                    add([node([separator]+rectangle,m,flags,lower=lower,upper=upper,
                              translation=[3,4,5],attached=attached)])
    add([node([rectangle[i%4] for i in range(2500)])])
    add([node([separator])])
    for lower,upper in [(1e101,2e101),(-2e101,-1e101),(-1e101,1e101),(-1e100,1e100)]:
        add([node([separator]+rectangle,lower=lower,upper=upper)])
    return out

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path);a=p.parse_args()
    report=probe(a.dll_root.resolve(),graphs())
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf8')
    if a.fixture:a.fixture.write_text('#pragma once\n// Original R1.18 inline reference clipping observations.\ninline constexpr const char* reference_clipping_oracle = R"oracle('+json.dumps(report,separators=(',',':'),allow_nan=False)+')oracle";\n',encoding='utf8')
    print('Observed',len(report['cases']),'full inline clipping traversals and default tables')
if __name__=='__main__':main()
