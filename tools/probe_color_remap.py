#!/usr/bin/env python3
"""Probe original R1.18 1e3380 on resident system-context color caches.

Original vtable 535e10 supplies self at +28 and null model at +58. No host,
file construction, GUI, callbacks or disk writes from the DLL are performed.
Bounded synthetic extended-cache buffers never grow; their native XML/tree/
string allocations are reclaimed at process exit, not by synthetic destructors.
"""
import argparse
import ctypes as C
import hashlib
import json
import os
from pathlib import Path

HASHES = {
    'ROOT/P3DKJ.dll': '37fc96b97d230ba31619af4fba09ee11b2a38bfffca616621f11b240079f5901',
    'ROOT/P3DKJJC.dll': 'd5735b0f6c706beafb5feb2a21933404c70eb0371d8d208bf48867a4c43a313c',
    'ROOT/P3DDC.dll': '1d1e3bfa704aaf9d3124815814fa7ac0a5e6fb1044b7c2c453d837347a114e03',
    'SHARE/p3dlibxml2.dll': 'ca24cc124168dd90958f79e72d2af9dbca1dd937338ce6dd07fb0500cf21270a',
}


def probe(root, corpus_report=None):
    if os.name != 'nt' or C.sizeof(C.c_void_p) != 8: raise RuntimeError('Requires Windows x64')
    for name, expected in HASHES.items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Unsupported DLL: {name}')
    directories = [os.add_dll_directory(str(root/d)) for d in ['ROOT','SHARE','SHARE/vcredist/X64','PLATFORM']]
    try:
        dll = C.WinDLL(str(root/'ROOT/P3DKJ.dll'))
        fn = C.CFUNCTYPE(C.c_uint32,C.c_uint32,C.c_void_p,C.c_void_p,C.c_void_p,C.c_bool)(dll._handle+0x1e3380)
        read = C.CFUNCTYPE(C.c_int,C.c_void_p,C.c_void_p,C.c_uint64)(dll._handle+0x2665b0)
        free = C.CFUNCTYPE(None,C.c_void_p)(C.c_void_p.from_address(dll._handle+0x521f78).value)
        def string(p):
            size, cap = C.c_uint64.from_address(p+16).value, C.c_uint64.from_address(p+24).value
            assert size < 100000
            return C.string_at(C.c_void_p.from_address(p).value if cap>=8 else p, size*2).decode('utf-16le')
        class Context:
            def __init__(self, palette, xml):
                self.file, self.palette, self.cache, self.slots, self.head = (C.create_string_buffer(n) for n in [0x1000,0x440,64,16*256,40])
                self.address = C.addressof(self.file)+0x6c8
                C.c_void_p.from_buffer(self.file,0x6c8).value = dll._handle+0x535e10
                C.c_void_p.from_buffer(self.file,0x768).value = C.addressof(self.file)
                C.c_void_p.from_buffer(self.file,0xf20).value = C.addressof(self.palette)
                C.c_void_p.from_buffer(self.file,0xf28).value = C.addressof(self.cache)
                h = C.addressof(self.head)
                for o in [0,8,16]: C.c_void_p.from_buffer(self.head,o).value=h
                C.c_uint16.from_buffer(self.head,24).value=0x101
                for o,v in [(8,C.addressof(self.slots)),(16,C.addressof(self.slots)),(24,C.addressof(self.slots)+len(self.slots)),(32,h)]:
                    C.c_void_p.from_buffer(self.cache,o).value=v
                self.set_palette(palette)
                text = C.create_unicode_buffer(xml)
                assert read(self.cache,text,len(xml.encode('utf-16le')))==0
            def set_palette(self, values):
                C.memmove(C.addressof(self.palette)+16,bytes(v for c in values for v in c),1024)
            def snapshot(self):
                n=(C.c_void_p.from_buffer(self.cache,16).value-C.addressof(self.slots))//16
                assert 0<=n<=256
                entries=[]
                for i in range(n):
                    pair=C.c_void_p.from_buffer(self.slots,16*i+8).value
                    entries.append({'rgb':list(self.slots.raw[16*i:16*i+3]),'book_name':
                        {'book':string(pair),'name':string(pair+32)} if pair else None})
                return {'entries':entries,'dirty':bool(self.cache.raw[56])}
        gray=[[i,i,i,0] for i in range(256)]
        reverse=[[255-i]*3+[0] for i in range(256)]
        wheel=[[(i*47)%256,(i*73)%256,(i*137)%256,0] for i in range(256)]
        alpha=[c[:] for c in gray]; alpha[50]=alpha[0][:]; alpha[50][3]=99
        duplicate=[c[:] for c in gray]; duplicate[50]=duplicate[0][:]
        last=[c[:] for c in gray];last[255]=[255,0,0,7]
        palettes={'gray':gray,'reverse':reverse,'wheel':wheel,'alpha':alpha,'duplicate':duplicate,'last':last}
        xml='<Colors><Entry Color="(10,20,30)" Book="Book" Name="Name"/><Entry Color="(10,20,30)"/><Entry Color="(255,0,0)"/></Colors>'
        target_xml='<Colors><Entry Color="(10,20,30)"/><Entry Color="(10,20,30)"/></Colors>'
        ids=list(range(256))+[0x100,0x101,0x1ff,0x200,0x2ff,0x300,0x301,0x400,0xf0000101,0x10000001,0xfffffeff,0xffffff00,0xfffffffe,0xffffffff,0x7fffffff]
        scenarios=[]
        specs=[('cross', 'gray','reverse',True,False,False),('same','gray','gray',True,False,True),
               ('no_mapping','gray','reverse',False,False,False),('bypass','gray','reverse',True,True,False),
               ('alpha','alpha','duplicate',True,False,False),('slot255','gray','last',True,False,False),
               ('empty_target','wheel','gray',True,False,False)]
        for label,sp,tp,enabled,bypass,same in specs:
            contexts=[Context(palettes[sp],xml)]
            if not same: contexts.append(Context(palettes[tp],'<Colors/>' if label=='empty_target' else target_xml))
            initial=[{'palette':sp if i==0 else tp,**c.snapshot()} for i,c in enumerate(contexts)]
            cache=C.c_void_p();rows=[]
            for color_id in ids:
                target=0 if same else 1
                result=fn(color_id,C.byref(cache) if enabled else None,contexts[target].address,contexts[0].address,bypass)
                state=None if not cache.value else {'source':0,'target':target,
                    'indices':list(C.string_at(cache.value+16,256)),
                    'palettes_differ':bool(C.c_ubyte.from_address(cache.value+0xd10).value)}
                rows.append({'id':color_id,'source':0,'target':target,'mapping_enabled':enabled,'bypass':bypass,
                             'color_id':result,'mapping':state,'target_state':contexts[target].snapshot()})
            if cache.value:free(cache)
            scenarios.append({'name':label,'contexts':initial,'queries':rows})
        # One persistent cache: in-place palette changes, source identity changes,
        # target identity changes, and revisiting an already-computed entry.
        contexts=[Context(gray,xml),Context(reverse,target_xml),Context(wheel,xml)]
        initial=[{'palette':p,**c.snapshot()} for p,c in zip(['gray','reverse','wheel'],contexts)]
        cache=C.c_void_p();rows=[]
        operations=[(0,1,30,None),(0,1,30,(1,'wheel')),(0,1,31,None),
                    (0,1,32,(0,'wheel')),(2,1,33,None),(0,1,34,None),
                    (0,2,35,None),(0,1,36,None),(0,1,30,None)]
        for s,t,color_id,mutation in operations:
            if mutation:contexts[mutation[0]].set_palette(palettes[mutation[1]])
            result=fn(color_id,C.byref(cache),contexts[t].address,contexts[s].address,False)
            state={'source':s,'target':t,'indices':list(C.string_at(cache.value+16,256)),
                   'palettes_differ':bool(C.c_ubyte.from_address(cache.value+0xd10).value)}
            rows.append({'id':color_id,'source':s,'target':t,'mapping_enabled':True,'bypass':False,
                         'mutation':mutation,'color_id':result,'mapping':state,'target_state':contexts[t].snapshot()})
        free(cache)
        scenarios.append({'name':'identity_and_palette_mutation','contexts':initial,'queries':rows})
        apply = C.CFUNCTYPE(None,C.c_void_p,C.c_void_p,C.c_void_p,C.c_void_p)(dll._handle+0xdaaf0)
        backgrounds=[]
        for same in [False,True]:
            contexts=[Context(gray,xml),Context(reverse,target_xml)]
            initial=[{'palette':p,**c.snapshot()} for p,c in zip(['gray','reverse'],contexts)]
            rows=[]
            for enabled in [False,True]:
                for color_id in ids:
                    t=0 if same else 1
                    style=C.create_string_buffer(0x90);view=C.create_string_buffer(0x190)
                    C.c_void_p.from_buffer(style,0x40).value=C.addressof(contexts[0].file)
                    C.c_uint32.from_buffer(style,0x48).value=0x2000 if enabled else 0
                    C.c_uint32.from_buffer(style,0x80).value=color_id
                    C.memmove(C.addressof(view)+0xec,bytes([17,29,43]),3)
                    before=view.raw
                    apply(style,None,view,contexts[t].file)
                    assert view.raw[:0xec]==before[:0xec] and view.raw[0xef:]==before[0xef:]
                    rows.append({'id':color_id,'source':0,'target':t,'enabled':enabled,
                                 'initial_rgb':[17,29,43],'rgb':list(view.raw[0xec:0xef]),
                                 'target_state':contexts[t].snapshot()})
            backgrounds.append({'contexts':initial,'queries':rows})
        if corpus_report:
            report=json.loads(corpus_report.read_text(encoding='utf8'))
            for file in report['files']:
                if file['execution_status']!='completed':raise ValueError('Incomplete corpus audit')
                doc=json.loads((Path(file['output'])/'document.json').read_text(encoding='utf8'))
                palette=doc['initial_color_palette']
                if palette['status']!='resolved':raise ValueError('Explicitly resolved initial palette required')
                tables=doc['initial_extended_color_tables']
                if len(tables)!=1 or tables[0]['outcome']!='selected_xml_input':raise ValueError('Unique initial color input required')
                selected=tables[0]['selected_attribute']
                attrs=[a for g in doc['graphics_records'] if g['stream']==selected['stream']
                       for a in g['attributes'] if a['offset']==selected['attribute_offset']+16]
                if len(attrs)!=1:raise ValueError('Ambiguous color source')
                name='corpus_'+file['sha256']
                palettes[name]=[c+[0] for c in palette['rgb']]
                context=Context(palettes[name],attrs[0]['decoded']['xml'])
                initial=[{'palette':name,**context.snapshot()}];rows=[]
                for table in doc['initial_display_style_tables']:
                    for ref in table['reference_inputs']['references']:
                        entry=ref['lite_slot_query']['entry']
                        imported=entry.get('projected_xml_import',entry.get('native_xml_import'))
                        fields=imported['fields']
                        color_id=fields['Overrides.BackgroundColor']['value'] & 0xffffffff
                        enabled=fields['Flags.BackgroundColor']['value']
                        style=C.create_string_buffer(0x90);view=C.create_string_buffer(0x190)
                        C.c_void_p.from_buffer(style,0x40).value=C.addressof(context.file)
                        C.c_uint32.from_buffer(style,0x48).value=0x2000 if enabled else 0
                        C.c_uint32.from_buffer(style,0x80).value=color_id
                        C.memmove(C.addressof(view)+0xec,bytes([17,29,43]),3)
                        apply(style,None,view,context.file)
                        rows.append({'id':color_id,'source':0,'target':0,'enabled':enabled,
                                     'input_occurrence_index':ref['input_occurrence_index'],
                                     'initial_rgb':[17,29,43],'rgb':list(view.raw[0xec:0xef]),
                                     'target_state':context.snapshot()})
                backgrounds.append({'corpus_sha256':file['sha256'],'contexts':initial,'queries':rows})
        return {'scope':'resident_system_contexts_original_vtable','palettes':palettes,
                'scenarios':scenarios,'backgrounds':backgrounds}
    finally:
        for directory in directories:directory.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',type=Path)
    p.add_argument('--corpus-report',type=Path,help='Audit produced with palette/style default-service flags')
    a=p.parse_args()
    result=probe(a.dll_root.resolve(),a.corpus_report)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps({'dll_sha256':HASHES,**result},indent=2),encoding='utf8')
    if a.fixture:
        body=json.dumps(result,separators=(',',':'))
        parts=['R"P3D('+body[i:i+8192]+')P3D"' for i in range(0,len(body),8192)]
        a.fixture.write_text('#pragma once\n// Original R1.18 system-context color remapping.\ninline constexpr const char *color_remap_oracle_parts[] = {\n'+',\n'.join(parts)+'\n};\n',encoding='utf8')
    print('Recorded',sum(len(s['queries']) for s in result['scenarios']),'native system color remaps')
    print('Recorded',sum(len(s['queries']) for s in result['backgrounds']),'native background applications')

if __name__=='__main__':main()
