"""R1.18 system-first entity input in an inspected console service context.

The caller verifies DLL/source hashes and installs the existing input_context.
Attribute/resource/message objects use original factories; font outer fields
reproduce the inline constructor and original functions initialize its tables.
Original typed-handler registration is required before this module is called.
No callback stub, binary patch, writer or final file/model destructor is used.
Native allocations remain alive until process exit.
"""
import ctypes as C


def install_services(base):
    V=C.c_void_p
    ptr=lambda a:V.from_address(a).value
    fn=lambda r,result,*args:C.CFUNCTYPE(result,*args)(base+r)
    host=C.CFUNCTYPE(V,V)(ptr(base+0x51e788))(base+0x643db8)
    for offset,factory,vt in [(0x88,0x360dc0,0x544390),(0x60,0x164140,0x5306d0),
                               (0x80,0x164170,0x530688)]:
        service=fn(factory,V)()
        assert ptr(service)==base+vt
        V.from_address(host+offset).value=service
    # Full outer-field initialization inlined by 1636bc..163777. This is an
    # inspected shell, not an invocation of the full application constructor.
    fonts=fn(0x4ef808,V,C.c_size_t)(0xb0)
    C.memset(fonts,0,0xb0)
    V.from_address(fonts).value=base+0x52a828
    for offset in (0x28,0x60,0x80,0xa0):C.c_uint64.from_address(fonts+offset).value=7
    C.c_uint8.from_address(fonts+0xa).value=1
    C.c_uint16.from_address(fonts+0xa9).value=0x100
    V.from_address(host+0xe0).value=fonts
    fn(0xa59a0,None,V)(fonts)


def capture(base,model):
    V=C.c_void_p
    ptr=lambda a:V.from_address(a).value
    u32=lambda a:C.c_uint32.from_address(a).value
    u64=lambda a:C.c_uint64.from_address(a).value
    u16=lambda a:C.c_uint16.from_address(a).value
    byte=lambda a:C.c_uint8.from_address(a).value
    def vector(address):
        begin,end=ptr(address) or 0,ptr(address+8) or 0
        assert end>=begin and (end-begin)%8==0 and end-begin<80000000
        return [ptr(p) for p in range(begin,end,8)]
    pointers=[]; indices={}; entities=[]
    def entity(p):
        assert p and p not in indices
        i=len(pointers);indices[p]=i;pointers.append(p)
        header=ptr(p+0x40);size=u32(header+4)*2
        assert 32<=size<=0x20000
        attributes=ptr(p+0x28);attrs=[]
        if attributes:
            for a in vector(attributes+8):
                n=u32(a+8);assert n<100000000
                attrs.append(dict(key=u32(a),index=u32(a+4),flag=byte(a+12),
                    ordinal=u16(a+14),payload_hex=C.string_at(ptr(a+16),n).hex()))
        row=dict(id=u64(header+16),loaded_header=C.string_at(header,size).hex(),
            flags=u32(p+0x10),ordinal=u32(p+0x58),parent_address=ptr(p+0x20),
            children=[],attributes=attrs,attribute_flags=byte(attributes) if attributes else None)
        entities.append(row)
        if ptr(p+0x38):
            for child in vector(ptr(p+0x38)):row['children'].append(entity(child))
        return i
    lists=[]
    for name,address in [('control',ptr(model+0x138)),('graphics',model+0x140)]:
        assert address
        blocks=[];seen=set();block=ptr(address+0x18)
        while block:
            assert block not in seen and len(seen)<100000;seen.add(block)
            assert ptr(block)==address
            roots=[entity(p) for p in vector(block+0x18)]
            blocks.append(dict(number=u32(block+0x38),roots=roots))
            block=ptr(block+8)
        lists.append(dict(kind=name,loaded=bool(byte(address+0x14)),blocks=blocks))
    for row in entities:
        parent=row.pop('parent_address')
        assert not parent or parent in indices
        row['parent']=indices[parent] if parent else None
    registry=[];table=ptr(model+0x698)
    for id_ in sorted({row['id'] for row in entities}):
        present=table and C.CFUNCTYPE(C.c_bool,V,C.c_uint64)(ptr(ptr(table)+0x10))(table,id_)
        if not present:continue
        slot=C.CFUNCTYPE(V,V,C.c_uint64,V,V)(ptr(ptr(table)+0x28))(table,id_,None,None)
        assert slot and ptr(slot) in indices
        registry.append(dict(id=id_,entity=indices[ptr(slot)]))
    return dict(model_id=u32(model+0x1b8),lists=lists,entities=entities,registry_lookups=registry,
        readonly=bool(byte(model+0x7a0)),unassigned_byte_at_7a2=byte(model+0x7a2),
        version=[u32(model+0x7e0),u32(model+0x7e4)],model_reference_count=u32(model+0x1c0))


def acquire(base,file,directory):
    V,I=C.c_void_p,C.c_int
    ptr=lambda a:V.from_address(a).value
    u32=lambda a:C.c_uint32.from_address(a).value
    u64=lambda a:C.c_uint64.from_address(a).value
    fn=lambda r,result,*args:C.CFUNCTYPE(result,*args)(base+r)
    install_services(base)
    assert C.c_uint8.from_address(ptr(file+0x800)+0x14).value==0
    system_rc=fn(0x129bb0,I,V)(file)
    assert system_rc==0,hex(system_rc)
    system_initial=capture(base,file+0x6c8)
    system_counter=u64(file+0x190)
    print('system entities',len(system_initial['entities']),flush=True)
    rows=[]
    for entry in directory:
        mid=entry['model_id'];model=fn(0x1263f0,V,V,I)(file,mid)
        assert fn(0x199330,I,V)(model)==1
        counter=u64(file+0x190)
        rc=fn(0x129d50,I,V,V,I,V)(file,model,6,None)
        assert rc==0,hex(rc)
        row=capture(base,model)
        row.update(return_code=rc,counter_before=counter,counter_after=u64(file+0x190))
        rows.append(row)
        print('model',mid,'entities',len(row['entities']),flush=True)
    return dict(models=rows,system=capture(base,file+0x6c8),system_initial=system_initial,system_return=system_rc,system_counter=system_counter,
        active_model_count=u64(file+0x6b0),held_model_count=u32(file+0x684),
        file_reference_count=u32(file+0x680),final_counter=u64(file+0x190))
