"""Original R1.18 file/model construction for caller-held input experiments.

Requires verified DLLs and an active native_input_context. The surrounding
application host/config is still bounded; this does not open a persisted file
or run the outer host flush. All allocations survive until process exit.
"""
import ctypes as C


def constructed_model(base, initial_counter, spatial, file_wrapper=None):
    # An optional original file wrapper supplies the file specification needed
    # by type-13 input. It is retained by the original file factory below.
    ptr=lambda a:C.c_void_p.from_address(a).value
    u32=lambda a:C.c_uint32.from_address(a).value
    u64=lambda a:C.c_uint64.from_address(a).value
    def allocate(size):
        address=C.CFUNCTYPE(C.c_void_p,C.c_size_t)(base+0x4ef808)(size)
        assert address
        C.memset(address,0,size)
        return address
    host=C.CFUNCTYPE(C.c_void_p,C.c_void_p)(ptr(base+0x51e788))(base+0x643db8)
    # BPPlatform BRConsoleAppHost virtual +40 forwards to this real factory.
    service=C.CFUNCTYPE(C.c_void_p)(base+0x360e20)()
    assert ptr(service)==base+0x544230
    C.c_void_p.from_address(host+0x70).value=service
    holder=C.c_void_p()
    C.CFUNCTYPE(C.c_void_p,C.c_void_p,C.c_void_p,C.c_void_p)(base+0x360c10)(service,C.byref(holder),file_wrapper)
    file=holder.value
    assert file and ptr(file)==base+0x52eef8 and u32(file+0x680)==1
    model=allocate(0x7f8)
    assert C.CFUNCTYPE(C.c_void_p,C.c_void_p,C.c_void_p,C.c_int)(base+0x19d0f0)(model,file,7)==model
    assert ptr(model)==base+0x5333c8 and ptr(model+0xa0)==file
    assert C.CFUNCTYPE(C.c_int,C.c_void_p,C.c_void_p,C.c_bool)(base+0x1223e0)(file,model,False)==0
    # Real caller acquisition: iteration's model +a8/+b0 callbacks take/release
    # a nested hold. Final caller release/unload is outside this experiment.
    assert C.CFUNCTYPE(C.c_int,C.c_void_p)(base+0x199330)(model)==1
    input_service=allocate(0xa0)
    # This derived provider targets linked substorages when opening files. Here
    # only its inherited entity-input methods run; no disk storage is opened.
    assert C.CFUNCTYPE(C.c_void_p,C.c_void_p,C.c_void_p,C.c_void_p,C.c_int)(base+0x1cb2a0)(input_service,file,None,0)==input_service
    C.c_void_p.from_address(file+0xee0).value=input_service
    C.c_uint64.from_address(file+0x190).value=initial_counter
    C.c_uint8.from_address(model+0x78).value=spatial
    assert u32(base+0x619118)>0
    def metadata():
        assert u32(model+0x1c0)==1 and u32(file+0x684)==1 and u32(file+0x680)==1
        page=ptr(model+0x178)
        return dict(file_vtable=hex(ptr(file)-base),model_vtable=hex(ptr(model)-base),
            model_host_vtable=hex(ptr(service)-base),file_model_count=u64(file+0x6b0),
            model_reference_count=u32(model+0x1c0),file_reference_count=u32(file+0x680),
            file_held_model_count=u32(file+0x684),model_dirty_count=u32(model+0x1bc),
            input_list_vtable=hex(ptr(ptr(model+0x138))-base),graphics_list_vtable=hex(ptr(model+0x140)-base),
            storage_page_bytes=u32(page+0x10) if page else 0,storage_page_used=u32(page+0x14) if page else 0)
    return ((C.c_char*0x7f8).from_address(model),(C.c_char*0xf88).from_address(file),
            (C.c_char*0x38).from_address(model+0x140),metadata)
