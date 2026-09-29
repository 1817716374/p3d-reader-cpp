"""Bounded TLS context for original R1.18 file-input callback experiments.

Call only after the caller verifies the original DLL hashes. The host/config
and service shell contain inspected fields, not a full application host.
Original constructors create all internal registries. No native destructor
is applied to a Python buffer; allocations live until process exit.
"""
import ctypes as C
from contextlib import contextmanager


@contextmanager
def input_context(base):
    host=C.create_string_buffer(0x110)
    config=C.create_string_buffer(0x150)
    service=C.create_string_buffer(0x68)
    get=C.CFUNCTYPE(C.c_void_p,C.c_void_p)(C.c_void_p.from_address(base+0x51e788).value)
    set_=C.CFUNCTYPE(None,C.c_void_p,C.c_void_p)(C.c_void_p.from_address(base+0x51e5b0).value)
    slot=C.CFUNCTYPE(C.c_void_p,C.c_void_p,C.c_void_p)(base+0x1632e0)
    malloc=C.CFUNCTYPE(C.c_void_p,C.c_size_t)(C.c_void_p.from_address(base+0x51eb80).value)
    key=base+0x643db8
    previous=get(key)
    try:
        C.c_void_p.from_buffer(host,0xc8).value=C.addressof(config)
        C.c_uint32.from_buffer(config,0x148).value=1000
        set_(key,C.addressof(host))
        assert get(key)==C.addressof(host)
        C.c_void_p.from_buffer(service).value=base+0x535d30
        C.c_uint16.from_buffer(service,8).value=0x100
        C.c_uint32.from_buffer(service,0x5c).value=0xc8
        sentinel=malloc(0x28)
        for offset in (0,8,16):C.c_void_p.from_address(sentinel+offset).value=sentinel
        C.c_uint16.from_address(sentinel+0x18).value=0x101
        C.c_void_p.from_buffer(service,0x40).value=sentinel
        for rva in (0x1f1c80,0x1f1e60):
            C.CFUNCTYPE(None,C.c_void_p)(base+rva)(service)
        address=slot(C.addressof(host)+8,base+0x643e18)
        C.c_void_p.from_address(address).value=C.addressof(service)
        cast=C.CFUNCTYPE(C.c_void_p,C.c_void_p,C.c_int,C.c_void_p,C.c_void_p,C.c_int)(base+0x4f05de)
        assert cast(service,0,base+0x631650,base+0x62f418,0)==C.addressof(service)
        registry=C.c_void_p.from_buffer(service,0x10).value
        assert registry and C.c_uint32.from_address(registry+0x150).value==25
        assert all(C.c_void_p.from_buffer(service,o).value for o in (0x18,0x38))
        def snapshot():
            transaction=C.CFUNCTYPE(C.c_void_p)(base+0x199d90)()
            return dict(callback_depth=C.c_uint32.from_buffer(service,0x24).value,
                        transaction_count=C.c_uint32.from_address(transaction+0xc).value,
                        transaction_status=C.c_uint32.from_address(transaction+8).value,
                        registry_set_counts=[C.c_uint64.from_address(registry+o+8).value
                                             for o in range(0x10,0x130,0x10)])
        yield snapshot
    finally:
        set_(key,previous)
        assert get(key)==previous
