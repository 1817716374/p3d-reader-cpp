"""Original full-file dependency observation and notification-only outer flush.

Requires the audited input_context and DLL hashes from probe_main_entities.
Model notifications are the only permitted work queue; active entity queues
require further analysis. Reverse lists retain order and duplicate identities.
No callback replacement or binary patch; objects stay alive until process exit.
"""
import ctypes as C


def observe_dependencies(base,file,result):
 V=C.c_void_p
 ptr=lambda a:V.from_address(a).value
 u32=lambda a:C.c_uint32.from_address(a).value
 u64=lambda a:C.c_uint64.from_address(a).value
 fn=lambda r,t,*args:C.CFUNCTYPE(t,*args)(base+r)
 entities=[];keys=[]
 def visit(entity,mid):
  assert entity not in entities
  keys.append(dict(model=mid,occurrence=sum(k['model']==mid for k in keys)))
  entities.append(entity)
  children=ptr(entity+0x38)
  if children:
   for p in range(ptr(children) or 0,ptr(children+8) or 0,8):visit(ptr(p),mid)
 for row in [result['system_initial']]+result['models']:
  mid=row['model_id'];model=file+0x6c8 if mid==0xffffffff else fn(0x1263f0,V,V,C.c_int)(file,mid)
  start=len(entities)
  for listing in (ptr(model+0x138),model+0x140):
   block=ptr(listing+0x18)
   while block:
    for p in range(ptr(block+0x18) or 0,ptr(block+0x20) or 0,8):visit(ptr(p),mid)
    block=ptr(block+8)
  assert len(entities)-start==len(row['entities'])
 index={p:i for i,p in enumerate(entities)}
 def identity(p):
  assert p in index,hex(p)
  return index[p]
 def members(address):
  sentinel=ptr(address);out=[];seen=set()
  def walk(node):
   if node==sentinel:return
   assert node and node not in seen and len(seen)<1000000;seen.add(node)
   walk(ptr(node));out.append(node);walk(ptr(node+0x10))
  walk(ptr(sentinel+8));assert len(out)==u64(address+8)
  return out
 host=C.CFUNCTYPE(V,V)(ptr(base+0x51e788))(base+0x643db8)
 service=ptr(fn(0x1632e0,V,V,V)(host+8,base+0x643e18));assert ptr(service)==base+0x535d30
 registry=ptr(service+0x10)
 reverse=[]
 for p in entities:
  link=fn(0x19d5b0,V,V)(p);row=[];seen=set()
  while link:
   assert link not in seen;seen.add(link)
   row.append(identity(ptr(link+8)));link=ptr(link)
  reverse.append(row)
 transaction=fn(0x199d90,V)()
 pairs=[]
 for n in members(registry+0x90):
  target=identity(ptr(n+0x20))
  for child in members(n+0x28):pairs.append([target,identity(ptr(child+0x20))])
 return dict(entities=keys,dependents=reverse,
   pending=[identity(ptr(n+0x20)) for n in members(registry+0x30)],
   monitored=[identity(ptr(n+0x20)) for n in members(service+0x40)],scheduled_pairs=pairs,
   registry_counts=[u64(registry+o+8) for o in range(0x10,0x150,0x10)],
   notified_models=sorted(u32(ptr(n+0x20)+0x1b8) for n in members(registry+0x70)),
   registry_work_count=u32(registry),callback_depth=u32(service+0x24),
   transaction_count=u32(transaction+12),transaction_status=u32(transaction+8))


def observe_and_flush(base,file,result):
 directory=result['models']
 before=observe_dependencies(base,file,result)
 assert all(n==0 for i,n in enumerate(before['registry_counts']) if i!=6)
 assert before['registry_counts'][6]==len(directory)+1
 assert before['notified_models']==sorted([0xffffffff]+[m['model_id'] for m in directory])
 assert before['registry_work_count']==len(directory)+1 and before['callback_depth']==0
 V=C.c_void_p;ptr=lambda a:V.from_address(a).value
 host=C.CFUNCTYPE(V,V)(ptr(base+0x51e788))(base+0x643db8)
 slot=C.CFUNCTYPE(V,V,V)(base+0x1632e0)(host+8,base+0x643e18)
 service=ptr(slot);config=ptr(host+0xc8)
 assert ptr(config)==base+0x52edf8 and ptr(config+0x250)==base+0x62ad80
 assert ptr(ptr(host+0xc0))==base+0x535cf0
 assert C.c_uint32.from_address(service+0x50).value==0
 assert C.c_uint32.from_address(service+0x28).value==0
 assert C.c_uint8.from_address(service+0x60).value==0
 assert ptr(config+0x1e0)==ptr(config+0x1e8)
 rc=C.CFUNCTYPE(C.c_int,V)(base+0x1f1a90)(service)
 assert rc==0,rc
 after=observe_dependencies(base,file,result)
 assert all(n==0 for n in after['registry_counts']) and after['registry_work_count']==0
 assert before['dependents']==after['dependents']
 assert ptr(config+0x250)==base+0x62ad80 and ptr(config+0x1e0)==ptr(config+0x1e8)
 assert C.c_uint8.from_address(service+0x60).value==0
 from native_main_entity_input import capture
 for row in [result['system_initial']]+result['models']:
  mid=row['model_id'];model=file+0x6c8 if mid==0xffffffff else C.CFUNCTYPE(V,V,C.c_int)(base+0x1263f0)(file,mid)
  actual=capture(base,model)
  assert actual=={k:v for k,v in row.items() if k not in ('return_code','counter_before','counter_after','initial_bounds')}
 result['dependencies']=dict(before=before,after=after,flush_return=rc,
   entity_snapshots_preserved=True,configuration_restored=True)
 return result

