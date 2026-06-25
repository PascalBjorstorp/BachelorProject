#!/usr/bin/env python3
from __future__ import annotations
import struct
from pathlib import Path

OP={1:'Header',2:'Footer',3:'Schema',4:'Channel',5:'Message',6:'Chunk',7:'MessageIndex',8:'ChunkIndex',9:'Attachment',10:'AttachmentIndex',11:'Statistics',12:'Metadata',13:'MetadataIndex',14:'SummaryOffset',15:'DataEnd'}

def read_str(b,off):
 n=struct.unpack_from('<I',b,off)[0]; off+=4
 return b[off:off+n].decode('utf8'),off+n

def records(data,base=0):
 o=0
 while o+9<=len(data):
  op=data[o]; n=struct.unpack_from('<Q',data,o+1)[0]; start=o+9; end=start+n
  if end>len(data): return
  yield op,data[start:end],base+o
  o=end

def main(p):
 chan={}
 schemas={}
 messages=0
 with open(p,'rb') as f: raw=f.read()
 # skip first magic and final magic
 for op,body,pos in records(raw[8:-8],8):
  if op==6:
   st,et,sz,crc=struct.unpack_from('<QQQI',body,0); off=28
   comp,off=read_str(body,off); record_len=struct.unpack_from('<Q',body,off)[0]; off += 8
   compressed=body[off:off+record_len]
   print('chunk',pos,'t',st,et,'size',sz,'comp',repr(comp), 'data',len(compressed))
   if comp: continue
   for iop, ibody, ipos in records(compressed,pos+9+off):
    if iop==3:
     sid=struct.unpack_from('<H',ibody,0)[0]; o=2
     name,o=read_str(ibody,o); enc,o=read_str(ibody,o); dl=struct.unpack_from('<I',ibody,o)[0];o+=4
     schemas[sid]=(name,enc,ibody[o:o+dl])
    elif iop==4:
     cid,sid=struct.unpack_from('<HH',ibody,0); o=4
     topic,o=read_str(ibody,o); enc,o=read_str(ibody,o)
     chan[cid]=(topic,sid,enc)
     print(' channel',cid,chan[cid], 'schema',schemas.get(sid,('?',))[0])
    elif iop==5:
     cid,seq=struct.unpack_from('<HI',ibody,0); log,pub=struct.unpack_from('<QQ',ibody,6)
     if messages<60:
      print(' message',messages,chan.get(cid), 'seq',seq,'log',log,'pub',pub,'n=',len(ibody)-22,'data',ibody[22:22+32].hex())
     messages+=1
   if messages >= 60: break
  elif op in (3,4): print('outer',OP[op],body[:100])
 print('schemas',[(k,v[:2]) for k,v in schemas.items()])
 print('channels',len(chan),'messages printed',messages)

if __name__=='__main__': main('/mnt/data/mpc_identification/raw/shortened_baseline/shortened_baseline_0.mcap')
