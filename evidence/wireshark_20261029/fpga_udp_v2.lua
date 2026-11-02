-- Local display decoder for the project's real UDP V2 payload.
-- Each 16-bit word is big endian; sequence/time words are least-significant first.
-- This file does not generate or change packets.
local p=Proto('fpga_v2','FPGA 同步采集数据 V2')
local f={
 magic=ProtoField.uint16('fpga_v2.magic','同步字',base.HEX),
 version=ProtoField.uint8('fpga_v2.version','数据格式版本',base.DEC),
 quality=ProtoField.uint8('fpga_v2.quality','采样时的时间状态',base.DEC,{[0]='失效 INVALID',[1]='重新校时 REACQUIRE',[2]='时间正常 LOCKED',[3]='短时守时 HOLDOVER'}),
 sequence=ProtoField.uint32('fpga_v2.sequence','数据包序号',base.DEC),
 channel=ProtoField.uint16('fpga_v2.channel','采集通道',base.DEC),
 samples=ProtoField.uint16('fpga_v2.samples','本包样本数',base.DEC),
 timestamp=ProtoField.uint64('fpga_v2.timestamp','首个样本产生时间 / ns',base.DEC),
 payload=ProtoField.uint32('fpga_v2.payload_length','应用数据长度 / 字节',base.DEC),
 raw=ProtoField.bytes('fpga_v2.raw','16 位采集样本'),
 valid=ProtoField.bool('fpga_v2.length_valid','长度与样本数一致')
}
p.fields=f
function p.dissector(tvb,pinfo,tree)
 if tvb:len()<20 or tvb(0,2):uint()~=0x55aa then return 0 end
 local version=tvb(3,1):uint();local q=tvb(2,1):uint()%4
 if version~=2 then return 0 end
 local seq=tvb(4,2):uint()+65536*tvb(6,2):uint()
 local lo=tvb(12,2):uint()+65536*tvb(14,2):uint()
 local hi=tvb(16,2):uint()+65536*tvb(18,2):uint()
 local timestamp=UInt64.new(lo,hi)
 local n=tvb(10,2):uint()
 pinfo.cols.protocol='FPGA V2'
 pinfo.cols.info=string.format('采集数据 | %s | 序号 %u | 通道 %u | 样本 %u',({[0]='失效',[1]='重新校时',[2]='时间正常',[3]='短时守时'})[q],seq,tvb(8,2):uint(),n)
 local root=tree:add(p,tvb(),string.format('FPGA 同步采集数据 V2：时间正常状态按包传递，共 %d 字节',tvb:len()))
 root:add(f.magic,tvb(0,2));root:add(f.version,tvb(3,1));root:add(f.quality,tvb(2,1),q)
 root:add(f.sequence,tvb(4,4),seq);root:add(f.channel,tvb(8,2));root:add(f.samples,tvb(10,2))
 root:add(f.timestamp,tvb(12,8),timestamp)
 root:add(f.payload,tvb(),tvb:len()):set_generated()
 root:add(f.valid,tvb(),tvb:len()==20+2*n):set_generated()
 if tvb:len()>20 then root:add(f.raw,tvb(20)) end
 return tvb:len()
end
DissectorTable.get('udp.port'):add(6000,p)
