"""只读扫描ROS1 BAG，抢救完整消息到新副本并重建索引，不补造缺失数据。"""
import argparse
import bz2
from collections import Counter
from datetime import datetime, timedelta, timezone
import hashlib
import io
import json
from pathlib import Path
import struct
import time


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(8*1024*1024),b''): h.update(block)
    return h.hexdigest()


def fields(raw):
    out={}; i=0
    while i<len(raw):
        if i+4>len(raw): raise ValueError('字段长度被截断')
        n=struct.unpack_from('<I',raw,i)[0]; i+=4
        if n==0 or i+n>len(raw): raise ValueError('字段内容被截断')
        key,value=raw[i:i+n].split(b'=',1); i+=n
        if key.decode() in out: raise ValueError('重复字段')
        out[key.decode()]=value
    return out


def read_record(stream,limit):
    offset=stream.tell(); raw=stream.read(4)
    if not raw: return None
    if len(raw)!=4: raise ValueError(f'记录头长度截断，偏移{offset}')
    n=struct.unpack('<I',raw)[0]
    if n>1024*1024: raise ValueError(f'记录头长度异常，偏移{offset}')
    raw=stream.read(n)
    if len(raw)!=n: raise ValueError(f'记录头截断，偏移{offset}')
    header=fields(raw); raw=stream.read(4)
    if len(raw)!=4: raise ValueError(f'数据长度字段截断，偏移{offset}')
    length=struct.unpack('<I',raw)[0]
    if length>limit: raise ValueError(f'记录声明长度{length}超过内存保护上限{limit}，偏移{offset}')
    data=stream.read(length)
    return offset,header,data,length,len(data)==length


def fingerprint(topic,msgtype,md5,timestamp,data):
    # Per-message SHA256 sum preserves duplicate multiplicity and ignores repacking order.
    h=hashlib.sha256()
    for text in (topic,msgtype,md5):
        value=text.encode(); h.update(struct.pack('<I',len(value))); h.update(value)
    h.update(struct.pack('<QI',timestamp,len(data))); h.update(data)
    return int.from_bytes(h.digest(),'big')


def repair(source,output,max_record_mb=256):
    from rosbags.rosbag1 import Writer, Reader
    from rosbags.typesys.msg import normalize_msgtype
    import lz4.frame
    source=Path(source).resolve(); output=Path(output).resolve()
    if source==output: raise ValueError('输出路径不能是原文件')
    if not source.is_file(): raise ValueError('源文件不存在')
    temp=output.with_name(output.stem+'.partial.bag')
    report_path=output.with_suffix('.recovery.json'); md_path=output.with_suffix('.recovery.md')
    if any(p.exists() for p in (output,temp,report_path,md_path)): raise FileExistsError('输出/临时文件/报告已存在，请指定新的输出名称')
    with source.open('rb') as f:
        if f.readline()!=b'#ROSBAG V2.0\n': raise ValueError('仅支持ROS1 BAG V2.0，不支持ROS2目录或存储文件')
    output.parent.mkdir(parents=True,exist_ok=True)
    report={'source':str(source),'output':str(output),'started_beijing':datetime.now(timezone(timedelta(hours=8))).isoformat(),
            'issues':[],'source_unchanged':False,'verified':False,'status':'RUNNING'}
    size=source.stat().st_size; started=time.monotonic(); limit=max_record_mb*1024*1024
    connections={}; counts=Counter(); checksum=0; chunks=0; skipped=0; first=None; last=None
    def note(message,**positions): report['issues'].append({'message':message,**positions})
    def connection(header,data,writer):
        cid=struct.unpack('<I',header['conn'])[0]; meta=fields(data)
        typ=normalize_msgtype(meta['type'].decode()); topic=header['topic'].decode()
        info=(topic,typ,meta['md5sum'].decode(),meta['message_definition'].decode(),
              meta.get('callerid',b'').decode() or None,int(meta['latching']) if meta.get('latching') else None)
        if cid in connections:
            if connections[cid][1]!=info: raise ValueError('相同连接编号出现冲突定义')
            return
        c=writer.add_connection(info[0],info[1],msgdef=info[3],md5sum=info[2],callerid=info[4],latching=info[5])
        connections[cid]=(c,info)
    try:
        print('1/4 计算原文件SHA256……',flush=True); report['source_sha256_before']=sha(source)
        print('2/4 扫描数据块并写入完整消息……',flush=True)
        with source.open('rb') as f, Writer(temp) as writer:
            f.seek(13)
            while True:
                try: rec=read_record(f,limit)
                except Exception as exc:
                    note('停止扫描：'+str(exc),source_offset=f.tell()); break
                if rec is None: break
                offset,header,data,length,complete=rec; op=header.get('op',b'\0')[0]
                if not complete: note('记录载荷截断',source_offset=offset,declared_bytes=length,available_bytes=len(data))
                if op==7:
                    try:
                        if not complete: raise ValueError('连接定义残缺')
                        connection(header,data,writer)
                    except Exception as exc: note('连接定义不可用：'+str(exc),source_offset=offset)
                elif op==5:
                    try:
                        method=header['compression'].decode()
                        if method=='none': payload=data
                        elif method=='bz2': payload=bz2.decompress(data)
                        elif method=='lz4': payload=lz4.frame.decompress(data)
                        else: raise ValueError('不支持的压缩方式：'+method)
                        expected=struct.unpack('<I',header['size'])[0]
                        if len(payload)!=expected:
                            note('解压数据块长度与声明不一致，仅读取完整记录',source_offset=offset,declared_uncompressed_bytes=expected,available_uncompressed_bytes=len(payload))
                    except Exception as exc:
                        skipped+=1; note('跳过无法解压数据块：'+str(exc),source_offset=offset)
                        if not complete: break
                        continue
                    chunks+=1; inner=io.BytesIO(payload)
                    while True:
                        pos=inner.tell()
                        try:
                            item=read_record(inner,limit)
                            if item is None: break
                            _,h,body,_,ok=item
                            if not ok: raise ValueError('消息/连接记录不完整')
                            kind=h['op'][0]
                            if kind==7: connection(h,body,writer)
                            elif kind==2:
                                cid=struct.unpack('<I',h['conn'])[0]
                                if cid not in connections: raise ValueError(f'未找到连接{cid}的定义')
                                sec,nsec=struct.unpack('<II',h['time'])
                                if nsec>=10**9: raise ValueError('时间戳纳秒字段非法')
                                timestamp=sec*10**9+nsec; c,info=connections[cid]
                                writer.write(c,timestamp,body); counts[c.topic]+=1
                                checksum=(checksum+fingerprint(info[0],info[1],info[2],timestamp,body))%(1<<256)
                                first=timestamp if first is None else min(first,timestamp)
                                last=timestamp if last is None else max(last,timestamp)
                            else: raise ValueError(f'数据块内不支持的记录类型{kind}')
                        except Exception as exc:
                            note('跳过本块剩余内容：'+str(exc),source_offset=offset,uncompressed_offset=pos,
                                 unprocessed_uncompressed_bytes=len(payload)-pos); break
                    if chunks%250==0:
                        print(f'  {100*f.tell()/size:.1f}%：{chunks}个数据块，{sum(counts.values()):,}条消息',flush=True)
                elif op not in (3,4,6):
                    note(f'未知顶层记录类型{op}，停止扫描',source_offset=offset); break
                if not complete: break
        report.update(recovered_chunks=chunks,skipped_chunks=skipped,topic_counts=dict(counts),
            recovered_messages=sum(counts.values()),source_message_fingerprint=f'{checksum:064x}',
            duration_s=(last-first)/1e9 if first is not None else 0.)
        if not counts: raise ValueError('未恢复到完整消息，临时副本不作为成功结果')
        print('3/4 完整读取恢复副本，核对消息字节和时间戳……',flush=True)
        actual=Counter(); check_sum=0
        with Reader(temp) as reader:
            for c,t,data in reader.messages():
                actual[c.topic]+=1
                check_sum=(check_sum+fingerprint(c.topic,c.msgtype,c.digest,t,data))%(1<<256)
        report['verified_message_counts']=dict(actual)
        report['output_message_fingerprint']=f'{check_sum:064x}'
        report['verified']=actual==counts and check_sum==checksum
        if not report['verified']: raise ValueError('恢复副本的消息字节/时间戳/数量核验失败')
        print('4/4 复核原文件SHA256……',flush=True)
        report['source_sha256_after']=sha(source)
        report['source_unchanged']=report['source_sha256_before']==report['source_sha256_after']
        if not report['source_unchanged']: raise ValueError('分析期间原文件改变，临时副本不能作为稳定恢复结果')
        temp.rename(output)
        report['output_sha256']=sha(output)
        report['status']='RECOVERED_WITH_LOSS_OR_WARNINGS' if report['issues'] else 'RECOVERED'
    except Exception as exc:
        report['status']='FAILED'; report['error']=f'{type(exc).__name__}: {exc}'
        report['partial_output']=str(temp) if temp.exists() else None
    report['elapsed_s']=round(time.monotonic()-started,2)
    report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    lines=['# ROS1 BAG恢复结果','',f'状态：{report["status"]}', '',f'原文件：`{source}`','',f'恢复副本：`{output}`','',
           '| 项目 | 结果 |','| --- | --- |',f'| 恢复消息数 | {report.get("recovered_messages",0):,} |',
           f'| 恢复数据块 | {chunks} |',f'| 无法解压数据块 | {skipped} |',
           f'| 消息字节、时间戳和数量核验 | {report["verified"]} |',f'| 原文件SHA256不变 | {report["source_unchanged"]} |',
           f'| 恢复记录时间跨度 | {report.get("duration_s",0):.3f}秒 |',f'| 耗时 | {report["elapsed_s"]}秒 |','',
           '## 各话题消息数','', '| 话题 | 数量 |','| --- | ---: |']
    lines += [f'| {topic} | {count:,} |' for topic,count in counts.items()]
    lines += ['', '## 扫描异常与损失范围','']
    lines += ['- '+json.dumps(item,ensure_ascii=False) for item in report['issues']] or ['- 未发现扫描异常。']
    if 'error' in report: lines += ['', '执行错误：'+report['error']]
    lines += ['', '说明：原序列化消息和录制时间戳保留，输出重新打包并写入索引，字节级文件哈希会不同。仅保留可定位的完整记录；原文件仍保留所有残片。遇到坏头停止扫描，无法解压的压缩块会跳过，不承诺恢复任意损坏；不填零、不插值、不补造消息。',
              '', '核验比较的是消息内容、时间戳及数量的无序指纹，允许重新打包改变存储顺序。原消息能否按类型正确解码以及静止/点云质量，应继续使用check_bag_dataset.py检验。']
    md_path.write_text('\n'.join(lines),encoding='utf-8')
    print('状态：',report['status'],'；报告：',md_path,flush=True)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',required=True,type=Path,help='原始ROS1 .bag路径')
    parser.add_argument('--output',type=Path,help='新的恢复.bag路径，禁止覆盖原件或已有文件')
    parser.add_argument('--max-record-mb',type=int,default=256,help='单记录内存保护上限MiB，默认256')
    args=parser.parse_args()
    if not args.bag.is_file(): parser.error('原文件不存在')
    if args.max_record_mb<1: parser.error('--max-record-mb至少1')
    output=args.output or Path(__file__).resolve().parent/'repaired_bags'/(args.bag.stem+'.'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.recovered.bag')
    try: r=repair(args.bag,output,args.max_record_mb)
    except (OSError,ValueError) as exc: parser.exit(2,'无法开始恢复：'+str(exc)+'\n')
    return 2 if r['status']=='FAILED' else 0


if __name__=='__main__': raise SystemExit(main())
