"""检验ROS1/ROS2 bag的16项指标；--ros-version默认1，输出中文True/False表格。"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timedelta, timezone
import csv
import hashlib
import json
import shutil
import unicodedata
from pathlib import Path
import numpy as np


ITEMS = [
    ('readable','文件结构','BAG能否完整读取'),
    ('decode','文件结构','消息能否正确解码'),
    ('counts','文件结构','实际消息数是否与索引一致'),
    ('topics','文件结构','必要话题是否存在'),
    ('unchanged','文件保护','分析期间文件是否保持不变'),
    ('time_forward','时间质量','传感器时间戳是否无回退'),
    ('time_unique','时间质量','传感器时间戳是否无重复'),
    ('time_gaps','时间质量','是否无明显采样缺口'),
    ('imu_finite','数值质量','IMU加速度及角速度是否为有限值'),
    ('point_count','点云质量','点云声明点数或布局是否一致'),
    ('point_finite','点云质量','点云XYZ坐标是否为有限值'),
    ('nonempty','点云质量','是否无空点云'),
    ('start_static','静止候选段','首段是否支持近似静止'),
    ('end_static','静止候选段','末段是否支持近似静止'),
    ('trajectory','轨迹分析','能否计算轨迹内部指标'),
    ('closure','轨迹分析','能否计算条件性闭合残差'),
]


def display_width(text):
    """Count terminal cells: Chinese/full-width characters use two cells."""
    return sum(0 if unicodedata.combining(c) else 2 if unicodedata.east_asian_width(c) in ('W','F') else 1 for c in text)


def wrap_cells(text,width):
    text=str(text).replace('\t','    ').replace('\r','')
    lines=[]
    for paragraph in text.split('\n'):
        current=''; used=0
        for char in paragraph:
            size=display_width(char)
            if used+size>width and current:
                lines.append(current); current=''; used=0
            current+=char; used+=size
        lines.append(current)
    return lines


def format_metric_table(rows,terminal_width=None):
    columns=min(128,terminal_width or shutil.get_terminal_size(fallback=(120,24)).columns)
    name_width=min(32,max([display_width(r['检验指标']) for r in rows]+[8]),max(14,(columns-15)//3))
    widths=[name_width,max(18,columns-name_width-15),5]
    border='+'+'+'.join('-'*(width+2) for width in widths)+'+'
    def render(cells):
        wrapped=[wrap_cells(cell,width) for cell,width in zip(cells,widths)]
        lines=[]
        for index in range(max(map(len,wrapped))):
            parts=[]
            for col,(cell,width) in enumerate(zip(wrapped,widths)):
                text=cell[index] if index<len(cell) else ''
                padding=width-display_width(text)
                if col==2:
                    text=' '*(padding//2)+text+' '*(padding-padding//2)
                else:
                    text+=' '*padding
                parts.append(' '+text+' ')
            lines.append('|'+'|'.join(parts)+'|')
        return lines
    lines=[border,*render(['检验指标','实测结果','通过']),border]
    for row in rows:
        lines.extend(render([row['检验指标'],row['实测结果'],str(row['通过'])]))
        lines.append(border)
    return '\n'.join(lines)


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(8*1024*1024),b''): h.update(block)
    return h.hexdigest()


def input_snapshot(path):
    """Directory fingerprint includes file names and every file's content."""
    if path.is_file():
        value=digest(path)
        return value,{path.name:value}
    files={p.relative_to(path).as_posix():digest(p) for p in sorted(path.rglob('*')) if p.is_file()}
    encoded=json.dumps(files,sort_keys=True,ensure_ascii=False).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest(),files


def pointcloud2_xyz(msg):
    """Check byte layout and expose XYZ with row padding and endian preserved."""
    if msg.height<0 or msg.width<0 or msg.point_step<=0 or msg.row_step<msg.width*msg.point_step or len(msg.data)!=msg.row_step*msg.height:
        raise ValueError('PointCloud2字节长度/行宽/点步长不一致')
    formats={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
    fields={f.name:f for f in msg.fields}
    xyz=[]
    for name in ('x','y','z'):
        f=fields.get(name)
        if f is None or f.count!=1 or f.datatype not in formats:
            raise ValueError('PointCloud2缺少支持的单值XYZ字段')
        dtype=np.dtype(('>' if msg.is_bigendian else '<')+formats[f.datatype])
        if f.offset<0 or f.offset+dtype.itemsize>msg.point_step: raise ValueError('PointCloud2字段偏移超出点步长')
        if msg.width*msg.height==0: xyz.append(np.empty((msg.height,msg.width))); continue
        xyz.append(np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                              offset=f.offset,strides=(msg.row_step,msg.point_step)))
    return xyz


def stamp(t):
    return t.sec+t.nanosec/1e9


def time_stats(times,gap_factor):
    a=np.asarray(times,dtype=float); d=np.diff(a); positive=d[d>0]
    median=float(np.median(positive)) if len(positive) else None
    return {'samples':len(a),'backwards':int((d<0).sum()),'duplicates':int((d==0).sum()),
        'duration_s':float(a[-1]-a[0]) if len(a) else 0.,
        'median_rate_hz':1/median if median else None,
        'max_gap_s':float(positive.max()) if len(positive) else None,
        'large_gaps':int((d>gap_factor*median).sum()) if median else 0,
        'enough_samples':bool(len(a)>=3 and median is not None)}


def angle(q1,q2):
    q1,q2=q1/np.linalg.norm(q1),q2/np.linalg.norm(q2)
    return float(np.degrees(2*np.arccos(np.clip(abs(q1@q2),0,1))))


def mean_q(q):
    q=q/np.linalg.norm(q,axis=1)[:,None]
    _,vectors=np.linalg.eigh(q.T@q)
    return vectors[:,-1]


def quiet_window(imu,pose,start,end,scale,gap_factor):
    v=imu[(imu[:,0]>=start)&(imu[:,0]<=end)]
    p=pose[(pose[:,0]>=start)&(pose[:,0]<=end)]
    if len(v)<20 or len(p)<3:
        return {'passed':False,'reason':'IMU不足20条或轨迹不足3条','imu_samples':len(v),'pose_samples':len(p)}
    a,g=v[:,1:4]*scale,v[:,4:7]
    centered=g-g.mean(0)
    grid=np.arange(v[0,0],v[-1,0],.01)
    if len(grid)<20: return {'passed':False,'reason':'窗口不足以计算0.2秒平滑统计'}
    smoothed=np.array([np.convolve(np.interp(grid,v[:,0],a[:,i]),np.ones(20)/20,'valid') for i in range(3)]).T
    pg=np.arange(p[0,0],p[-1,0],.2)
    if len(pg)<3: return {'passed':False,'reason':'位姿时间覆盖不足'}
    xyz=np.array([np.interp(pg,p[:,0],p[:,i]) for i in (1,2,3)]).T
    speed=np.linalg.norm(np.diff(xyz,axis=0),axis=1)/.2
    r={'imu_samples':len(v),'pose_samples':len(p),'coverage_fraction':float(min(v[-1,0]-v[0,0],p[-1,0]-p[0,0])/(end-start)),
       'gyro_mean_xyz_rad_s':g.mean(0).tolist(),
       'raw_gyro_norm_p95_rad_s':float(np.percentile(np.linalg.norm(g,axis=1),95)),
       'raw_accel_residual_rms_m_s2':float(np.sqrt(np.mean(np.sum((a-a.mean(0))**2,axis=1)))),
       'demeaned_gyro_norm_p95_rad_s':float(np.percentile(np.linalg.norm(centered,axis=1),95)),
       'smoothed_accel_residual_rms_m_s2':float(np.sqrt(np.mean(np.sum((smoothed-smoothed.mean(0))**2,axis=1)))),
       'speed_p95_m_s':float(np.percentile(speed,95)),
       'position_radius_p95_m':float(np.percentile(np.linalg.norm(p[:,1:4]-p[:,1:4].mean(0),axis=1),95)),
       'pose_angle_p95_deg':float(np.percentile([angle(p[0,4:8],q) for q in p[:,4:8]],95))}
    t=time_stats(v[:,0],gap_factor)
    r['passed']=bool(r['coverage_fraction']>=.8 and t['large_gaps']==0 and
        r['demeaned_gyro_norm_p95_rad_s']<.03 and r['smoothed_accel_residual_rms_m_s2']<.15 and
        r['speed_p95_m_s']<.05 and r['position_radius_p95_m']<.02 and r['pose_angle_p95_deg']<.5)
    r['reason']='联合支持近似静止，雷达IMU偏置与噪声仍需复核' if r['passed'] else '未满足近似静止联合阈值'
    return r


def check(bag,output,start_window=5.,end_window=2.,gap_factor=5.,accel_unit='auto',
          imu_topic='/livox/imu',lidar_topic='/livox/lidar',pose_child='aft_mapped',
          ros_version=1,ros_distro='humble',msg_defs=None):
    from rosbags.highlevel import AnyReader
    bag=Path(bag).resolve(); output=Path(output).resolve()
    if ros_version not in (1,2): raise ValueError('ros_version只能为1或2')
    if not bag.exists(): raise ValueError('输入路径不存在')
    if bag.is_dir() and output.is_relative_to(bag): raise ValueError('输出目录不能位于ROS2输入目录内，否则会改变被检验数据')
    output.mkdir(parents=True,exist_ok=False)
    details={}; errors=[]; values={key:(False,'未实施或证据不足') for key,_,_ in ITEMS}
    def result(key,passed,text): values[key]=(bool(passed),text)
    before,before_files=input_snapshot(bag)
    expected,counts,decode_errors=Counter(),Counter(),Counter()
    times=defaultdict(list); imu=[]; pose=[]; parent_frames=set()
    measured=Counter(); loaded=False; metadata=False
    print(f'ROS{ros_version} BAG：全量读取与解码',bag,flush=True)
    try:
        if ros_version==1 and (not bag.is_file() or bag.suffix.lower()!='.bag'):
            raise ValueError('ROS1模式需要.bag文件；ROS2请增加 --ros-version 2')
        if ros_version==2 and bag.suffix.lower()=='.bag' and bag.is_file():
            raise ValueError('ROS2模式不接受ROS1 .bag文件')
        typestore=None
        if ros_version==2:
            from rosbags.typesys import Stores, get_typestore, get_types_from_msg
            typestore=get_typestore(Stores('ros2_'+ros_distro))
            definitions={}
            for spec in msg_defs or []:
                typename,filename=spec.split('=',1)
                definitions.update(get_types_from_msg(Path(filename).read_text(encoding='utf-8'),typename))
            if definitions: typestore.register(definitions)
        with AnyReader([bag],default_typestore=typestore) as reader:
            if ros_version==2 and definitions: reader.typestore.register(definitions)
            for c in reader.connections: expected[c.topic]+=c.msgcount
            metadata=True
            for c,_,raw in reader.messages():
                counts[c.topic]+=1
                try: m=reader.deserialize(raw,c.msgtype)
                except Exception as exc:
                    decode_errors[c.topic]+=1
                    if decode_errors[c.topic]==1: errors.append(f'{c.topic} 解码失败：{exc}')
                    continue
                try:
                    if c.topic==imu_topic and c.msgtype.endswith('/Imu'):
                        t=stamp(m.header.stamp); times[c.topic].append(t)
                        a,g=m.linear_acceleration,m.angular_velocity
                        v=[a.x,a.y,a.z,g.x,g.y,g.z]; measured['imu_checked']+=1
                        measured['imu_nonfinite']+=int((~np.isfinite(v)).sum())
                        imu.append([t,*v])
                    elif c.topic==lidar_topic and c.msgtype.endswith('/CustomMsg'):
                        times[c.topic].append(stamp(m.header.stamp)); measured['lidar_checked']+=1
                        measured['point_mismatch']+=int(m.point_num!=len(m.points))
                        measured['empty_clouds']+=int(len(m.points)==0)
                        if len(m.points):
                            xyz=np.asarray([[p.x,p.y,p.z] for p in m.points],dtype=float)
                            measured['xyz_nonfinite']+=int((~np.isfinite(xyz)).sum())
                    elif c.topic==lidar_topic and c.msgtype.endswith('/PointCloud2'):
                        times[c.topic].append(stamp(m.header.stamp)); measured['lidar_checked']+=1
                        measured['empty_clouds']+=int(m.width*m.height==0)
                        try: xyz=pointcloud2_xyz(m)
                        except ValueError:
                            measured['point_mismatch']+=1
                            raise
                        measured['xyz_nonfinite']+=sum(int((~np.isfinite(axis)).sum()) for axis in xyz)
                    elif c.topic=='/tf' and c.msgtype.endswith('/TFMessage'):
                        for tf in m.transforms:
                            if tf.child_frame_id.strip('/')!=pose_child.strip('/'): continue
                            p,q=tf.transform.translation,tf.transform.rotation
                            pose.append([stamp(tf.header.stamp),p.x,p.y,p.z,q.x,q.y,q.z,q.w])
                            parent_frames.add(tf.header.frame_id.strip('/'))
                except Exception as exc:
                    measured['analysis_errors']+=1
                    if measured['analysis_errors']<=3: errors.append(f'{c.topic} 内容检查失败：{exc}')
            loaded=True
    except Exception as exc: errors.append(f'BAG无法完整读取：{type(exc).__name__}: {exc}')
    total=sum(counts.values()); dec=sum(decode_errors.values())
    result('readable',loaded and total>0,f'遍历消息 {total:,} 条；读取异常 {0 if loaded else 1}')
    result('decode',loaded and total>0 and dec==0,f'解码失败 {dec:,} 条')
    result('counts',loaded and metadata and total>0 and counts==expected,f'索引 {sum(expected.values()):,} 条，实际 {total:,} 条')
    sensor_ok=all(measured[k]>0 for k in ('imu_checked','lidar_checked'))
    result('topics',loaded and sensor_ok,f'{imu_topic} 有效解码 {measured["imu_checked"]:,} 条；{lidar_topic} {measured["lidar_checked"]:,} 帧')
    timeline={topic:time_stats(times[topic],gap_factor) for topic in (imu_topic,lidar_topic)}
    enough=loaded and all(s['enough_samples'] for s in timeline.values())
    for key,field,label in [('time_forward','backwards','回退'),('time_unique','duplicates','重复'),('time_gaps','large_gaps','大间隔')]:
        result(key,enough and all(s[field]==0 for s in timeline.values()),'；'.join(f'{t} {label} {s[field]} 次' for t,s in timeline.items()))
    imu_checked=loaded and measured['imu_checked']>0 and counts[imu_topic]==measured['imu_checked']
    lidar_checked=loaded and measured['lidar_checked']>0 and counts[lidar_topic]==measured['lidar_checked']
    result('imu_finite',imu_checked and measured['imu_nonfinite']==0,f'加速度/角速度 NaN及Inf {measured["imu_nonfinite"]} 个')
    for key,field,label in [('point_count','point_mismatch','点数不符'),('point_finite','xyz_nonfinite','XYZ非法数值'),('nonempty','empty_clouds','空点云')]:
        result(key,lidar_checked and measured['analysis_errors']==0 and measured[field]==0,f'{label} {measured[field]}')
    i=np.asarray(imu,dtype=float); p=np.asarray(pose,dtype=float)
    imu_ok=bool(imu_checked and measured['imu_nonfinite']==0 and len(i)>=20 and np.isfinite(i).all() and np.all(np.diff(i[:,0])>0))
    pose_ok=bool(loaded and len(p)>=3 and np.isfinite(p).all() and len(parent_frames)==1 and
                 np.all(np.diff(p[:,0])>0) and np.all(np.abs(np.linalg.norm(p[:,4:8],axis=1)-1)<.001))
    if pose_ok:
        duration=float(p[-1,0]-p[0,0]); length=float(np.linalg.norm(np.diff(p[:,1:4],axis=0),axis=1).sum())
        details['trajectory']={'samples':len(p),'duration_s':duration,'path_length_m':length,'parent_frames':sorted(parent_frames)}
        result('trajectory',True,f'轨迹 {len(p):,} 条，{duration:.3f} 秒，折线路径 {length:.3f} 米')
    if imu_ok and pose_ok and i[-1,0]-i[0,0]>=max(start_window,end_window):
        norm=float(np.median(np.linalg.norm(i[:,1:4],axis=1)))
        scale=9.80665 if accel_unit=='g' or (accel_unit=='auto' and .5<norm<2) else 1.
        details['acceleration']={'setting':accel_unit,'raw_median_norm':norm,'scale_to_m_s2':scale}
        bounds=[('start_static',i[0,0],i[0,0]+start_window),('end_static',i[-1,0]-end_window,i[-1,0])]
        for key,s,e in bounds:
            w=quiet_window(i,p,s,e,scale,gap_factor); details[key]=w
            result(key,w['passed'],f'{e-s:g}秒窗口：{w["reason"]}')
        a=p[(p[:,0]>=bounds[0][1])&(p[:,0]<=bounds[0][2])]
        b=p[(p[:,0]>=bounds[1][1])&(p[:,0]<=bounds[1][2])]
        if len(a)>=3 and len(b)>=3:
            delta=b[:,1:4].mean(0)-a[:,1:4].mean(0)
            distance=float(np.linalg.norm(delta)); rotation=angle(mean_q(a[:,4:8]),mean_q(b[:,4:8]))
            details['endpoint_residual']={'delta_xyz_m':delta.tolist(),'distance_m':distance,'rotation_deg':rotation,
                'start_samples':len(a),'end_samples':len(b),'start_spread_rms_m':float(np.sqrt(np.mean(np.sum((a[:,1:4]-a[:,1:4].mean(0))**2,axis=1)))),
                'end_spread_rms_m':float(np.sqrt(np.mean(np.sum((b[:,1:4]-b[:,1:4].mean(0))**2,axis=1))))}
            supported=values['start_static'][0] and values['end_static'][0]
            result('closure',supported,f'平均位置差 {distance*1000:.3f} mm；姿态差 {rotation:.4f}°；'+('首尾近似静止均支持' if supported else '首尾静止条件未同时支持，仅为描述性位置差'))
    try: after,after_files=input_snapshot(bag)
    except OSError as exc:
        after,after_files=None,{}; errors.append(f'输入复核失败：{exc}')
    result('unchanged',before==after,'分析前后SHA256'+('一致' if before==after else '不同'))
    if pose_ok:
        np.savetxt(output/'trajectory.csv',p,delimiter=',',header='time_s,x_m,y_m,z_m,qx,qy,qz,qw',comments='')
    if imu_ok:
        np.savetxt(output/'imu.csv',i,delimiter=',',header='time_s,ax_raw,ay_raw,az_raw,gx_rad_s,gy_rad_s,gz_rad_s',comments='')
    rows=[{'id':k,'类别':category,'检验指标':label,'实测结果':values[k][1],'通过':values[k][0]} for k,category,label in ITEMS]
    report={'bag':str(bag),'ros_version':ros_version,'input_files_before':before_files,'input_files_after':after_files,
        'generated_beijing':datetime.now(timezone(timedelta(hours=8))).isoformat(),
        'sha256_before':before,'sha256_after':after,'all_selected_checks_passed':all(r['通过'] for r in rows),
        'parameters':{'start_window_s':start_window,'end_window_s':end_window,'gap_factor':gap_factor,
                      'accel_unit':accel_unit,'imu_topic':imu_topic,'lidar_topic':lidar_topic,'pose_child':pose_child,
                      'ros_version':ros_version,'ros_distro':ros_distro if ros_version==2 else None,'msg_defs':list(msg_defs or [])},
        'metrics':rows,'topic_counts':dict(counts),'expected_counts':dict(expected),'decode_errors':dict(decode_errors),
        'sensor_counters':dict(measured),'time_statistics':timeline,'details':details,'errors':errors}
    (output/'检验指标.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    with (output/'检验指标.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['类别','检验指标','实测结果','通过']); writer.writeheader()
        writer.writerows({k:v for k,v in row.items() if k!='id'} for row in rows)
    lines=['# BAG数据集16项检验','',f'输入：`{bag}`','',f'格式：ROS{ros_version}','',f'时间：{report["generated_beijing"]}','',
        '仅包含约定的16项指标。True表示该项通过；False表示未通过或证据不足，实际异常不隐藏。','',
        '| 类别 | 检验指标 | 实测结果 | 通过 |','| --- | --- | --- | --- |']
    lines += [f'| {r["类别"]} | {r["检验指标"]} | {r["实测结果"].replace(chr(10)," ").replace("|","/")} | {r["通过"]} |' for r in rows]
    lines += ['', '说明：', '',
        '- 仅检查指定BAG，不读取BIN、RLOG、TLOG，不修改或修复文件；不评估定位精度或落座重复性。',
        '- 文件读通和索引数量一致仅证明当前文件内部可读，不能证明原始采集无丢失；SHA256仅核对分析期间未改变。',
        f'- 时间检查针对两个配置的传感器话题消息头时间；大间隔阈值为各自正间隔中位数的{gap_factor:g}倍，不把事件话题当周期传感器。没有明显缺口不等于证明零丢帧。',
        '- 静止筛查要求窗口覆盖≥80%；去均值角速度模P95<0.03rad/s、0.2秒平滑加速度残差RMS<0.15m/s²、轨迹速度P95<0.05m/s、位置半径P95<0.02m、位姿角变化P95<0.5°。阈值为启发式；去均值仅作诊断，不代表标定。',
        '- auto加速度单位：原始加速度模中位数在0.5～2时暂按g换算，否则暂按m/s²；建议核对驱动后显式指定 --accel-unit。',
        '- “能否计算闭合残差”通过不代表残差达到验收标准。必须额外确认从同一物理位置出发返回、坐标系连续；雷达位姿与IMU可能非独立。',
        '- 缺少传感器/轨迹、样本不足或无法读取时，对应项为False，不把未检查当作通过。','']
    if ros_version==2:
        lines += ['ROS2说明：目录模式对目录内全部文件（包含metadata.yaml和分片）前后计算指纹；单文件模式只覆盖指定.db3/.mcap，不代表整个分片数据集。点云项同时支持Livox CustomMsg与PointCloud2，后者检查width/height/row_step/point_step布局及全部XYZ。','']
    if errors: lines += ['读取/分析异常：','']+['- '+e.replace('\n',' ') for e in errors]
    (output/'检验结果_中文.md').write_text('\n'.join(lines),encoding='utf-8')
    print('\n'+format_metric_table(rows)+'\n')
    print(f'完成：{sum(r["通过"] for r in rows)}/16 项为True；{output/"检验结果_中文.md"}',flush=True)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',required=True,type=Path,help='ROS1 .bag文件，或ROS2 bag目录/.db3/.mcap文件')
    parser.add_argument('--ros-version',type=int,choices=[1,2],default=1,help='选择ROS版本，默认1；ROS2请传2')
    parser.add_argument('--ros-distro',default='humble',choices=['dashing','eloquent','foxy','galactic','humble','iron','jazzy','kilted','lyrical'],help='ROS2缺少内嵌定义时的标准消息类型，默认humble')
    parser.add_argument('--msg-def',action='append',default=[],help='补充ROS2自定义定义：package/msg/Type=文件.msg，可重复，依赖也需提供')
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'bag_checks'/datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S'),help='新建输出目录，必须不存在')
    parser.add_argument('--start-window',type=float,default=5,help='首段窗口秒数，默认5')
    parser.add_argument('--end-window',type=float,default=2,help='末段窗口秒数，默认2')
    parser.add_argument('--gap-factor',type=float,default=5,help='大间隔相对中位数倍数，默认5')
    parser.add_argument('--accel-unit',choices=['auto','g','mps2'],default='auto',help='雷达加速度原始单位，默认推断')
    parser.add_argument('--imu-topic',default='/livox/imu')
    parser.add_argument('--lidar-topic',default='/livox/lidar')
    parser.add_argument('--pose-child',default='aft_mapped',help='/tf轨迹子坐标系')
    args=parser.parse_args()
    if not args.bag.exists(): parser.error('BAG文件或目录不存在')
    if args.bag.is_dir() and args.output.resolve().is_relative_to(args.bag.resolve()): parser.error('输出目录不能位于ROS2输入目录内')
    if not all(np.isfinite(x) for x in (args.start_window,args.end_window,args.gap_factor)) or min(args.start_window,args.end_window)<1 or args.gap_factor<=1:
        parser.error('窗口必须为至少1秒的有限数，大间隔倍数必须为大于1的有限数')
    r=check(args.bag,args.output,args.start_window,args.end_window,args.gap_factor,args.accel_unit,args.imu_topic,args.lidar_topic,args.pose_child,args.ros_version,args.ros_distro,args.msg_def)
    return 0 if r['all_selected_checks_passed'] else 1


if __name__=='__main__': raise SystemExit(main())
