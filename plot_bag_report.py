"""将检验JSON及同目录CSV转换为中文表格图和运动概览图，不读取BAG。"""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np


def plot(report_path,output=None,font=None):
    report_path=Path(report_path)
    r=json.loads(report_path.read_text(encoding='utf-8'))
    output=Path(output) if output else report_path.parent/'charts'
    output.mkdir(parents=True,exist_ok=True)
    if font:
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name()
    else:
        plt.rcParams['font.sans-serif']=['Microsoft YaHei','SimHei','Noto Sans CJK SC','WenQuanYi Zen Hei','DejaVu Sans']
    plt.rcParams['axes.unicode_minus']=False
    rows=r['metrics']; passed=sum(row['通过'] for row in rows)
    fig,ax=plt.subplots(figsize=(18,10)); ax.axis('off')
    ax.set_title(f'BAG数据集检验表：{passed}/{len(rows)}项通过',fontsize=20,pad=18)
    cells=[[row['检验指标'],row['实测结果'],str(row['通过'])] for row in rows]
    table=ax.table(cellText=cells,colLabels=['检验指标','实测结果','通过'],colWidths=[.30,.63,.07],
                   cellLoc='left',loc='center',bbox=[0,.08,1,.88])
    table.auto_set_font_size(False); table.set_fontsize(10)
    for (i,j),cell in table.get_celld().items():
        cell.set_edgecolor('#d1d9e0')
        if i==0:
            cell.set_facecolor('#17344c'); cell.set_text_props(color='white',weight='bold')
        else:
            cell.set_facecolor('#f4f8fc' if i%2 else 'white')
            if j==2:
                cell.set_text_props(color='#167c48' if rows[i-1]['通过'] else '#b42318',weight='bold')
    ax.text(0,.025,'True表示所选检查通过；闭合残差是内部一致性指标。恢复副本读通不证明原始采集完整。',transform=ax.transAxes,fontsize=12)
    fig.savefig(output/'metrics_table.png',dpi=160,bbox_inches='tight'); plt.close(fig)
    imu_path=report_path.parent/'imu.csv'; pose_path=report_path.parent/'trajectory.csv'
    if imu_path.is_file() and pose_path.is_file():
        imu=np.atleast_2d(np.loadtxt(imu_path,delimiter=',',skiprows=1))
        tr=np.atleast_2d(np.loadtxt(pose_path,delimiter=',',skiprows=1))
        t0=imu[0,0]; start=r['parameters']['start_window_s']; end=r['parameters']['end_window_s']
        fig,axes=plt.subplots(2,2,figsize=(13,9))
        ax=axes[0,0]; ax.plot(tr[:,1],tr[:,2],color='#2475ac')
        ax.scatter(tr[[0,-1],1],tr[[0,-1],2],c=['green','red'],s=45)
        ax.set(title='估计轨迹：绿起点、红记录终点',xlabel='X（米）',ylabel='Y（米）'); ax.axis('equal')
        ax=axes[0,1]; ax.plot(imu[:,0]-t0,np.linalg.norm(imu[:,4:7],axis=1))
        ax.axvspan(0,start,color='green',alpha=.15); ax.axvspan(imu[-1,0]-t0-end,imu[-1,0]-t0,color='orange',alpha=.2)
        ax.set(title='角速度模及首尾候选窗口',xlabel='相对开始时间（秒）',ylabel='角速度模（rad/s）')
        scale=r.get('details',{}).get('acceleration',{}).get('scale_to_m_s2',1)
        ax=axes[1,0]; ax.plot(imu[:,0]-t0,np.linalg.norm(imu[:,1:4]*scale,axis=1))
        ax.set(title='加速度模（单位以检验设置为准）',xlabel='相对开始时间（秒）',ylabel='加速度模（m/s²）')
        ax=axes[1,1]
        first=tr[(tr[:,0]>=t0)&(tr[:,0]<=t0+start),1:4]
        last=tr[(tr[:,0]>=imu[-1,0]-end)&(tr[:,0]<=imu[-1,0]),1:4]
        if len(first) and len(last):
            center=first.mean(0)
            for label,v in [('首段',first),('末段',last)]:
                ax.scatter((v[:,0]-center[0])*1000,(v[:,1]-center[1])*1000,label=label,s=15)
            ax.legend()
        ax.set(title='首尾估计位置散布（非定位精度）',xlabel='相对首段中心X（毫米）',ylabel='相对首段中心Y（毫米）'); ax.axis('equal')
        for ax in axes.flat: ax.grid(alpha=.25)
        fig.tight_layout(); fig.savefig(output/'motion_summary.png',dpi=160); plt.close(fig)
    print('图表目录：',output.resolve())
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',required=True,type=Path,help='检验指标.json路径')
    parser.add_argument('--output',type=Path,help='图表输出目录，默认报告旁charts目录')
    parser.add_argument('--font',type=Path,help='可选中文字体ttf/ttc/otf路径')
    args=parser.parse_args(); plot(args.report,args.output,args.font)
