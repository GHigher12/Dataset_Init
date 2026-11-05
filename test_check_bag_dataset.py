"""用合成BAG验证布尔结果，含正例、损坏数值/时间及缺失数据。"""
from pathlib import Path
import struct
import tempfile
import unittest
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from check_bag_dataset import check


def string(s):
    b=s.encode(); return struct.pack('<I',len(b))+b


def header(t,frame):
    sec=int(t); nano=int(round((t-sec)*1e9))
    return struct.pack('<III',0,sec,nano)+string(frame)


def fixture(path,bad=False):
    store=get_typestore(Stores.ROS1_NOETIC)
    store.register(get_types_from_msg('geometry_msgs/TransformStamped[] transforms\n','tf2_msgs/msg/TFMessage'))
    # Include a real Header in this synthetic custom message to test sensor clock checks.
    store.register(get_types_from_msg('std_msgs/Header header\nuint32 point_num\ngeometry_msgs/Point[] points\n','test_driver/msg/CustomMsg'))
    with Writer(path) as w:
        im=w.add_connection('/livox/imu','sensor_msgs/msg/Imu',typestore=store)
        li=w.add_connection('/livox/lidar','test_driver/msg/CustomMsg',typestore=store)
        tf=w.add_connection('/tf','tf2_msgs/msg/TFMessage',typestore=store)
        for index in range(161):
            t=100+index*.05
            mt=100.1 if bad and index==20 else t
            a=[0.,0.,float('nan') if bad and index==25 else 1.]
            body=header(mt,'imu')+struct.pack('<4d',0,0,0,0)+bytes(72)+struct.pack('<3d',0,0,-.053)+bytes(72)+struct.pack('<3d',*a)+bytes(72)
            w.write(im,round(t*1e9),body)
            if index%2==0:
                count=2 if bad and index==10 else 1
                x=float('inf') if bad and index==12 else 1.
                w.write(li,round(t*1e9),header(t,'lidar')+struct.pack('<II3d',count,1,x,2,3))
                body=struct.pack('<I',1)+header(t,'camera_init')+string('aft_mapped')+struct.pack('<7d',0,0,0,0,0,0,1)
                w.write(tf,round(t*1e9),body)


class DatasetChecksTests(unittest.TestCase):
    def test_complete_fixture_has_exactly_sixteen_true_booleans(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp); fixture(p/'ok.bag')
            r=check(p/'ok.bag',p/'out',start_window=2)
            self.assertEqual(len(r['metrics']),16)
            self.assertTrue(all(type(x['通过']) is bool for x in r['metrics']))
            self.assertTrue(r['all_selected_checks_passed'],r)
            self.assertTrue((p/'out'/'检验指标.csv').is_file())

    def test_bad_values_and_clock_are_not_hidden(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp); fixture(p/'bad.bag',bad=True)
            r=check(p/'bad.bag',p/'out',start_window=2)
            rows={v['id']:v['通过'] for v in r['metrics']}
            for name in ('imu_finite','point_finite','point_count','time_forward','closure'):
                self.assertFalse(rows[name],name)
            self.assertTrue(rows['readable'])
            self.assertTrue(rows['decode'])

    def test_missing_data_is_not_vacuously_true(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            with Writer(p/'missing.bag') as w:
                c=w.add_connection('/test','std_msgs/msg/String',msgdef='string data\n',md5sum='992ce8a1687cec8c8bd883ec73ca41d1')
                w.write(c,1000000000,string('test'))
            r=check(p/'missing.bag',p/'out')
            rows={v['id']:v['通过'] for v in r['metrics']}
            for name in ('topics','time_forward','time_unique','time_gaps','imu_finite','point_count','point_finite','nonempty','start_static','trajectory'):
                self.assertFalse(rows[name],name)

    def test_truncated_file_still_outputs_failure_table(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp); (p/'broken.bag').write_bytes(b'#ROSBAG V2.0\ntruncated')
            r=check(p/'broken.bag',p/'out')
            self.assertFalse(r['all_selected_checks_passed'])
            self.assertEqual(len(r['metrics']),16)
            self.assertTrue((p/'out'/'检验结果_中文.md').is_file())


if __name__=='__main__': unittest.main()
