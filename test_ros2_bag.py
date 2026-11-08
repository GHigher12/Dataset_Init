"""ROS2真实CDR合成数据覆盖sqlite3、MCAP、压缩、自定义定义与异常路径。"""
import contextlib
import io
import tempfile
import sqlite3
import struct
import unittest
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer, StoragePlugin, CompressionMode, CompressionFormat
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from check_bag_dataset import check, pointcloud2_xyz
from test_check_bag_dataset import fixture


def fixture2(path,storage=StoragePlugin.SQLITE3,version=9,bad=False,compressed=False,pointcloud2=False):
    source=path.parent/(path.name+'_source.bag')
    fixture(source,bad=bad)
    store=get_typestore(Stores.ROS2_HUMBLE)
    store.register(get_types_from_msg('geometry_msgs/TransformStamped[] transforms\n','tf2_msgs/msg/TFMessage'))
    store.register(get_types_from_msg('std_msgs/Header header\nuint32 point_num\ngeometry_msgs/Point[] points\n','test_driver/msg/CustomMsg'))
    with AnyReader([source]) as reader:
        writer=Writer(path,version=version,storage_plugin=storage)
        if compressed: writer.set_compression(CompressionMode.FILE,CompressionFormat.ZSTD)
        with writer:
            connections={c.id:writer.add_connection(c.topic,'sensor_msgs/msg/PointCloud2' if pointcloud2 and c.topic=='/livox/lidar' else c.msgtype,typestore=store) for c in reader.connections}
            for c,t,raw in reader.messages():
                msg=reader.deserialize(raw,c.msgtype)
                if pointcloud2 and c.topic=='/livox/lidar':
                    field=store.types['sensor_msgs/msg/PointField']
                    msg=store.types['sensor_msgs/msg/PointCloud2'](header=msg.header,height=1,width=1,
                        fields=[field(name=n,offset=j*4,datatype=7,count=1) for j,n in enumerate(('x','y','z'))],
                        is_bigendian=False,point_step=12,row_step=12,data=np.frombuffer(struct.pack('<3f',1,2,3),dtype=np.uint8),is_dense=True)
                writer.write(connections[c.id],t,store.serialize_cdr(msg,connections[c.id].msgtype))
    source.unlink()


class Ros2Tests(unittest.TestCase):
    def run_check(self,path,out,**kw):
        with contextlib.redirect_stdout(io.StringIO()):
            return check(path,out,start_window=2,**kw)

    def test_sqlite_and_mcap_directory_and_single_file(self):
        for storage in (StoragePlugin.SQLITE3,StoragePlugin.MCAP):
            with self.subTest(storage=storage),tempfile.TemporaryDirectory() as tmp:
                base=Path(tmp); bag=base/'ros2'; fixture2(bag,storage)
                for index,path in enumerate([bag,next(bag.glob('*.db3' if storage==StoragePlugin.SQLITE3 else '*.mcap'))]):
                    r=self.run_check(path,base/f'out{index}',ros_version=2)
                    self.assertTrue(r['all_selected_checks_passed'],r['errors'])
                    self.assertEqual(r['ros_version'],2)
                    self.assertEqual(len(r['metrics']),16)
                    self.assertEqual(r['input_files_before'],r['input_files_after'])

    def test_old_sqlite_requires_custom_definition(self):
        with tempfile.TemporaryDirectory() as tmp:
            base=Path(tmp); bag=base/'old'; fixture2(bag,version=8)
            with sqlite3.connect(str(next(bag.glob('*.db3')))) as db:
                db.execute('DELETE FROM message_definitions')
            db.close()
            r=self.run_check(bag,base/'without',ros_version=2)
            self.assertFalse(r['all_selected_checks_passed'])
            definition=base/'CustomMsg.msg'
            definition.write_text('std_msgs/Header header\nuint32 point_num\ngeometry_msgs/Point[] points\n',encoding='utf-8')
            tfdef=base/'TFMessage.msg'; tfdef.write_text('geometry_msgs/TransformStamped[] transforms\n',encoding='utf-8')
            r=self.run_check(bag,base/'with',ros_version=2,msg_defs=[f'test_driver/msg/CustomMsg={definition}',f'tf2_msgs/msg/TFMessage={tfdef}'])
            self.assertTrue(r['all_selected_checks_passed'],r['errors'])

    def test_file_compression_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            base=Path(tmp); bag=base/'compressed'; fixture2(bag,compressed=True)
            r=self.run_check(bag,base/'out',ros_version=2)
            self.assertTrue(r['all_selected_checks_passed'],r['errors'])

    def test_bad_ros2_values_not_hidden(self):
        with tempfile.TemporaryDirectory() as tmp:
            base=Path(tmp); bag=base/'bad'; fixture2(bag,bad=True)
            r=self.run_check(bag,base/'out',ros_version=2)
            rows={v['id']:v['通过'] for v in r['metrics']}
            for key in ('time_forward','point_count','point_finite','imu_finite','closure'): self.assertFalse(rows[key])

    def test_missing_split_is_read_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            base=Path(tmp); bag=base/'missing'; fixture2(bag)
            next(bag.glob('*.db3')).unlink()
            r=self.run_check(bag,base/'out',ros_version=2)
            self.assertFalse(r['metrics'][0]['通过'])
            self.assertTrue(r['errors'])

    def test_nested_output_and_wrong_version_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            base=Path(tmp); bag=base/'ros2'; fixture2(bag)
            with self.assertRaises(ValueError): self.run_check(bag,bag/'out',ros_version=2)
            self.assertFalse((bag/'out').exists())
            r=self.run_check(bag,base/'wrong_version')
            self.assertFalse(r['metrics'][0]['通过'])
            self.assertIn('--ros-version 2',r['errors'][0])

    def test_pointcloud2_cdr_bag(self):
        with tempfile.TemporaryDirectory() as tmp:
            base=Path(tmp); bag=base/'pc2'; fixture2(bag,pointcloud2=True)
            r=self.run_check(bag,base/'out',ros_version=2)
            self.assertTrue(r['all_selected_checks_passed'],r['errors'])

    def test_pointcloud2_padding_endian_and_bad_layout(self):
        from types import SimpleNamespace as N
        fields=[N(name=n,offset=j*4,datatype=7,count=1) for j,n in enumerate(('x','y','z'))]
        data=struct.pack('>6f',1,2,3,4,5,6)+b'pad!'+struct.pack('>6f',7,8,9,10,11,12)+b'pad!'
        m=N(height=2,width=2,point_step=12,row_step=28,fields=fields,is_bigendian=True,data=data)
        x,y,z=pointcloud2_xyz(m)
        np.testing.assert_equal(x,[[1,4],[7,10]])
        np.testing.assert_equal(z,[[3,6],[9,12]])
        m.row_step=24
        with self.assertRaises(ValueError): pointcloud2_xyz(m)


if __name__=='__main__': unittest.main()
