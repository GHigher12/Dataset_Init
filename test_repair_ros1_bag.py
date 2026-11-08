import contextlib
import io
from pathlib import Path
import struct
import tempfile
import unittest
from rosbags.rosbag1 import Writer, Reader
from repair_ros1_bag import repair, read_record


def fixture(path,compression=None):
    writer=Writer(path)
    if compression: writer.set_compression(compression)
    with writer:
        c=writer.add_connection('/test','std_msgs/msg/String',msgdef='string data\n',md5sum='992ce8a1687cec8c8bd883ec73ca41d1')
        for i in range(3): writer.write(c,1000000000+i*100000000,struct.pack('<I',100)+bytes([65+i])*100)
    data=bytearray(path.read_bytes()); pos=data.index(b'index_pos=')+10
    index=struct.unpack_from('<Q',data,pos)[0]; struct.pack_into('<Q',data,pos,0)
    path.write_bytes(data[:index])


class RecoveryTests(unittest.TestCase):
    def run_repair(self,source,output):
        with contextlib.redirect_stdout(io.StringIO()): return repair(source,output)

    def test_unindexed_and_compressed_messages_preserved(self):
        for compression in (None,Writer.CompressionFormat.BZ2,Writer.CompressionFormat.LZ4):
            with self.subTest(compression=compression),tempfile.TemporaryDirectory() as tmp:
                p=Path(tmp); source=p/'source.bag'; output=p/'out.bag'; fixture(source,compression)
                before=source.read_bytes(); r=self.run_repair(source,output)
                self.assertTrue(r['verified'],r)
                self.assertTrue(r['source_unchanged'])
                self.assertEqual(r['recovered_messages'],3)
                self.assertEqual(source.read_bytes(),before)
                with Reader(output) as reader:
                    records=list(reader.messages())
                    self.assertEqual([t for _,t,_ in records],[1000000000,1100000000,1200000000])
                    self.assertEqual(bytes(records[-1][2]),struct.pack('<I',100)+b'C'*100)

    def test_truncated_chunk_keeps_only_complete_messages(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp); source=p/'source.bag'; fixture(source)
            with source.open('rb') as f:
                f.seek(13); read_record(f,1024*1024)
                _,_,payload,_,_=read_record(f,1024*1024)
                start=f.tell()-len(payload)
            inner=io.BytesIO(payload); read_record(inner,1024*1024)
            read_record(inner,1024*1024); read_record(inner,1024*1024)
            end=start+inner.tell()+5
            source.write_bytes(source.read_bytes()[:end])
            r=self.run_repair(source,p/'out.bag')
            self.assertEqual(r['recovered_messages'],2)
            self.assertTrue(r['verified'])
            self.assertTrue(r['issues'])

    def test_no_overwrite_or_source_replacement(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp); source=p/'source.bag'; fixture(source)
            with self.assertRaises(ValueError): self.run_repair(source,source)
            out=p/'out.bag'; out.write_bytes(b'keep')
            with self.assertRaises(FileExistsError): self.run_repair(source,out)
            self.assertEqual(out.read_bytes(),b'keep')

    def test_wrong_file_magic_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp); source=p/'bad.bag'; source.write_bytes(b'not a ros1 bag')
            with self.assertRaises(ValueError): self.run_repair(source,p/'out.bag')
            self.assertFalse((p/'out.bag').exists())


if __name__=='__main__': unittest.main()
