import os,socket,sys,tempfile,threading,time,types,unittest
from unittest.mock import patch
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from reply_transport import receive,serial_owners,accelerated

class Parser:
    def __init__(self):self.buffer=bytearray();self.invalid=False
    def feed(self,data):
        self.buffer.extend(data);out=[]
        while b'\n' in self.buffer:
            raw,rest=self.buffer.split(b'\n',1);self.buffer=bytearray(rest)
            if raw==b'bad':self.invalid=True
            else:out.append(types.SimpleNamespace(raw=bytes(raw)+b'\n'))
        return out
    def finish(self):
        if self.invalid or self.buffer:raise ValueError('invalid frame')

class ReplyTests(unittest.TestCase):
    def receive(self,parts,timeout=.12):
        a,b=socket.socketpair();a.setblocking(False)
        def send():
            for delay,data in parts:
                time.sleep(delay);b.sendall(data)
        thread=threading.Thread(target=send);thread.start();start=time.monotonic()
        try:return receive(types.SimpleNamespace(FrameParser=Parser,CommunicationError=ValueError),a.fileno(),b'command\n',timeout),time.monotonic()-start
        finally:thread.join();a.close();b.close()
    def test_complete_reply_finishes_early_and_echo_is_excluded(self):
        frames,elapsed=self.receive([(0,b'command\nreply\n')]);self.assertEqual([f.raw for f in frames],[b'reply\n']);self.assertLess(elapsed,.06)
    def test_delayed_fragmented_reply_retains_full_deadline(self):
        frames,elapsed=self.receive([(.06,b're'),(.01,b'ply\n')]);self.assertEqual(len(frames),1);self.assertGreater(elapsed,.06)
    def test_duplicate_replies_are_preserved_for_caller_rejection(self):
        frames,_=self.receive([(0,b'reply\n'),(.0002,b'duplicate\n')]);self.assertEqual(len(frames),2)
    def test_corrupt_or_partial_trailing_frame_is_not_silently_discarded(self):
        for data in (b'reply\nbad\n',b'reply\npartial'):
            with self.assertRaises(ValueError):self.receive([(0,data)])
    def test_full_owner_scan_follows_fd_links_and_excludes_only_self(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for pid in (101,202,303):(root/str(pid)/'fd').mkdir(parents=True)
            (root/'101/fd/4').symlink_to('/dev/null')
            (root/'202/fd/5').symlink_to('/dev/null')
            (root/'303/fd/6').symlink_to('/dev/zero')
            (root/'303/fd/7').symlink_to(root/'missing')
            self.assertEqual(serial_owners(os.stat('/dev/null').st_rdev,101,root),{202})
    def test_owner_scan_permission_failure_still_refuses_access(self):
        wrapped=accelerated(types.SimpleNamespace(LinuxPort=object,SafetyError=RuntimeError))
        port=wrapped.LinuxPort();port.rdev=1
        with patch('reply_transport.serial_owners',side_effect=PermissionError):
            with self.assertRaises(RuntimeError):port.assert_free()

    def test_missing_reply_uses_original_timeout(self):
        frames,elapsed=self.receive([],timeout=.03);self.assertEqual(frames,[]);self.assertGreaterEqual(elapsed,.025)

if __name__=='__main__':unittest.main()
