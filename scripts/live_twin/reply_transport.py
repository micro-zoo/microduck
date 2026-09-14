"""Complete unicast transactions on a validated reply instead of a fixed delay.

The installed transport still owns device locking, exclusivity, termios and packet
validation. Full response deadlines remain unchanged. A short quiet interval also
collects duplicate/trailing responses so callers retain their ID/count checks.
"""
import os,select,stat,struct,time,types

QUIET_SECONDS=.002

def serial_owners(device,current_pid,proc='/proc'):
    owners=set()
    with os.scandir(proc) as processes:
        for process in processes:
            if not process.name.isdigit() or int(process.name)==current_pid:continue
            try:
                with os.scandir(process.path+'/fd') as entries:
                    for entry in entries:
                        try:
                            info=entry.stat(follow_symlinks=True)
                            if stat.S_ISCHR(info.st_mode) and info.st_rdev==device:owners.add(int(process.name))
                        except (FileNotFoundError,ProcessLookupError):continue
            except (FileNotFoundError,ProcessLookupError):continue
    return owners

def receive(protocol,fd,outgoing,timeout):
    parser=protocol.FrameParser();frames=[];deadline=time.monotonic()+timeout;quiet=None
    while True:
        end=min(deadline,quiet) if quiet is not None else deadline
        remaining=end-time.monotonic()
        if remaining<=0 or not select.select([fd],[],[],remaining)[0]:break
        try:chunk=os.read(fd,4096)
        except BlockingIOError:continue
        if not chunk:raise protocol.CommunicationError('串口连接断开')
        frames.extend(frame for frame in parser.feed(chunk) if frame.raw!=outgoing)
        # A partial trailing frame must retain the full timeout and fail finish().
        quiet=time.monotonic()+QUIET_SECONDS if frames and not parser.buffer else None
    parser.finish()
    return frames

def accelerated(protocol):
    class ReplyPort(protocol.LinuxPort):
        def assert_free(self):
            # Same complete /proc check as the maintenance transport. DirEntry
            # avoids constructing several Path objects for each descriptor.
            try:owners=serial_owners(self.rdev,os.getpid())
            except PermissionError as error:
                raise protocol.SafetyError('无法检查全部串口使用者，拒绝继续') from error
            if owners:raise protocol.SafetyError(f'串口仍被其它进程使用，PID={sorted(owners)}；不会终止它们')
        def exchange(self,outgoing,timeout):
            # Discovery/broadcast reads have multiple responders and keep the
            # original bounded collection window. Pose setup uses unicast only.
            if len(outgoing)<8 or outgoing[4]>=253:return super().exchange(outgoing,timeout)
            try:
                self.termios.tcflush(self.fd,self.termios.TCIFLUSH)
                deadline=time.monotonic()+.5;offset=0
                while offset<len(outgoing):
                    remaining=deadline-time.monotonic()
                    if remaining<=0 or not select.select([],[self.fd],[],remaining)[1]:
                        raise protocol.CommunicationError('串口发送超时，设备可能收到部分帧')
                    try:count=os.write(self.fd,outgoing[offset:])
                    except BlockingIOError:continue
                    if not count:raise protocol.CommunicationError('串口未接受数据')
                    offset+=count
                while struct.unpack('I',self.fcntl.ioctl(self.fd,self.termios.TIOCOUTQ,struct.pack('I',0)))[0]:
                    if time.monotonic()>=deadline:raise protocol.CommunicationError('UART 发送队列超时')
                    time.sleep(.002)
                return receive(protocol,self.fd,outgoing,timeout)
            except (OSError,self.termios.error) as error:
                raise protocol.CommunicationError(f'串口 I/O 失败：{error}') from error
    return types.SimpleNamespace(**{**vars(protocol),'LinuxPort':ReplyPort})
