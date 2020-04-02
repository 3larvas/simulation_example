#-*- coding:utf-8 -*-
import sys
import time
import serial
import threading
import queue

class Protocol(object):
    """\
    Protocol as used by the ReaderThread. This base class provides empty
    implementations of all methods.
    """

    def connection_made(self, transport):
        """Called when reader thread is started"""

    def data_received(self, data):
        """Called with snippets received from the serial port"""

    def connection_lost(self, exc):
        """\
        Called when the serial port is closed or the reader loop terminated
        otherwise.
        """
        if isinstance(exc, Exception):
            raise exc


class ReaderThread(threading.Thread):
    """\
    Implement a serial port read loop and dispatch to a Protocol instance (like
    the asyncio.Protocol) but do it with threads.
    Calls to close() will close the serial port but it is also possible to just
    stop() this thread and continue the serial port instance otherwise.
    """

    def __init__(self, serial_instance, protocol_factory):
        """\
        Initialize thread.
        Note that the serial_instance' timeout is set to one second!
        Other settings are not changed.
        """
        super(ReaderThread, self).__init__()
        self.daemon = True
        self.serial = serial_instance
        self.protocol_factory = protocol_factory
        self.alive = True
        self._lock = threading.Lock()
        self._connection_made = threading.Event()
        self.protocol = None

    def stop(self):
        """Stop the reader thread"""
        self.alive = False
        if hasattr(self.serial, 'cancel_read'):
            self.serial.cancel_read()
        self.join(2)

    def run(self):
        """Reader loop"""
        if not hasattr(self.serial, 'cancel_read'):
            self.serial.timeout = 1
        self.protocol = self.protocol_factory()
        try:
            self.protocol.connection_made(self)
        except Exception as e:
            self.alive = False
            self.protocol.connection_lost(e)
            self._connection_made.set()
            return
        error = None
        self._connection_made.set()
        while self.alive and self.serial.is_open:
            try:
                # read all that is there or wait for one byte (blocking)
                data = self.serial.read(self.serial.in_waiting or 1)
            except serial.SerialException as e:
                # probably some I/O problem such as disconnected USB serial
                # adapters -> exit
                error = e
                break
            else:
                if data:
                    # make a separated try-except for called used code
                    try:
                        self.protocol.data_received(data)
                    except Exception as e:
                        error = e
                        break
        self.alive = False
        self.protocol.connection_lost(error)
        self.protocol = None

    def write(self, data):
        """Thread safe writing (uses lock)"""
        self.serial.write(data)
        # with self._lock:
        #
        #     print(data)
            # self.serial.write(data)

    def close(self):
        """Close the serial port and exit reader thread (uses lock)"""
        # use the lock to let other threads finish writing
        with self._lock:
            # first stop reading, so that closing can be done on idle port
            self.stop()
            self.serial.close()

    def connect(self):
        """
        Wait until connection is set up and return the transport and protocol
        instances.
        """
        if self.alive:
            self._connection_made.wait()
            if not self.alive:
                raise RuntimeError('connection_lost already called')
            return (self, self.protocol)
        else:
            raise RuntimeError('already stopped')

    # - -  context manager, returns protocol

    def __enter__(self):
        """\
        Enter context handler. May raise RuntimeError in case the connection
        could not be created.
        """
        self.start()
        self._connection_made.wait()
        if not self.alive:
            raise RuntimeError('connection_lost already called')
        return self.protocol

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Leave context: close port"""
        self.close()


# 프로토콜
class rawProtocal(Protocol):
    # 연결 시작시 발생
    def connection_made(self, transport):
        self.transport = transport
        self.running = True

    # 연결 종료시 발생
    def connection_lost(self, exc):
        self.transport = None

    #데이터가 들어오면 이곳에서 처리함.
    def data_received(self, data):
        print('recv : ', data)

    # 데이터 보낼 때 함수
    def write(self,data):
        print('send : ', data)
        self.transport.write(data)

    # 종료 체크
    def isDone(self):
        return self.running

# 포트 설정 ############################## 값 변경하는 곳
PORT = 'COM2'
# 연결
ser = serial.serial_for_url(PORT, baudrate=115200, timeout=1)

# 쓰레드 시작
packet = bytearray()
packet.append(0x53) #msg_s #변경 X
packet.append(0x54) #msg_t #변경 X
packet.append(0x58) #msg_x #변경 X
packet.append(0x01) #msg_AorM #변경 X
packet.append(0x00) #msg_Estop
packet.append(0x00) #msg_gear
packet.append(0x00) #msg_speed_
packet.append(0x55) #msg_speed_
packet.append(0x00) #msg_steer_
packet.append(0x00) #msg_steer_
packet.append(0x01) #mag_break
packet.append(0x09) #msg_alive #변경 X
packet.append(0x0D) #msg_etx_0 #변경 X
packet.append(0x0A) #msg_etx_1 #변경 X

with ReaderThread(ser, rawProtocal) as p:
    while p.isDone():
        ser.write(packet)
        time.sleep(1)

