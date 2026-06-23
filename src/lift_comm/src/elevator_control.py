#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
elevator_control.py

与电梯接口装置的 RS485 通信模块（SMEC协议）
ID号 = 机器人的固定编号（Robot ID）
目前来说就是机器人的ID是固定的，而且电梯群号和群内电梯号也都是固定的，前后门打开也都是固定的。
导航目前也只有1楼~3楼的跨层往返。
"""

import serial
import threading
import time
import rospy
from typing import Optional, Tuple

STX = bytes([0xAA, 0x55])
ETX = bytes([0x03])

# 命令字
CMD_QUERY_STATUS     = 0x02
CMD_REGISTER_FLOOR   = 0x03
CMD_DOOR_CONTROL     = 0x05
CMD_HEARTBEAT        = 0x06

RESP_QUERY_STATUS    = 0x82
RESP_REGISTER_FLOOR  = 0x83
RESP_DOOR_CONTROL    = 0x85
RESP_HEARTBEAT       = 0x86


class ElevatorControl:
    def __init__(self,
                 port: str = "/dev/ttyUSB0", # ← 根据实际情况修改串口名称
                 baudrate: int = 9600,
                 default_bnk: int = 1,
                 default_nod: int = 1,
                 robot_id: int = 1,          # ← 新增：机器人固定编号（0~255）
                 timeout: float = 0.5):
        
        self.current_bnk = default_bnk & 0xFF
        self.current_nod = default_nod & 0xFF
        self.robot_id = robot_id & 0xFF     # ← 固定不变的机器人ID
        
        rospy.loginfo(f"机器人ID已设置为: 0x{self.robot_id:02X}")

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout
            )
            rospy.loginfo(f"电梯串口打开成功: {port}")
        except Exception as e:
            rospy.logerr(f"串口打开失败: {e}")
            raise

        self._lock = threading.RLock()
        self._running = True
        self._last_heartbeat = time.monotonic()

        # 心跳线程（暂时关闭，需要时取消注释）
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()

        self.last_status = None

    def set_target_elevator(self, bnk: int, nod: int):
        """切换要控制的电梯（多梯场景使用）"""
        self.current_bnk = bnk & 0xFF
        self.current_nod = nod & 0xFF
        rospy.loginfo(f"已切换目标电梯 → 群号:{bnk}  梯号:{nod}")

    def _calc_checksum(self, data: bytes) -> int:
        """计算校验和"""
        s = self.current_bnk + self.current_nod + len(data)
        for b in data:
            s += b
        s &= 0xFF
        return (~s + 1) & 0xFF

    def _build_packet(self, cmd_data: bytes) -> bytes:
        """构建数据包"""
        length = len(cmd_data)
        payload = bytes([self.current_bnk, self.current_nod, length]) + cmd_data
        checksum = self._calc_checksum(cmd_data)
        return STX + payload + bytes([checksum]) + ETX

    def _send_and_recv(self, cmd_data: bytes, expect_resp_cmd: int, timeout: float = 13.0) -> Optional[bytes]:
        packet = self._build_packet(cmd_data)

        with self._lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(packet)
                rospy.loginfo(f"【485】发送报文: {packet.hex()}")

                start = time.monotonic()
                resp = bytearray()

                while time.monotonic() - start < timeout:
                    if self.ser.in_waiting:
                        new_data = self.ser.read(self.ser.in_waiting)
                        resp.extend(new_data)

                    if len(resp) >= 8 and resp.startswith(STX) and resp.endswith(ETX):
                        data_len = resp[4]
                        data_start = 5

                        expected_len = 2 + 1 + 1 + 1 + data_len + 1 + 1
                        # = 头2 + 群号1 + 电梯号1 + len1 + data + checksum1 + 尾1

                        if len(resp) == expected_len:

                            rospy.loginfo(f"【485】收到完整报文: {resp.hex()} (len={len(resp)})")

                            checksum = resp[data_start + data_len]#这个地方不需要-1了，因为data_len已经是data的长度了，checksum就在data的后面一个字节
                            calc = self._calc_checksum(resp[data_start:data_start + data_len])

                            if checksum == calc:

                                # 命令校验（data第一个字节）
                                cmd = resp[data_start]

                                #机器人ID校验
                                recv_robot_id = resp[data_start + 1]

                                rospy.loginfo(f"【485】解析: cmd=0x{cmd:02X}(期望0x{expect_resp_cmd:02X}) id=0x{recv_robot_id:02X}(期望0x{self.robot_id:02X})")

                                if cmd == expect_resp_cmd and recv_robot_id == self.robot_id:
                                    
                                    return bytes(resp[data_start:data_start + data_len])
                                
                                else:
                                    rospy.logwarn(f"【485】命令或ID不匹配，丢弃该报文")
                                    resp.clear()

                            else:
                                rospy.logwarn(f"【485】校验和不匹配: 收到=0x{checksum:02X} 计算=0x{calc:02X}")
                                resp.clear()
                        elif len(resp) > expected_len:
                            rospy.logwarn(f"【485】报文过长: {resp.hex()} (len={len(resp)}, 期望={expected_len})")
                            resp.clear()

                    if len(resp) > 0 and time.monotonic() - start > timeout * 0.5 and len(resp) < 8:
                        rospy.loginfo(f"【485】当前缓冲区({len(resp)}字节): {resp.hex()}")

                rospy.logwarn(f"【485】电梯响应超时，最终缓冲区({len(resp)}字节): {resp.hex()}")
                return None

            except Exception as e:
                rospy.logerr(f"串口异常: {e}")
                return None

    # ====================== 对外接口 ======================

    def _parse_elevator_status(self, resp: bytes) -> Optional[Tuple[int, bool, bool, bool, bool, bool, bool]]:
        """解析电梯返回的状态信息"""
        if not resp or len(resp) < 5:
            return None

        # 第4个字节后7位是楼层
        floor_code = resp[3] & 0x7F

        # 第5个字节后6位是各种状态
        flags = resp[4] & 0x3F

        return (
            floor_code,
            bool((flags & 0x20) >> 5),  # 电梯运行状态
            bool((flags & 0x10) >> 4),  # 后门开门到位
            bool((flags & 0x08) >> 3),  # 前门开门到位
            bool((flags & 0x04) >> 2),  # 通信状态
            bool((flags & 0x02) >> 1),  # 下行服务方向
            bool(flags & 0x01),         # 上行服务方向
        )

    def query_status(self) -> Optional[Tuple[int, bool, bool, bool, bool, bool, bool]]:
        """查询电梯状态"""
        data = bytes([CMD_QUERY_STATUS, self.robot_id, 0xFF])   # ← 使用固定robot_id
        resp = self._send_and_recv(data, RESP_QUERY_STATUS)
        return self._parse_elevator_status(resp)


    def register_floor(self, floor_actual: int, front: bool = True) -> bool:
        """机器人登记楼层"""
        door = 0 if front else 1
        data = bytes([CMD_REGISTER_FLOOR, self.robot_id, 0x00, (door << 7) | (floor_actual & 0x7F)])
        resp = self._send_and_recv(data, RESP_REGISTER_FLOOR)
        if resp:
            rospy.loginfo(f"【485】收到完整报文: {resp.hex()} (len={len(resp)})")
        return bool(resp and len(resp) >= 3 and (resp[1] & 0x01))


    def control_front_door(self, open: bool) -> Optional[Tuple[int, bool, bool, bool, bool, bool, bool]]:
        """控制前门开关"""
        bits = 0
        if open:
            bits |= 0x10  # D4 前门开门
        else:
            bits |= 0x40  # D6 前门关门
        data = bytes([CMD_DOOR_CONTROL, self.robot_id,bits])
        resp=self._send_and_recv(data, RESP_DOOR_CONTROL)
        return self._parse_elevator_status(resp)
    

    def control_rear_door(self, open: bool) -> Optional[Tuple[int, bool, bool, bool, bool, bool, bool]]:
        """控制后门开关"""
        bits = 0
        if open:
            bits |= 0x20  # D5 后门开门
        else:
            bits |= 0x80  # D7 后门关门
        data = bytes([CMD_DOOR_CONTROL, self.robot_id,bits])
        resp=self._send_and_recv(data, RESP_DOOR_CONTROL)
        return self._parse_elevator_status(resp)


    def heartbeat(self) -> bool:
        """和电梯间的心跳报文（保持通信链路活跃）"""
        data = bytes([CMD_HEARTBEAT, self.robot_id, 0x00])
        return self._send_and_recv(data, RESP_HEARTBEAT, timeout=3) is not None

    def _heartbeat_loop(self):
        while self._running:
            if time.monotonic() - self._last_heartbeat >= 100.0:
                if self.heartbeat():
                    self._last_heartbeat = time.monotonic()
                else:
                    rospy.logwarn_throttle(10, "电梯心跳失败")
            time.sleep(1.0)

    def close(self):
        self._running = False
        if self.ser.is_open:
            self.ser.close()
            rospy.loginfo("电梯串口已关闭")


# 测试用（调试时直接运行此文件）
if __name__ == "__main__":
    rospy.init_node("elevator_test")
    ec = ElevatorControl(
        robot_id=0x01,           # ← 这里改成你们分配的机器人ID（默认0也行）
        default_bnk=0,
        default_nod=0
    )

    try:
        while not rospy.is_shutdown():
            status = ec.query_status()
            if status:
                f, run, ro, fo, comm, _, _ = status
                print(f"楼层码:0x{f:02X}  前门:{fo}  后门:{ro}  运行:{run}")
            time.sleep(2)
    finally:
        ec.close()