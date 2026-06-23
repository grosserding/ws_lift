#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
lift_controller.py

梯控主节点 —— 接收导航的 3 种 srv 请求，通过 RS485 转发给电梯。
1. /lift/call           LiftCall         导航呼梯
2. /lift/state_inquiry  StateInquiry     导航查询电梯状态
3. /lift/hodor          Hodor            导航请求按开门键
"""

import rospy
from lift_comm.srv import LiftCall, LiftCallResponse
from lift_comm.srv import StateInquiry, StateInquiryResponse
from lift_comm.srv import Hodor, HodorResponse
from elevator_control import ElevatorControl


class LiftController:

    def __init__(self):
        rospy.init_node("lift_controller")

        self.elevator = ElevatorControl(
            port=rospy.get_param("~port", "/dev/ttyACM0"),
            baudrate=rospy.get_param("~baudrate", 9600),
            robot_id=rospy.get_param("~robot_id", 1),
            default_bnk=rospy.get_param("~bnk", 0),
            default_nod=rospy.get_param("~nod", 0)
        )
        rospy.loginfo("电梯通信模块初始化成功")

        rospy.Service("/lift/call", LiftCall, self.handle_call)
        rospy.Service("/lift/state_inquiry", StateInquiry, self.handle_state_inquiry)
        rospy.Service("/lift/hodor", Hodor, self.handle_hodor)

        rospy.loginfo("====== 梯控节点启动完成 ======")

    # ===============================
    # /lift/call
    # ===============================
    def handle_call(self, req):
        target_floor = req.target_floor
        actual_floor = target_floor - 1
        rospy.loginfo(f"收到呼梯请求: 导航楼层 {target_floor} → 实际发送 {actual_floor}")
        success = self.elevator.register_floor(actual_floor, front=True)
        if success:
            rospy.loginfo("【485】呼梯命令发送成功")
        else:
            rospy.logerr("【485】呼梯命令发送失败")

        return LiftCallResponse(success)

    # ===============================
    # /lift/state_inquiry
    # ===============================
    def handle_state_inquiry(self, req):
        status = self.elevator.query_status()

        if status is None:
            rospy.logwarn("【485】电梯状态查询失败")
            return StateInquiryResponse(
                door_open=False,
                current_floor=0,
                up_down_state=0
            )

        floor_code, running, rear_open, front_open, comm_ok, down_valid, up_valid = status

        door_open = front_open or rear_open

        if running:
            if up_valid and not down_valid:
                up_down_state = 1   # 上行
            elif down_valid and not up_valid:
                up_down_state = 2   # 下行
            else:
                up_down_state = 0   # 未知
        else:
            up_down_state = 3       # 静止

        rospy.loginfo(
            f"【485】电梯状态: 楼层=0x{floor_code:02X} 门开={door_open} 方向={up_down_state}"
        )

        return StateInquiryResponse(
            door_open=door_open,
            current_floor=floor_code,
            up_down_state=up_down_state
        )

    # ===============================
    # /lift/hodor
    # ===============================
    def handle_hodor(self, req):
        rospy.loginfo("收到开门请求")

        result = self.elevator.control_front_door(True)
        success = result is not None

        if success:
            rospy.loginfo("【485】开门命令发送成功")
        else:
            rospy.logerr("【485】开门命令发送失败")

        return HodorResponse(success)


if __name__ == "__main__":
    try:
        controller = LiftController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
