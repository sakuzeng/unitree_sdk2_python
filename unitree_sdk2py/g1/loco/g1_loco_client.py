import json

from ...rpc.client import Client
from .g1_loco_api import *

"""
" class SportClient
"""
class LocoClient(Client):
    def __init__(self):
        super().__init__(LOCO_SERVICE_NAME, False)
        self.first_shake_hand_stage_ = -1

    def Init(self):
        # set api version
        self._SetApiVerson(LOCO_API_VERSION)

        # regist api
        self._RegistApi(ROBOT_API_ID_LOCO_GET_FSM_ID, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_FSM_MODE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_BALANCE_MODE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_SWING_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_STAND_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_PHASE, 0) # deprecated

        self._RegistApi(ROBOT_API_ID_LOCO_SET_FSM_ID, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_BALANCE_MODE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_SWING_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_STAND_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_VELOCITY, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_ARM_TASK, 0)

    # 7101
    # 设置 FSM（有限状态机）ID，控制机器人运动模式。 
    def SetFsmId(self, fsm_id: int):
        p = {}
        p["data"] = fsm_id
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_FSM_ID, parameter)
        return code

    # 7102
    # 设置机器人平衡模式。
    def SetBalanceMode(self, balance_mode: int):
        p = {}
        p["data"] = balance_mode
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_BALANCE_MODE, parameter)
        return code

    # 7104
    # 设置站立高度。
    def SetStandHeight(self, stand_height: float):
        p = {}
        p["data"] = stand_height
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_STAND_HEIGHT, parameter)
        return code

    # 7105
    # 设置机器人速度（前进、侧移、旋转），可指定持续时间。
    def SetVelocity(self, vx: float, vy: float, omega: float, duration: float = 1.0):
        p = {}
        velocity = [vx,vy,omega]
        p["velocity"] = velocity
        p["duration"] = duration
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_VELOCITY, parameter)
        return code
    
    # 7106
    # 设置机械臂任务 ID
    def SetTaskId(self, task_id: float):
        p = {}
        p["data"] = task_id
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_ARM_TASK, parameter)
        return code
    
    # 7001
    # 获得FSM（有限状态机）ID
    def GetFsmId(self):
        code, data = self._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, "")
        if code == 0:
            return data
        return None
    # 7002
    # 获得FSM（有限状态机）模式
    def GetFsmMode(self):
        code, data = self._Call(ROBOT_API_ID_LOCO_GET_FSM_MODE, "")
        if code == 0:
            return data
        return None
    # 7003
    # 获得平衡模式
    def GetBalanceMode(self):
        code, data = self._Call(ROBOT_API_ID_LOCO_GET_BALANCE_MODE, "")
        if code == 0:
            return data
        return None
    # 零力矩模式
    def ZeroTorque(self):
        self.SetFsmId(0)
    # 阻尼模式
    def Damp(self):
        self.SetFsmId(1)
    # 站立模式
    def StandUp(self):
        self.SetFsmId(4)
    # 主运控模式
    def Start(self):
        self.SetFsmId(200)
    # 蹲下到站立
    def Squat2StandUp(self):
        self.SetFsmId(706)
    # 躺下到站立
    def Lie2StandUp(self):
        self.SetFsmId(702)
    # 坐
    def Sit(self):
        self.SetFsmId(3)
    # 站立到蹲下
    def StandUp2Squat(self):
        self.SetFsmId(706)

    # 停止移动
    def StopMove(self):
        self.SetVelocity(0., 0., 0.)
    # 最高站立
    def HighStand(self):
        UINT32_MAX = (1 << 32) - 1
        self.SetStandHeight(UINT32_MAX)
    # 最低站立
    def LowStand(self):
        UINT32_MIN = 0
        self.SetStandHeight(UINT32_MIN)
    # 移动（前进、侧移、旋转），可选持续移动。
    def Move(self, vx: float, vy: float, vyaw: float, continous_move: bool = False):
        duration = 864000.0 if continous_move else 1
        self.SetVelocity(vx, vy, vyaw, duration)
    # 平衡站立。
    def BalanceStand(self, balance_mode: int):
        self.SetBalanceMode(balance_mode)
    # 挥手（可选转身）。
    def WaveHand(self, turn_flag: bool = False):
        self.SetTaskId(1 if turn_flag else 0)
    # 握手动作
    def ShakeHand(self, stage: int = -1):
        if stage == 0:
            self.first_shake_hand_stage_ = False
            self.SetTaskId(2)
        elif stage == 1:
            self.first_shake_hand_stage_ = True
            self.SetTaskId(3)
        else:
            self.first_shake_hand_stage_ = not self.first_shake_hand_stage_
            return self.SetTaskId(3 if self.first_shake_hand_stage_ else 2)
    