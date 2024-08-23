#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl
from receive_theta.msg import FaceInfo

class FaceTrackingNode(Node):
    def __init__(self):
        super().__init__('face_tracking_node')
        self.subscription = self.create_subscription(FaceInfo, '/face_info', self.listener_callback, 10)
        self.motion_publisher = self.create_publisher(MotionCtrl, 'diablo/MotionCmd', 10)
        self.ctrlMsgs = MotionCtrl()
        self.thr_params = {
            'alpha_up': 0.005,
            'alpha_pitch': 0.0,
            'up_threshold': 0.8,
            # NOTE:x：小 → 大，image：左 → 右
            'x_threshold_1': 40,            # 左右判稳/一级调速
            'x_threshold_2': 80,            # 二级调速
            # NOTE:y：小 → 大，image：上 → 下
            'y_threshold_1': -200,          # 站立区间下限，即低于该阈值则进入站立模式
            'y_threshold_2': -120,          # 站立区间上限，即高出该阈值则退出站立模式
            'pitch_threshold': -280,        
            'forward_threshold_1': 450,     # 二级调速（foward）
            'forward_threshold_2': 600,     # 一级调速（foward）
            'backward_threshold_1': 800,    # 一级调速（backward） 
            'backward_threshold_2': 950     # 二级调速（backward）
        }
        self.mot_params = {
            'forward_vel_1': 0.15,        
            'forward_vel_2': 0.52,
            'left_vel_1': 0.3,
            'left_vel_2': 0.55    
        }
        self.previous_up = 0.0
        self.previous_pitch = 0.0

    def generMsgs(self, forward=None, left=None, roll=None, up=None, pitch=None,
                  mode_mark=False, height_ctrl_mode=None, pitch_ctrl_mode=None,
                  roll_ctrl_mode=None, stand_mode=None, jump_mode=False, dance_mode=None):
        
        self.ctrlMsgs.mode_mark = mode_mark
        self.ctrlMsgs.mode.jump_mode = jump_mode

        if dance_mode is not None:
            self.ctrlMsgs.mode.split_mode = dance_mode
        if forward is not None:
            self.ctrlMsgs.value.forward = forward
        if left is not None:
            self.ctrlMsgs.value.left = left
        if pitch is not None:
            self.ctrlMsgs.value.pitch = pitch
        if roll is not None:
            self.ctrlMsgs.value.roll = roll
        if up is not None:
            self.ctrlMsgs.value.up = up
        if height_ctrl_mode is not None:
            self.ctrlMsgs.mode.height_ctrl_mode = height_ctrl_mode
        if pitch_ctrl_mode is not None:
            self.ctrlMsgs.mode.pitch_ctrl_mode = pitch_ctrl_mode
        if roll_ctrl_mode is not None:
            self.ctrlMsgs.mode.roll_ctrl_mode = roll_ctrl_mode
        if stand_mode is not None:
            self.ctrlMsgs.mode.stand_mode = stand_mode

        self.motion_publisher.publish(self.ctrlMsgs)

    def listener_callback(self, msg):
        x = msg.x
        y = msg.y
        w = msg.width
        h = msg.height

        forward, left, up, pitch = 0.0, 0.0, 0.0, 0.0
        mode_mark = False
        stand_mode = False
        pitch_ctrl_mode = False

        # 检查是否需要进入站立模式
        if y < self.thr_params['y_threshold_1']: 
            mode_mark = True    # 每次切换mode时，需要将mode_mark设置为true
            stand_mode = True
            pitch_ctrl_mode = False  # 初始不进入 pitch 控制模式
            up = 1.0
            self.generMsgs(up = up, mode_mark=mode_mark, stand_mode=stand_mode, pitch_ctrl_mode=pitch_ctrl_mode)
            mode_mark = False 
            '''
            # 线性映射 y 到 up (0 到 1)
            up_new = (y - self.thr_params['x_threshold_1']) / (-320 - self.thr_params['x_threshold_1'])
            up = self._apply_smoothing(up_new, 'up')

            # 检查是否进入 pitch 调整模式
            if up >= self.thr_params['up_threshold'] and y < self.thr_params['pitch_threshold']:
                mode_mark = True
                pitch_ctrl_mode = True
                self.generMsgs(mode_mark=mode_mark, stand_mode=stand_mode, pitch_ctrl_mode=pitch_ctrl_mode)
                mode_mark = False 
                
                # 线性映射 y 到 pitch (-0.5 到 0.5)
                pitch_new = (y - self.thr_params['pitch_threshold']) / (-320 - self.thr_params['pitch_threshold']) * -0.5
                pitch = self._apply_smoothing(pitch_new, 'pitch')
            else:
                pitch = 0.0 
        elif y > self.thr_params['y_threshold_2']:  # y_threshold_1 < y < y_threshold_2为不调整区域，避免反复站起
            mode_mark = True
            stand_mode = False
            pitch_ctrl_mode = False
            up, pitch = 0.0, 0.0
            self.generMsgs(up=up, pitch=pitch, mode_mark=mode_mark, stand_mode=stand_mode, pitch_ctrl_mode=pitch_ctrl_mode)
            mode_mark = False  
        '''
        # 当 mode_mark = False 时，才可以进行水平移动
        if not mode_mark:
            # 检查左右移动
            if abs(x) > self.thr_params['x_threshold_1']:
                if abs(x) > self.thr_params['x_threshold_2']:
                    left = -self.mot_params['left_vel_2'] if x > 0 else self.mot_params['left_vel_2']
                else:
                    left = -self.mot_params['left_vel_1'] if x > 0 else self.mot_params['left_vel_1']

            face_area = w * h
            # 检查前后移动
            if face_area < self.thr_params['forward_threshold_1']:
                forward = self.mot_params['forward_vel_2']
            elif face_area < self.thr_params['forward_threshold_2']:
                forward = self.mot_params['forward_vel_1']
            elif face_area > self.thr_params['backward_threshold_2']:
                forward = -self.mot_params['forward_vel_2']
            elif face_area > self.thr_params['backward_threshold_1']:
                forward = -self.mot_params['forward_vel_1']

        # 判为稳定
        if abs(x) <= self.thr_params['x_threshold_1'] and face_area >= self.thr_params['forward_threshold_2'] and face_area <= self.thr_params['backward_threshold_1']:
            forward = 0.0
            left = 0.0
            pitch = 0.0
            mode_mark = False

        self.generMsgs(forward=forward, left=left, up=up, pitch=pitch, mode_mark=mode_mark, stand_mode=stand_mode, pitch_ctrl_mode=pitch_ctrl_mode)

    def _apply_smoothing(self, new_value, param_type):
        previous_value = getattr(self, f'previous_{param_type}', 0.0)
        alpha = self.thr_params[f'alpha_{param_type}']
        smoothed_value = previous_value + alpha * (new_value - previous_value)
        setattr(self, f'previous_{param_type}', smoothed_value)
        return smoothed_value


def main(args=None):
    rclpy.init(args=args)
    face_tracking_node = FaceTrackingNode()
    rclpy.spin(face_tracking_node)
    face_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
