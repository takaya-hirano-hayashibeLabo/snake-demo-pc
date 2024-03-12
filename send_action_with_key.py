"""
キーボードで蛇を操作するプログラム
"""

from math import cos, sin, acos, asin, atan, pi
import numpy as np

import cv2
import mediapipe as mp
import math

import time

import keyboard

## register address ########################
from socket import socket,AF_INET,SOCK_DGRAM
from config import CLIENT   
HOST=""
PORT=5000
sock=socket(AF_INET,SOCK_DGRAM)
sock.bind((HOST,PORT))
############################################

def main():

    ## Setting ######################################
    ctrl_time_step = 0.05 #[s]
    update = 1 #update capture every 1 step
    time_step = 0 # initialize time step
    time_interval=0.05 #実機と同期させる
    t_now = time.perf_counter()
    t_pre = time.perf_counter()
    t_switch=time.perf_counter() #スイッチした時間
    RCOS,LCOS=0,0
    is_forward=True
    #################################################

    while True:

        #>> 同期を取る >>
        t_now=time.perf_counter()
        if (t_now-t_pre<time_interval):
            continue
        t_pre=t_now
        #>> 同期を取る >>

        try:
            if time_step % update ==0: 

                #>> キーボードによる操作 >>
                if keyboard.read_key()=="w":
                    RCOS=0
                    LCOS=0
                    is_forward=True
                elif keyboard.read_key()=="s":
                    RCOS,LCOS=0,0
                    is_forward=False
                elif keyboard.read_key()=="a":
                    RCOS=-1.0
                    LCOS=1.0
                elif keyboard.read_key()=="d":
                    RCOS=1.0
                    LCOS=-1.0
                elif keyboard.read_key()=="l":
                    break
                else:
                    time_step-=1
                #>> キーボードによる操作 >>
                

                ## controller #################################################
                "-------- change here ----------------------------------------"
                frequency=4 #周波数
                amplitude=0.6 #振幅
                wavelength=np.pi/4.5 #波長
                command_gain=0.2 #左右のターンのゲイン
                
                r_command,l_command=(-RCOS+1)*0.5,(-LCOS+1)*0.5 #取得したcosの範囲を0~1に変換する               

                t = time_step*ctrl_time_step
                action = np.zeros(12) # joint angle

                for i in range(12):
                    # action[i]=command_gain*(r_command-l_command)+amplitude*np.sin(omega*(t/(1)) + (int(is_forward)-0.5)/abs(int(is_forward)-0.5)* np.pi/3.5*i)
                    action[i]=command_gain*(r_command-l_command)+amplitude*np.sin(frequency*(t/(1)) + (int(is_forward)-0.5)/abs(int(is_forward)-0.5)* wavelength *i)
                "-------------------------------------------------------------"
                ###############################################################

            ## send action to real robot ###############################################################################
            msg=f"{action[0]},{action[1]},{action[2]},{action[3]},{action[4]},{action[5]},{action[6]},{action[7]},{action[8]},{action[9]},{action[10]},{action[11]}"
            sock.sendto(msg.encode(),(CLIENT,PORT))
            # print("msg",msg)
            ############################################################################################################

            time_step += 1

        except Exception as e:
            print(e)
            break
            
        if cv2.waitKey(1) == 27: #press esc to close
            break


if __name__=="__main__":
    main()