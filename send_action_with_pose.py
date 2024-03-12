"""
体のポーズで蛇を操作するプログラム
"""

from math import cos, sin, acos, asin, atan, pi
import numpy as np

import cv2
import mediapipe as mp
import math

import time

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
    time_interval=0.1 #実機と同期させる
    t_now = time.perf_counter()
    t_pre = time.perf_counter()
    t_switch=time.perf_counter() #スイッチした時間
    RCOS,LCOS=0,0
    is_forward=True
    #################################################

    ## initialize pose estimator #################################################
    mp_drawing = mp.solutions.drawing_utils
    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
    cap = cv2.VideoCapture(0) # input image from camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 900)
    ##############################################################################

    while cap.isOpened():

        #>> 同期を取る >>
        t_now=time.perf_counter()
        if (t_now-t_pre<time_interval):
            continue
        t_pre=t_now
        #>> 同期を取る >>

        try:
            if time_step % update ==0: 
                ## mediapipe #######################################################################################
                _, frame = cap.read() # read frame
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # convert to RGB
                pose_results = pose.process(frame_rgb) # process the frame for pose detection
                mp_drawing.draw_landmarks(frame, pose_results.pose_landmarks, mp_pose.POSE_CONNECTIONS) # draw skeleton on the frame
                frame = cv2.flip(frame, 1) # Flip horizontal
                ####################################################################################################

                ## openCV #############################################################################################
                #text
                cv2.putText(frame,"enter text here",(10,60), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,0,0),3,cv2.LINE_AA)
                
                # display the frame
                cv2.namedWindow("Output", cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Output', 900, 500)
                cv2.imshow('Output', frame)
                #######################################################################################################

                ## Calculate human joint angle ###########################################################################
                if pose_results.pose_landmarks != None:
                    "------ change here -------------------------------------------------------------------------------------"
                    RShoulder = (pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].x, pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].y)
                    RElbow = (pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].x, pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].y)
                    RHip = (pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP].x, pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP].y) 
                    RSE = math.sqrt((RShoulder[0] - RElbow[0])*(RShoulder[0] - RElbow[0]) + (RShoulder[1] - RElbow[1])*(RShoulder[1] - RElbow[1]))
                    REH = math.sqrt((RElbow[0] - RHip[0])*(RElbow[0] - RHip[0]) + (RElbow[1] - RHip[1])*(RElbow[1] - RHip[1]))
                    RHS = math.sqrt((RHip[0] - RShoulder[0])*(RHip[0] - RShoulder[0]) + (RHip[1] - RShoulder[1])*(RHip[1] - RShoulder[1]))
                    RCOS = (RSE*RSE + RHS*RHS - REH*REH) / (2*RSE*RHS)
                    LShoulder = (pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].x, pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].y)
                    LElbow = (pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].x, pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].y)
                    LHip = (pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP].x, pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP].y)
                    LSE = math.sqrt((LShoulder[0] - LElbow[0])*(LShoulder[0] - LElbow[0]) + (LShoulder[1] - LElbow[1])*(LShoulder[1] - LElbow[1]))
                    LEH = math.sqrt((LElbow[0] - LHip[0])*(LElbow[0] - LHip[0]) + (LElbow[1] - LHip[1])*(LElbow[1] - LHip[1]))
                    LHS = math.sqrt((LHip[0] - LShoulder[0])*(LHip[0] - LShoulder[0]) + (LHip[1] - LShoulder[1])*(LHip[1] - LShoulder[1]))
                    LCOS = (LSE*LSE + LHS*LHS - LEH*LEH) / (2*LSE*LHS)
                    print('Rcos', RCOS,'    Lcos', LCOS)
                    "--------------------------------------------------------------------------------------------------------"
                else:
                    print("There are no landmark...")
                ##########################################################################################################

                ## controller #################################################
                "-------- change here ----------------------------------------"
                omega=3
                command_gain=0.3
                amplitude=0.7
                
                r_command,l_command=(-RCOS+1)*0.5,(-LCOS+1)*0.5 #取得したcosの範囲を0~1に変換する               
                if r_command>0.5 and l_command>0.5: #前後をスイッチする
                        if time.perf_counter()-t_switch>time_interval*10:
                            is_forward=not is_forward
                            t_switch=time.perf_counter()

                t = time_step*ctrl_time_step
                action = np.zeros(12) # joint angle

                for i in range(12):
                    # action[i]=command_gain*(r_command-l_command)+amplitude*np.sin(omega*(t/(1)) + (int(is_forward)-0.5)/abs(int(is_forward)-0.5)* np.pi/3.5*i)
                    action[i]=command_gain*(r_command-l_command)+amplitude*np.sin(omega*(t/(1)) + (int(is_forward)-0.5)/abs(int(is_forward)-0.5)* np.pi/4.5*i)
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

    cap.release()
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()