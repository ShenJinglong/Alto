# -*- coding: utf-8 -*-

from multiprocessing import Process
import cv2
import numpy as np
import time
import math
import global_params

feature_params = dict( maxCorners = 2,
                       qualityLevel = 0.5,
                       minDistance = 7,
                       blockSize = 7 )

lk_params = dict( winSize = (11, 11),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
                  minEigThreshold = 1e-4 )

class OpticalFlow(Process):
    def __init__(self, frameQueue, flowXY):
        super(OpticalFlow, self).__init__()
        self.frameQueue = frameQueue
        self.flowXY = flowXY

    def run(self):
        track_len = 5
        detect_interval = 5
        tracks = []
        frame_idx = 0
        fps_counter = 0
        start_time = time.time()
        fps = 0
        filter_counter = 0
        FlowX = 0
        FlowY = 0
        while True:
            if not self.frameQueue.empty():
                frame = self.frameQueue.get()
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                vis = frame.copy()

                if len(tracks) > 0:
                    img0, img1 = prev_gray, frame_gray
                    p0 = np.float32([tr[-1] for tr in tracks]).reshape(-1, 1, 2)
                    p1, _st, _err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
                    p0r, _st, _err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
                    d = abs(p0 - p0r).reshape(-1, 2).max(-1)
                    good = d < 1
                    new_tracks = []
                    for tr, (x, y), good_flag in zip(tracks, p1.reshape(-1, 2), good):
                        if not good_flag:
                            continue
                        tr.append((x, y))
                        if len(tr) > track_len:
                            del tr[0]
                        new_tracks.append(tr)
                        cv2.circle(vis, (x, y), 2, (0, 255, 0), -1)
                    tracks = new_tracks
                    cnt = len(tracks)
                    speed = np.zeros((cnt, 3))
                    for i in range(0, len(tracks)):
                        try:
                            speed[i][0] = tracks[i][1][0] - tracks[i][0][0]
                            speed[i][1] = tracks[i][1][1] - tracks[i][0][1]
                            speed[i][2] = math.sqrt(speed[i][0] * speed[i][0] + speed[i][1] * speed[i][1])
                        except:
                            error = 1
                    speed = speed[np.lexsort(-speed.T)]
                    if fps != None and len(tracks) != 0:
                        if len(tracks) > 3:
                            for i in range(0, len(tracks) - 1):
                                try:
                                    if speed[i][2] * 0.9 >= speed[i + 2][2]:
                                        filter_counter = filter_counter + 1
                                    else:
                                        break
                                except:
                                    error = 2
                            speed = speed[speed[:, 2] <= speed[filter_counter, 2]]
                            FlowX = int(speed[0, 0] * fps * 10)
                            FlowY = int(speed[0, 1] * fps * 10)
                            filter_counter = 0
                        else:
                            FlowX = int(np.mean(speed[:, 0]) * fps * 10)
                            FlowY = int(np.mean(speed[:, 1]) * fps * 10)
                    cv2.polylines(vis, [np.int32(tr) for tr in tracks], False, (0, 255, 0))

                if frame_idx % detect_interval == 0:
                    mask = np.zeros_like(frame_gray)
                    mask[:] = 255
                    for x, y in [np.int32(tr[-1]) for tr in tracks]:
                        cv2.circle(mask, (x, y), 5, 0, -1)
                    p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
                    if p is not None:
                        for x, y in np.float32(p).reshape(-1, 2):
                            tracks.append([(x, y)])
                        if len(tracks) > 20:
                            tracks = np.delete(tracks, [0, 1], axis = 0)
                
                frame_idx += 1
                prev_gray = frame_gray
                
                if not global_params.RASPBERRY_MODE:
                    cv2.imshow('Flow', vis)
                
                fps_counter += 1
                if time.time() - start_time > 1:
                    fps = fps_counter / (time.time() - start_time)
                    # print('FPS: %.1f'%(fps))
                    fps_counter = 0
                    start_time = time.time()

                cv2.waitKey(1)
        
                try:
                    self.flowXY.put((FlowX, FlowY), False)
                except:
                    continue

