#!/usr/bin/env python3

from filterpy.kalman import KalmanFilter
import matplotlib.pyplot as plt
import numpy as np
import pdb
from sklearn.utils.linear_assignment_ import linear_assignment
import sys
import time

from transform_utils import convert_3dbox_to_8corner
from iou_utils import compute_iou_2d_bboxes

def angle_in_range(angle):
  '''
  Input angle: -2pi ~ 2pi
  Output angle: -pi ~ pi
  '''
  if angle > np.pi:
    angle -= 2 * np.pi
  if angle < -np.pi:
    angle += 2 * np.pi
  return angle

def diff_orientation_correction(det, trk):
  '''
  return the angle diff = det - trk
  if angle diff > 90 or < -90, rotate trk and update the angle diff
  '''
  diff = det - trk
  diff = angle_in_range(diff)
  if diff > np.pi / 2:
    diff -= np.pi
  if diff < -np.pi / 2:
    diff += np.pi
  diff = angle_in_range(diff)
  return diff


class KalmanBoxTracker(object):
  """
  This class represents the internel state of individual tracked objects observed as bbox.
  """
  count = 0
  def __init__(self, bbox3D, info, p, conf, ps=1):
    """
    Initialises a tracker using initial bounding box.
    """

    dt = 1e-1
    #define constant velocity model
    self.kf = KalmanFilter(dim_x=10, dim_z=7)
    self.kf.F = np.array([[1,0,0,0,0,0,0,dt,0,0],      # state transition matrix
                          [0,1,0,0,0,0,0,0,dt,0],
                          [0,0,1,0,0,0,0,0,0,dt],
                          [0,0,0,1,0,0,0,0,0,0],
                          [0,0,0,0,1,0,0,0,0,0],
                          [0,0,0,0,0,1,0,0,0,0],
                          [0,0,0,0,0,0,1,0,0,0],
                          [0,0,0,0,0,0,0,1,0,0],
                          [0,0,0,0,0,0,0,0,1,0],
                          [0,0,0,0,0,0,0,0,0,1]])

    self.kf.H = np.array([[1,0,0,0,0,0,0,0,0,0],      # measurement function,
                          [0,1,0,0,0,0,0,0,0,0],
                          [0,0,1,0,0,0,0,0,0,0],
                          [0,0,0,1,0,0,0,0,0,0],
                          [0,0,0,0,1,0,0,0,0,0],
                          [0,0,0,0,0,1,0,0,0,0],
                          [0,0,0,0,0,0,1,0,0,0]])

    self.kf.P = p

    # self.kf.Q[-1,-1] *= 0.01    # process uncertainty

    # Dynamics (1)

    self.kf.Q[7:,7:] *= 1e-2
    self.kf.Q[0:7, 0:7] *= 1e-2

    self.kf.x[:7] = bbox3D.reshape((7, 1))

    self.kf.R *= 0.1

    self.id = KalmanBoxTracker.count
    KalmanBoxTracker.count += 1

    self.history = []
    self.hits = 1           # number of total hits including the first detection
    self.hit_streak = 1     # number of continuing hit considering the first detection
    self.first_continuing_hit = 1
    self.still_first = True
    self.age = 0
    self.info = info        # other info

    # Test new
    self.w = conf
    self.ps = ps

  def update(self, bbox3D, info, conf):
    """
    Updates the state vector with observed bbox.
    """

    self.w += conf
    if self.w > 1:
      self.w = 1

    self.history = []
    self.hits += 1
    self.hit_streak += 1          # number of continuing hit

    if self.still_first:
      self.first_continuing_hit += 1      # number of continuing hit in the fist time

    ######################### orientation correction
    if self.kf.x[3] >= np.pi: self.kf.x[3] -= np.pi * 2    # make the theta still in the range
    if self.kf.x[3] < -np.pi: self.kf.x[3] += np.pi * 2

    new_theta = bbox3D[3]
    if new_theta >= np.pi: new_theta -= np.pi * 2    # make the theta still in the range
    if new_theta < -np.pi: new_theta += np.pi * 2
    bbox3D[3] = new_theta

    predicted_theta = self.kf.x[3]
    if abs(new_theta - predicted_theta) > np.pi / 2.0 and abs(new_theta - predicted_theta) < np.pi * 3 / 2.0:     # if the angle of two theta is not acute angle
      self.kf.x[3] += np.pi
      if self.kf.x[3] > np.pi: self.kf.x[3] -= np.pi * 2    # make the theta still in the range
      if self.kf.x[3] < -np.pi: self.kf.x[3] += np.pi * 2

    # now the angle is acute: < 90 or > 270, convert the case of > 270 to < 90
    if abs(new_theta - self.kf.x[3]) >= np.pi * 3 / 2.0:
      if new_theta > 0: self.kf.x[3] += np.pi * 2
      else: self.kf.x[3] -= np.pi * 2

    #########################

    self.kf.update(bbox3D)

    if self.kf.x[3] >= np.pi: self.kf.x[3] -= np.pi * 2    # make the theta still in the range
    if self.kf.x[3] < -np.pi: self.kf.x[3] += np.pi * 2

    self.info = info

  def predict(self):
    """
    Advances the state vector and returns the predicted bounding box estimate.
    """
    self.kf.predict()

    if self.kf.x[3] >= np.pi: self.kf.x[3] -= np.pi * 2
    if self.kf.x[3] < -np.pi: self.kf.x[3] += np.pi * 2

    self.age += 1

    self.history.append(self.kf.x)

    # Multiply by p_s
    self.w *= self.ps

    return self.history[-1]

  def get_state(self):
    """
    Returns the current bounding box estimate.
    """
    return self.kf.x[:7].reshape((7, ))

def associate_detections_to_trackers(
  detections,
  trackers,
  threshold=0.1,
  distance='iou',
  dets=None,
  trks=None,
  trks_S=None
):
# def associate_detections_to_trackers(detections,trackers,iou_threshold=0.01):     # ablation study
# def associate_detections_to_trackers(detections,trackers,iou_threshold=0.25):
  """
  Assigns detections to tracked object (both represented as bounding boxes)
  detections:  N x 8 x 3
  trackers:    M x 8 x 3
  Returns 3 lists of matches, unmatched_detections and unmatched_trackers
  """
  if(len(trackers)==0):
    return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,8,3),dtype=int)
  iou_matrix = np.zeros((len(detections),len(trackers)),dtype=np.float32)
  distance_matrix = np.zeros((len(detections),len(trackers)),dtype=np.float32)

  for d,det in enumerate(detections):
    for t,trk in enumerate(trackers):

      if distance=='m':

        S_inv = np.linalg.inv(trks_S[t]) # 7 x 7
        diff = np.expand_dims(dets[d] - trks[t], axis=1) # 7 x 1

        # manual reversed angle by 180 when diff > 90 or < -90 degree
        corrected_angle_diff = diff_orientation_correction(dets[d][3], trks[t][3])
        diff[3] = corrected_angle_diff
        distance_matrix[d, t] = np.sqrt(np.matmul(np.matmul(diff.T, S_inv), diff)[0][0])
        to_max_mask = distance_matrix > threshold
        distance_matrix[to_max_mask] = threshold + 1

      else:
        #print(f'On d={d}, t={t}')
        #iou_matrix[d,t] = iou3d(det,trk)[1] # try 2d iou instead             # det: 8 x 3, trk: 8 x 3
        iou_matrix[d,t] = compute_iou_2d_bboxes(det, trk)

        distance_matrix = -iou_matrix

  matched_indices = linear_assignment(distance_matrix)      # hungarian algorithm

  unmatched_detections = []
  for d,det in enumerate(detections):
    if(d not in matched_indices[:,0]):
      unmatched_detections.append(d)
  unmatched_trackers = []
  for t,trk in enumerate(trackers):
    if(t not in matched_indices[:,1]):
      unmatched_trackers.append(t)

  #print(iou_matrix)

  #filter out matched with low IOU
  matches = []
  for m in matched_indices:

    match = True

    if distance=='m':
      if distance_matrix[m[0],m[1]] > threshold:
        match = False

    elif distance=='iou':
      if(iou_matrix[m[0],m[1]]<threshold):
        match = False

    if not match:
      unmatched_detections.append(m[0])
      unmatched_trackers.append(m[1])
    else:
      matches.append(m.reshape(1,2))

  if(len(matches)==0):
    matches = np.empty((0,2),dtype=int)
  else:
    matches = np.concatenate(matches,axis=0)

  return matches, np.array(unmatched_detections), np.array(unmatched_trackers)


class AB3DMOT(object):
  def __init__(self, thr_estimate: float = 0.8, thr_prune: float = 0.1, ps: float = 0.9):

    self.trackers = []
    self.frame_count = 0

    self.ps = ps
    self.thr_estimate = thr_estimate
    self.thr_prune = 0.1

    # self.reorder = [3, 4, 5, 6, 2, 1, 0]
    # self.reorder_back = [6, 5, 4, 0, 1, 2, 3]

  def update(
    self,
    dets_all,
    match_distance = 'iou',
    match_threshold = 0.1,
    match_algorithm = 'h',
    p = None,
  ):
    """
    Params:
      dets_all: dict
        dets - a numpy array of detections in the format [[x,y,z,theta,l,w,h],[x,y,z,theta,l,w,h],...]
        info: a array of other info for each det
    Requires: this method must be called once for each frame even with empty detections.
    Returns the a similar array, where the last column is the object ID.
    NOTE: The number of objects returned may differ from the number of detections provided.
    """
    dets, info = dets_all['dets'], dets_all['info']         # dets: N x 7, float numpy array
    conf = dets_all['conf']

    # dets = dets[:, self.reorder]
    self.frame_count += 1

    trks = np.zeros((len(self.trackers),7))         # N x 7 , #get predicted locations from existing trackers.
    to_del = []
    ret = []

    for t,trk in enumerate(trks):
      pos = self.trackers[t].predict().reshape((-1, 1))
      trk[:] = [pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]]
      if(np.any(np.isnan(pos))):
        to_del.append(t)

    trks = np.ma.compress_rows(np.ma.masked_invalid(trks))

    for t in reversed(to_del):
      self.trackers.pop(t)

    dets_8corner = [convert_3dbox_to_8corner(det_tmp) for det_tmp in dets]

    if len(dets_8corner) > 0:
      dets_8corner = np.stack(dets_8corner, axis=0)
    else:
      dets_8corner = []

    trks_8corner = [convert_3dbox_to_8corner(trk_tmp) for trk_tmp in trks]

    if len(trks_8corner) > 0:
      trks_8corner = np.stack(trks_8corner, axis=0)

    # Intersection over union
    if match_distance == 'iou':
      matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(
        dets_8corner,
        trks_8corner,
        threshold=match_threshold,
        distance=match_distance
      )
    # Mahalanobis
    elif match_distance == 'm':

      trks_S = [np.matmul(np.matmul(tracker.kf.H, tracker.kf.P), tracker.kf.H.T)
        + tracker.kf.R for tracker in self.trackers]

      matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(
        dets_8corner,
        trks_8corner,
        threshold=match_threshold,
        distance=match_distance,
        dets = dets,
        trks = trks,
        trks_S = trks_S,
      )

    # update matched trackers with assigned detections
    for t,trk in enumerate(self.trackers):
      if t not in unmatched_trks:
        d = matched[np.where(matched[:,1]==t)[0],0]     # a list of index
        trk.update(dets[d,:][0], info[d, :][0], conf[d[0]])

    #create and initialise new trackers for unmatched detections
    for i in unmatched_dets:        # a scalar of index
        trk = KalmanBoxTracker(dets[i,:], info[i, :], p, conf[i], ps=self.ps)
        self.trackers.append(trk)

    i = len(self.trackers)

    for trk in reversed(self.trackers):
        d = trk.get_state()      # bbox location

        if (trk.w >= self.thr_estimate):
          ret.append(np.concatenate((d, [trk.id+1], trk.info)).reshape(1,-1))

        i -= 1

        if trk.w <= self.thr_prune:
          self.trackers.pop(i)

    if(len(ret)>0):
      return np.concatenate(ret)      # x, y, z, theta, l, w, h, ID, other info, confidence

    return np.empty((0,15))      
