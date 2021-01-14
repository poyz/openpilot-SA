#!/usr/bin/env python3
import warnings

warnings.simplefilter(action='ignore', category=FutureWarning)  # for seaborn
import os
import pickle
import sys
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm  # type: ignore
from selfdrive.car.toyota.values import STEER_THRESHOLD
from scipy.signal import correlate
import seaborn as sns
from selfdrive.config import Conversions as CV

from common.realtime import DT_CTRL
from tools.lib.logreader import MultiLogIterator
from opendbc.can.parser import CANParser


MIN_SAMPLES = 0  # int(5 / DT_CTRL)
MAX_SAMPLES = np.inf  # int(15 / DT_CTRL)


def to_signed(n, bits):
  if n >= (1 << max((bits - 1), 0)):
    n = n - (1 << max(bits, 0))
  return n



BASEDIR = '/openpilot'
use_dir = '{}/pedal-exp/rlogs'.format(BASEDIR)  # change to a directory of rlogs
files = os.listdir(use_dir)
files = [f for f in files if '.ini' not in f]

lr = MultiLogIterator([os.path.join(use_dir, i) for i in files], wraparound=False)

data = [[]]
# engaged, steering_pressed = False, False
# torque_cmd, steer_angle = None, None
# yaw_rate = None
# v_ego = None

last_plan = None
last_car_state = None
last_controls_state = None

all_msgs = sorted(lr, key=lambda msg: msg.logMonoTime)
del lr

for msg in tqdm(all_msgs):
  if msg.which() == 'plan':
    last_plan = msg.plan
  elif msg.which() == 'carState':
    last_car_state = msg.carState
  elif msg.which() == 'controlsState':
    last_controls_state = msg.controlsState

  if None in [last_plan, last_car_state, last_controls_state]:
    continue

  if last_controls_state.enabled and msg.which() == 'carState':
    data[-1].append({'carstate': last_car_state, 'plan': last_plan, 'controlsstate': last_controls_state})
  elif len(data[-1]) and msg.which() == 'carState':
    data.append([])

del all_msgs

split = [sec for sec in data if MAX_SAMPLES > len(sec) > MIN_SAMPLES]  # long enough sections
del data
assert len(split) > 0, "Not enough valid sections of samples"

print(len(split))
print([len(i) / 100 for i in split])

data = split[3]
v_egos = [line['carstate'].vEgo for line in data]
v_cruises = [line['plan'].vCruise for line in data]
a_cruises = [line['plan'].aCruise for line in data]
v_pids = [line['controlsstate'].vPid for line in data]
a_targets = [line['controlsstate'].aTarget for line in data]
a_egos = [line['carstate'].aEgo for line in data]

pid_p = [line['controlsstate'].upAccelCmd for line in data]
pid_i = [line['controlsstate'].uiAccelCmd for line in data]
pid_f = [line['controlsstate'].ufAccelCmd for line in data]

plt.plot(v_egos, label='v_ego')
# plt.plot(v_cruises, label='v_cruise')
plt.plot(v_pids, label='v_pid')
# plt.plot(pid_p, label='PID p')
plt.legend()
plt.savefig('plots/v_egos.png')


plt.clf()
plt.plot(a_egos, label='a_ego')
plt.plot(a_cruises, label='a_cruise')
plt.plot(a_targets, label='aTarget')
# plt.plot(pid_f, label='PID f')
plt.legend()
plt.savefig('plots/a_egos.png')

plt.clf()
plt.plot(pid_p, label='PID p')
plt.plot(pid_i, label='PID i')
plt.legend()
plt.savefig('plots/pid.png')

# delays = []
# ptps = []
# seq_lens = []
# new_split = []
# mean_vels = []
# for idx, data in enumerate(split):
#   torque = np.array([line['torque_cmd'] for line in data])
#   angles = np.array([line['steer_angle'] for line in data])
#   if angles.std() == 0 or torque.std() == 0:
#     print('warning: angles or torque std is 0! skipping...')
#     continue
#   angles_ptp = abs(angles.ptp())  # todo: abs shouldn't be needed, but just in case
#   if angles_ptp <= 5:
#     print('angle range too low ({} <= 5)! skipping...'.format(angles_ptp))
#     continue
#   print('peak to peak: {}'.format(angles_ptp))
#
#   # diff = max(torque) / max(angles)  # todo: not sure which regularization method is better. normalization
#   # angles *= diff
#
#   Y1 = (angles - angles.mean()) / angles.std()  # todo: or standardization
#   Y2 = (torque - torque.mean()) / torque.std()
#
#   plt.clf()
#   plt.title('before offset')
#   plt.plot(angles, label='angle')
#   plt.plot(torque / (max(torque) / max(angles)), label='torque')
#   plt.legend()
#   plt.savefig('{}/steer_delay/plots/{}__before.png'.format(BASEDIR, idx))
#
#   # xcorr = correlate(Y1, Y2)[angles.size - 1:]  # indexing forces positive offset (torque always precedes angle)
#   xcorr = correlate(Y1, Y2)  # indexing forces positive offset (torque always precedes angle)
#   time_shift = np.arange(1 - angles.size, angles.size)[xcorr.argmax()]
#
#   if 0 < time_shift < 100:  # still plot them to debug todo: remove them
#     delays.append(time_shift)
#     ptps.append(angles_ptp)
#     seq_lens.append(len(data))
#     new_split.append(split)
#     mean_vels.append(np.mean([line['v_ego'] for line in data]))
#
#   print('len: {}'.format(len(data)))
#   print('time shift: {}'.format(time_shift))
#   print()
#
#   torque = np.roll(torque, time_shift)
#   plt.clf()
#   plt.title('after offset ({})'.format(time_shift))
#   plt.plot(angles, label='angle')
#   plt.plot(torque / (max(torque) / max(angles)), label='torque')
#   plt.legend()
#   plt.savefig('{}/steer_delay/plots/{}_after.png'.format(BASEDIR, idx))
#
# plt.clf()
# sns.distplot(delays, bins=40, label='delays')
# plt.legend()
# plt.savefig('{}/steer_delay/dist.png'.format(BASEDIR))
#
# with open('{}/steer_delay/data'.format(BASEDIR), 'wb') as f:
#   pickle.dump(new_split, f)
#
# print('mean vels:  {}'.format(np.round(np.array(mean_vels) * CV.MS_TO_MPH, 2).tolist()))  # todo: add vel list
# print('seq lens:   {}'.format(seq_lens))
# print('ptp angles: {}'.format(ptps))
# print('delays:     {}'.format(delays))
#
# print('\nmedian: {}'.format(np.median(delays)))
# print('mean: {}'.format(np.mean(delays)))

