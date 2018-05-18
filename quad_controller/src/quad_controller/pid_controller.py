# Copyright (C) 2017 Udacity Inc.
# All Rights Reserved.

# Author: Brandon Kinman

from collections import deque

def clip(x, mn, mx):
    if x < mn:
        return mn
    elif x > mx:
        return mx
    return x

def lerp(a, b, w):
    return a*w + b*(1.0-w)

class LowPassDFilter(object):
    def __init__(self, a, x=0.0):
        self._a = a
        self._x = x
    def __call__(self, x):
        dx = x - self._x
        self._x = lerp(self._x, x, self._a)
        return dx
    def reset(self, x=0.0):
        self._x = x

class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 10):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._max_windup = max_windup

        self._umin = -50.0
        self._umax = 50.0

        self._net_err = 0.0
        self._prv_err = LowPassDFilter(a = 0.5)
        self._target = 0.0
        self._time = None

    def reset(self):
        self._kp = 0.0
        self._ki = 0.0
        self._kd = 0.0
        self._net_err = 0.0
        self._prv_err.reset()
        self._target = 0.0
        self._time = None

    def setTarget(self, target):
        self._target = target

    def setKP(self, kp):
        self._kp = kp

    def setKI(self, ki):
        self._ki = ki

    def setKD(self, kd):
        self._kd = kd

    def setMaxWindup(self, max_windup):
        self._max_windup = max_windup

    def update(self, measured_value, timestamp):
        if self._time is None:
            self._time = timestamp
            self._prv_err.reset(self._target - measured_value)
            return None, (0,0,0)
        dt = timestamp - self._time
        if dt <= 0:
            return None, (0,0,0)

        err = self._target - measured_value
        #print('err', err)
        self._time = timestamp
        self._net_err += err * dt
        self._net_err = clip(self._net_err, -self._max_windup, self._max_windup)

        #d_err = (err - self._prv_err) / dt
        d_err = self._prv_err(err) / dt
        #self._prv_err = err

        p = self._kp * err
        i = self._ki * self._net_err
        d = clip(self._kd * d_err, self._umin, self._umax)

        u = p+i+d
        #u = clip(u, self._umin, self._umax)
        return u, (p,i,d)
