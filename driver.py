# the actual fuzzy logic

import skfuzzy as fuzz
import numpy as np
from skfuzzy import control as ctrl
from config import *

MAX_STEER_DEG = int(np.rad2deg(MAX_STEER))
MAX_ACC = 2

left = ctrl.Antecedent(np.arange(0, MAX_DIST, 1), 'left')
front = ctrl.Antecedent(np.arange(0, MAX_DIST, 1), 'front')
right = ctrl.Antecedent(np.arange(0, MAX_DIST, 1), 'right')
vel = ctrl.Antecedent(np.arange(MAX_REV_SPEED, MAX_SPEED, 0.01), 'vel')
ang_vel = ctrl.Antecedent(np.arange(-MAX_STEER_DEG, MAX_STEER_DEG, 0.01), 'ang_vel')
acc = ctrl.Consequent(np.arange(-MAX_ACC, MAX_ACC, 0.01), 'acc')
ang_acc = ctrl.Consequent(np.arange(-1, 1, 0.01), 'ang_acc')

for dist in [left, front, right]:
    dist['close'] = fuzz.trimf(dist.universe, [0, 0, 80])
    dist['medium'] = fuzz.gaussmf(dist.universe, 100, 15)
    dist['far'] = fuzz.trapmf(dist.universe, [100, 170, MAX_DIST, MAX_DIST])

vel['back'] = fuzz.trimf(vel.universe, [MAX_REV_SPEED, MAX_REV_SPEED, 0])
vel['stop'] = fuzz.trimf(vel.universe, [-0.01, 0, 0.01])
vel['slow'] = fuzz.trimf(vel.universe, [0, 1, 2.5])
vel['medium'] = fuzz.trimf(vel.universe, [1, 3, 4])
vel['fast'] = fuzz.trapmf(vel.universe, [2.5, 6, 7, 7])

ang_vel['left'] = fuzz.gaussmf(ang_vel.universe, -MAX_STEER_DEG, 5)
ang_vel['none'] = fuzz.trimf(ang_vel.universe, [-MAX_STEER_DEG * 0.2, 0, MAX_STEER_DEG * 0.2])
ang_vel['right'] = fuzz.gaussmf(ang_vel.universe, MAX_STEER_DEG, 5)

acc['back'] = fuzz.trimf(acc.universe, [-2, -2, 0])
acc['none'] = fuzz.trimf(acc.universe, [-0.5, 0, 0.5])
acc['light_front'] = fuzz.trimf(acc.universe, [0, 1, 2])
acc['front'] = fuzz.trimf(acc.universe, [1, 2, 2])

ang_acc['left'] = fuzz.gaussmf(ang_acc.universe, -1, 0.125)
ang_acc['light_left'] = fuzz.gaussmf(ang_acc.universe, -0.5, 0.125)
ang_acc['none'] = fuzz.gaussmf(ang_acc.universe, 0, 0.125)
ang_acc['light_right'] = fuzz.gaussmf(ang_acc.universe, 0.5, 0.125)
ang_acc['right'] = fuzz.gaussmf(ang_acc.universe, 1, 0.125)

# rules
rules = [
    # if going forwards and straight and no obstacles accelerate
    ctrl.Rule(~vel['back'] & ang_vel['none'] & front['far'], acc['front']),

    # if x wall is closer than y wall, turn to the y wall
    ctrl.Rule(left['close'] & ~right['close'] & ~front['close'], ang_acc['right']),
    ctrl.Rule(right['close'] & ~left['close'] & ~front['close'], ang_acc['left']),
    ctrl.Rule(~left['far'] & right['far'] & ~front['close'], ang_acc['light_right']),
    ctrl.Rule(~right['far'] & left['far'] & ~front['close'], ang_acc['light_left']),

    # if front is the best direction, go there
    ctrl.Rule(front['far'] & ang_vel['none'], ang_acc['none']),
    ctrl.Rule(front['far'] & ang_vel['left'], ang_acc['light_right']),
    ctrl.Rule(front['far'] & ang_vel['right'], ang_acc['light_left']),

    # turn if front is getting close
    ctrl.Rule(front['close'] & right['close'] & ~left['close'], ang_acc['left']),
    ctrl.Rule(vel['stop'] & front['close'] & right['close'] & ~left['close'], acc['light_front']),
    ctrl.Rule(~vel['back'] & ~vel['stop'] & front['close'] & right['close'] & ~left['close'], acc['none']),
    ctrl.Rule(front['close'] & ~right['close'] & left['close'], ang_acc['right']),
    ctrl.Rule(vel['stop'] & front['close'] & ~right['close'] & left['close'], acc['light_front']),
    ctrl.Rule(~vel['back'] & ~vel['stop'] & front['close'] & ~right['close'] & left['close'], acc['none']),

    # medium turning
    ctrl.Rule(front['medium'] & ~left['far'] & ~right['far'] & ang_vel['none'], ang_acc['none']),
    ctrl.Rule(front['medium'] & ~left['far'] & ~right['far'] & ang_vel['left'], ang_acc['light_right']),
    ctrl.Rule(front['medium'] & ~left['far'] & ~right['far'] & ang_vel['right'], ang_acc['light_left']),

    # far turning
    ctrl.Rule(~front['far'] & right['far'] & ~left['far'], ang_acc['light_right']),
    ctrl.Rule(~front['far'] & left['far'] & ~right['far'], ang_acc['light_left']),
    ctrl.Rule(~front['far'] & left['far'] & right['far'], ang_acc['light_right']),  # prefer right

    ctrl.Rule(ang_vel['left'] & ~vel['back'] & left['close'], ang_acc['light_right']),
    ctrl.Rule(ang_vel['right'] & ~vel['back'] & right['close'], ang_acc['light_left']),

    # slow down when turning
    ctrl.Rule(vel['fast'] & ~ang_vel['none'], acc['none']),
    # but not completely
    ctrl.Rule(vel['slow'] & front['far'], acc['front']),
    ctrl.Rule(vel['slow'] & front['close'], acc['none']),
    ctrl.Rule(vel['medium'] & ~front['far'], acc['none']),
    ctrl.Rule(vel['fast'] & ~front['far'], acc['none']),

    # if empty road ahead, stop turning
    ctrl.Rule(~front['close'] & ang_vel['left'], ang_acc['light_right']),
    ctrl.Rule(~front['close'] & ang_vel['right'], ang_acc['light_left']),

    # if near wall, go back while turning
    ctrl.Rule(front['close'] & right['close'] & left['close'], acc['back']),
    ctrl.Rule(front['close'] & right['close'] & left['close'] & ang_vel['left'], ang_acc['right']),
    ctrl.Rule(front['close'] & right['close'] & left['close'] & ang_vel['right'], ang_acc['left']),

    # if going back and all dist medium, go back further
    ctrl.Rule(vel['back'] & front['medium'] & left['medium'] & right['medium'], acc['back']),
    ctrl.Rule(vel['back'] & front['medium'] & right['medium'] & left['medium'] & ang_vel['left'], ang_acc['right']),
    ctrl.Rule(vel['back'] & front['medium'] & right['medium'] & left['medium'] & ang_vel['right'], ang_acc['left']),

    # never stop yo
    ctrl.Rule(vel['stop'], acc['light_front'])
]

# controls
driving_ctrl = ctrl.ControlSystem(rules)
driver = ctrl.ControlSystemSimulation(driving_ctrl)
