# car physics and ray-cast collision
import sys

import numpy as np
from math import pi as pi
from driver import driver
from config import *
import pygame


class Car:
    CASTED_RAYS = 3

    def __init__(self, start_pos, start_angle, color):
        self.acc = 0
        self.color = color
        self._speed = 0
        self._pos = list(start_pos)
        self._angle = start_angle
        self._ang_vel = pi / 100
        self.ang_acc = -pi / 1000
        self.FOV = math.pi / 1.9
        self.STEP_ANGLE = self.FOV / (Car.CASTED_RAYS - 1)

    def update(self, distances):
        dists = np.array(distances) ** (1 / 2)

        driver.input['left'] = dists[0]
        driver.input['front'] = dists[1]
        driver.input['right'] = dists[2]
        driver.input['vel'] = self._speed
        driver.input['ang_vel'] = np.rad2deg(self._ang_vel)

        # print(driver.input)
        try:
            driver.compute()
        except ValueError as e:
            print("Error:")
            print(str(e))
            print(f"on input:\n{driver.input}")
            sys.exit(1)

        self.acc = driver.output['acc']
        self.ang_acc = np.deg2rad(driver.output['ang_acc']) * 3
        # print(f"Driver says acc={self.acc} ang_acc={self.ang_acc}")
        self.move()

    def respawn(self, pos):
        self._pos = np.array(pos)

    def move(self):
        # update angle
        self._ang_vel = Car.clamp(self._ang_vel + self.ang_acc, -MAX_STEER, MAX_STEER)
        self._angle += self._ang_vel * (7 / (1.5 + abs(self._speed))) * self._speed / MAX_SPEED
        self.ang_acc = 0
        # update velocity
        self._speed = Car.clamp(self._speed + self.acc, MAX_REV_SPEED, MAX_SPEED)
        self.acc = 0
        # apply velocity with turning
        self._pos += Car.rotate_point(self._speed, self._angle)
        self._speed *= 0.98
        self._ang_vel *= 0.9

    def cast_rays(self, win):
        # define left most angle of FOV
        start_angle = self.angle - self.FOV / 2
        distances = []
        # loop over casted rays
        for ray in range(self.CASTED_RAYS):
            # cast ray step by step
            for depth in range(MAX_DEPTH):
                # get ray target coordinates
                target_x = self.x - math.sin(start_angle) * depth
                target_y = self.y + math.cos(start_angle) * depth

                # covert target X, Y coordinate to map col, row
                col = int(target_x / TILE_SIZE)
                row = int(target_y / TILE_SIZE)

                # ray hits the condition
                if MAP[row][col] == '#':
                    # highlight wall that has been hit by a casted ray
                    pygame.draw.rect(win, (0, 225, 0), (col * TILE_SIZE + 1,
                                                        row * TILE_SIZE + 1,
                                                        TILE_SIZE - 2,
                                                        TILE_SIZE - 2))

                    # draw casted ray
                    pygame.draw.line(win, (205, 205, 0), (self.x, self.y), (target_x, target_y))

                    # get the precise distance to the wall
                    p1 = ((self.x, self.y), (target_x, target_y))  # ray
                    if target_y < self.y:
                        p2 = ((0, (row + 1) * TILE_SIZE), (1, (row + 1) * TILE_SIZE))  # bottom horizontal line
                    else:
                        p2 = ((0, row * TILE_SIZE), (1, row * TILE_SIZE))  # top horizontal line
                    if target_x > self.x:
                        p3 = ((col * TILE_SIZE, 0), (col * TILE_SIZE, 1))  # left vertical line
                    else:
                        p3 = (((col + 1) * TILE_SIZE, 0), ((col + 1) * TILE_SIZE, 1))  # right vertical line
                    int1 = line_intersection(p1, p2)
                    int2 = line_intersection(p1, p3)
                    # pygame.draw.circle(win, (0, 0, 255), int1, 5)
                    # pygame.draw.circle(win, (0, 0, 255), int2, 5)
                    d1 = sq_dist((self.x, self.y), int1)
                    d2 = sq_dist((self.x, self.y), int2)
                    if (int1[0] < min(self.x, target_x)) or (int1[1] < min(self.y, target_y)) or (
                            int1[0] > max(self.x, target_x) or (int1[1] > max(self.y, target_y))):
                        distances.append(d2)
                    elif (int2[0] < min(self.x, target_x)) or (int2[1] < min(self.y, target_y)) or (
                            int2[0] > max(self.x, target_x) or (int2[1] > max(self.y, target_y))):
                        distances.append(d1)
                    else:
                        distances.append(max(d1, d2))
                    break

            pygame.draw.circle(win, self.color, (int(self.x), int(self.y)), 8)
            # increment angle by a single step
            start_angle += self.STEP_ANGLE
        return distances

    @staticmethod
    def clamp(val, min_val, max_val):
        return max(min_val, min(val, max_val))

    @staticmethod
    def rotate_point(speed, angle):
        dx = speed * math.cos(angle + pi / 2)
        dy = speed * math.sin(angle + pi / 2)
        return np.array([dx, dy])

    @property
    def x(self):
        return self._pos[0]

    @property
    def y(self):
        return self._pos[1]

    @property
    def angle(self):
        return self._angle


def sq_dist(p1, p2):
    return (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2


def line_intersection(line1, line2):
    """Receives two lines defined by two points belonging to each. Returns their intersection"""
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return float('inf'), float('inf')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y
