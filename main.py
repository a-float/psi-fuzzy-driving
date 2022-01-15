import pygame
import sys
import math
from car import Car
from config import *

CENTER = (SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2)
fuzzy_car1 = Car(randomize_pos(CENTER, 40), 0, (255, 0, 0))
fuzzy_car2 = Car(randomize_pos(CENTER, 30), -math.pi / 100, (0, 255, 255))
# fuzzy_car3 = Car(CENTER, math.pi/100, (255, 0, 255))
my_car = Car(CENTER, 0, (50, 100, 255))
cars = [fuzzy_car1, fuzzy_car2, my_car]
pygame.init()

win = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

pygame.display.set_caption('Fuzzy cars')
clock = pygame.time.Clock()


def draw_map():
    for row in range(MAP_SIZE[1]):
        for col in range(MAP_SIZE[0]):
            pygame.draw.rect(
                win,
                (200, 200, 200) if MAP[row][col] == '#' else (100, 100, 100),
                (col * TILE_SIZE + 1, row * TILE_SIZE + 1, TILE_SIZE - 2, TILE_SIZE - 2)
            )


while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit(0)

    pygame.draw.rect(win, (0, 0, 0), (0, 0, SCREEN_WIDTH, SCREEN_HEIGHT))
    draw_map()

    # apply raycasting
    for fcar in cars:
        if fcar == my_car:
            continue
        dists = fcar.cast_rays(win)
        fcar.update(dists)

    my_car.cast_rays(win)
    my_car.move()

    for car in cars:
        if MAP[int(car.y // TILE_SIZE)][int(car.x // TILE_SIZE)] == '#':
            car.respawn(CENTER)

    keys = pygame.key.get_pressed()

    if keys[pygame.K_LEFT]:
        my_car.ang_acc = -math.pi / 150
    if keys[pygame.K_RIGHT]:
        my_car.ang_acc = math.pi / 150
    if keys[pygame.K_UP]:
        my_car.acc = 1 / 1.4
    if keys[pygame.K_DOWN]:
        my_car.acc = -1 / 2.9

    pygame.display.flip()
    clock.tick(30)
