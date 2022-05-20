import numpy as np  
import sys
import os
import time

# number of boids.
num = 150

# speed of simulation.
speed = 1
# nap time (s).
nap = 0.035

# radius (in cells) of boid awareness.
vision = 3
# dependence on frontal awareness over peripheral.
lookahead = 1.75

# weights for flocking behaviour.
weight_weight = 1.5
velo_weight = 1.2
steer_weight = 1.75
avoid_weight = 2.2

# printing details.
writing = [' ', '.', '*', '*', '#']
colours = [240, 250, 250, 250, 15]  

# 1. boids fly towards the centre of mass.

# 2. boids avoid collisions.

# 3. boids match velocity.

# print function.
def disp():
    global display, boids
    display = np.full((sizeX, sizeY), 0)
    for boid in boids:
        display[round(boid[0] - 1) % sizeX, round(boid[1] - 1) % sizeY] += 1
    for x in range(sizeX):
        for y in range(sizeY):
            total = display[x, y]
            if total > 4: 
                total = 4
            sys.stdout.write(f'\x1B[38;5;{ colours[total]}m {writing[total]}')
        print()
    print(f'\x1b[{sizeX + 1}A')

# vector normalisation
def normalise(vector):
    length = np.sqrt((vector**2).sum())
    return vector / length if length > 0.0 else vector

# boid awareness (no blind spots at current)
def view(boid):
    distances = np.sqrt(((boids - boid)**2).sum(1)) # array of distances to each other boid
    truths = distances < vision
    return(truths.reshape(num, 1))

def avoid(boid):
    b = np.round(boid).astype(int)
    indx = ix - np.round(vision)
    indy = iy - np.round(vision)
    ind = np.stack((indx, indy), 1)
    bools = mask[indx + b[0] + 6, indy + b[1] + 6].reshape((np.round(vision) * 2 + 1)**2, 1)
    obstacles = (-ind) * bools
    return obstacles
    
# application of rules for a single boid.
def rules(index):
    boid = boids[index] 
    if not np.all((0 < boid[0]) & (boid[0] < sizeX)):
        boid[0] = boid[0] % sizeX
        vel = oldvel[index]
    if not np.all((0 < boid[1]) & (boid[1] < sizeY)):
        boid[1] = boid[1] % sizeY
        vel = oldvel[index]
    else:
        vel = velocities[index]
    boids[index] = boid
    viewer = boid + (vel / 4 * lookahead)
    avoid(viewer)
    v = view(viewer)
    t = v.sum()
    visb = boids * v 
    visv = velocities * v
    if t != 0:
        centre = normalise((visb.sum(0) / t) - boid) * weight_weight
        velocity = normalise(visv.sum(0) / t) * velo_weight
        steer = visb - boid
        steer[steer == - boid] = 0
        steerage = normalise(steer.sum(0) / t) * steer_weight
        avoidance = normalise(avoid(viewer).sum(0)) * avoid_weight
        return((centre + velocity + steerage + avoidance) * speed) 
    else:
        return(vel)
    #avoid(boid + (velocities[index] / 4 * lookahead))
    #avoidance = 0

t = (np.round(vision) * 2 + 1).astype(int)
ix = np.repeat(np.arange(t), t)
iy = np.tile(np.arange(t), t)
boids = np.random.rand(num, 2) * os.get_terminal_size().lines
olds = np.copy(boids)
vectors = np.full((num, 2), 0.0) 
oldvel = np.full((num, 2), 0.0)

while True:
    try:
        velocities = (boids - olds)
        sizeX = os.get_terminal_size().lines - 2
        sizeY = np.floor(os.get_terminal_size().columns / 2).astype(int)
        mask = np.full((sizeX + 12, sizeY + 12), True)
        mask[6: -6, 6: -6] = False
        for i in range(num):
            vectors[i] = rules(i)
        olds = np.copy(boids)
        boids += vectors
        oldvel = np.copy(velocities)
        disp()
        time.sleep(nap)
    except KeyboardInterrupt:
        sys.exit(0)
       
