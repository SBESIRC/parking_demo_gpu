import pybullet as p
import time
import pybullet_data


if __file__ == '__main__':
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)