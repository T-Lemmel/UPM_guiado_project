from apolo import Apolo
import math
import random
import time
pi = math.pi

ap = Apolo()


def random_point_in_area(area):
    margin = 0.5
    x_min = area[0] + margin
    x_max = area[1] - margin
    y_min = area[2] + margin
    y_max = area[3] - margin
    x = random.uniform(x_min, x_max)
    y = random.uniform(y_min, y_max)
    return x, y


def random_point_in_multiple_areas(possible_areas):
    chosen_area = random.choice(possible_areas)
    return random_point_in_area(chosen_area)

areas = [
        (-1, 1, 0, 9), # corridor
        (1, 4, 7, 9), # end of corridor
        (2.3, 3.8, 1, 5.5), # kitchen
        (-5, -1, 0.5, 1.5), # living room 1
        (-3.4, -1, 1.5, 6), # living room 2
        (-4, -2.3, 6, 10.5), # bedroom 1
        (-2, 0.5, 9, 11), # bedroom 2
        (-0.5, 2.3, 11, 12), # bathroom 1
        (2, 5, 9.6, 11), # bathroom 2
        ]

# balizas
x_baliza_patient, y_baliza_patient = random_point_in_multiple_areas(areas)
ap.placeObject('Patient', x_baliza_patient, y_baliza_patient, 0.4, 0, 0, 0)
ap.placeObject('arrow', x_baliza_patient, y_baliza_patient, 0.6, pi/2, 0, 0)
ap.placeObject('baliza1', 1, 1.5, 0.4, 0, 0, 0)
ap.placeObject('baliza2', -1, 5, 0.4, 0, 0, 0)
ap.placeObject('baliza3', -1, 7, 0.4, 0, 0, 0)

# room 1 - kitchen
ap.placeObject('Sofa2', 4, 1, 0, 0, 0, pi)
ap.placeObject('insofa2', 2, 0, 0, 0, 0, 0)
ap.placeObject('kforn1', 3.2, 6, 0, 0, 0, 0)
ap.placeObject('kforn2', 4.2, 4, 0, 0, 0, 0)
ap.placeObject('kforn3', 4.3, 3.2, 0, 0, 0, 0)
ap.placeObject('kforn4', 4.2, 2.2, 0, 0, 0, 0)
ap.placeObject('kforn5', 5, 7, 1.2, pi/2, 0, -pi/2)
ap.placeObject('bigtable', 1, 3, 0, pi/2, 0, pi/2)
ap.placeObject('inbigtable', 1.3, 3.4, 0, 0, 0, 0)
ap.placeObject('chair3', 2.25, 5.75, 0.9, pi, 0, -pi/2)
ap.placeObject('inchair3', 1.8, 5.3, 0, 0, 0, 0)
ap.placeObject('chair4', 2.25, 4.75, 0.9, pi, 0, -pi/2)
ap.placeObject('inchair4', 1.8, 4.3, 0, 0, 0, 0)
ap.placeObject('chair5', 2.25, 3.75, 0.9, pi, 0, -pi/2)
ap.placeObject('inchair5', 1.8, 3.3, 0, 0, 0, 0)

# room 2 - living room
ap.placeObject('Sofa', -5, 2, 0, 0, 0, 0)
ap.placeObject('insofa', -5, 2, 0, 0, 0, 0)
ap.placeObject('livingforn', -1, 0, 0, 0, 0, pi/2)
ap.placeObject('table', -5, 5, 0, pi/2, 0, 0)
ap.placeObject('intable', -4.6, 4.4, 0, 0, 0, 0)
ap.placeObject('chair1', -4.3, 5.3, 0.9, pi, 0, 0)
ap.placeObject('inchair1', -4.3, 4.85, 0, 0, 0, 0)
ap.placeObject('chair2', -3.85, 3.8, 0.9, pi, 0, pi)
ap.placeObject('inchair2', -4.3, 3.8, 0, 0, 0, 0)
ap.placeObject('tv', -4.6, 0.08, 0.9, pi/2, 0, 0)

# room 3 - bedroom
ap.placeObject('bed1', -5, 12, 0, pi/2, 0, 0)
ap.placeObject('inbed1', -5, 10, 0, 0, 0, 0)
ap.placeObject('bed2', -3.5, 12, 0, pi/2, 0, 0)
ap.placeObject('inbed2', -3.5, 10, 0, 0, 0, 0)
ap.placeObject('bed3', -4, 6, 0, pi/2, 0, pi)
ap.placeObject('inbed3', -5, 6, 0, 0, 0, 0)
ap.placeObject('armadietto', -1.5, 7.8, 0, 0, 0, 0)
ap.placeObject('armadietto2', -5, 8.5, 0, 0, 0, 0)
ap.placeObject('armadietto3', -2, 12, 0, 0, 0, -pi/2)

# room 4 - bathroom
ap.placeObject('shower', 4.2, 11.2, 0, 0, 0, 0)
ap.placeObject('sink', 2.7, 11.4, 0, 0, 0, 0)
ap.placeObject('bathforn', 1.1, 9, 0, 0, 0, 0)
ap.placeObject('washingmach', 3, 9, 0, 0, 0, 0)
ap.placeObject('dryer', 2.2, 9, 0, 0, 0, 0)
ap.placeObject('wc', 4.5, 9, 0.9, pi, 0, -pi)
ap.placeObject('mirror', 3, 12, 1, pi/2, 0, 0)
ap.placeObject('inwc', 4.05, 9, 0, 0, 0, 0)

# end of corridor
ap.placeObject('armadio', 4.4, 7.25, 0, 0, 0, 0)

# obstacle
ap.placeObject('obstacle', -0.75, 2.25, 0, 0, 0, 7*pi/180)

ap.updateWorld()
