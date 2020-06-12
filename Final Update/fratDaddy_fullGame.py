import random, csv
import fratDaddy_control, sim
import numpy as np
import pyautogui, time

# CUP DATA --------------------------------------
cups_left = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]      # 1 if still in play
# baseAngle and ballVelocity combinations that have landed in the corresponding cup
cup11 = [[-66.3, -5.35], [-64.067, -5.4778], [-65.401, -5.427], [-59.943, -5.792]]
cup21 = [[-66.3, -5.5], [-84.93, -5.424], [-99.145, -5.679], [-98.222, -5.701], [-69.809, -5.455], [-96.246, -5.649], [-87.379, -5.636], [-76.019, -5.557], [-80.193, -5.584], [-73.597, -5.381], [-92.386, -5.621], [-85.907, -5.448], [-68.025, -5.471], [-98.97, -5.629], [-83.602, -5.538], [-94.111, -5.646], [-70.167, -5.521]]
cup22 = [[-58.563, -5.979], [-59.183, -5.881], [-58.5, -6], [-59.943, -5.792], [-59.919, -5.782]]
cup31 = [[-93.23, -5.657], [-93.814, -5.716], [-88.565, -5.423], [-85.994, -5.407], [-82.141, -5.671], [-104.467, -5.824], [-87.379, -5.636], [-80.193, -5.584]]
cup32 = [[-60.542, -5.873], [-59.919, -5.782], [-59.989, -5.917], [-70.196, -5.352], [-60.5, -5.88], [-56.5, -6.15], [-98.222, -5.701]]
cup33 = [[-61.417, -5.628], [-80.856, -5.972], [-75.282, -5.456]]
cup41 = [[-59.943, -5.792], [-67.241, -5.0266], [-88.565, -5.423], [-85.994, -5.407]]
cup42 = [[-66.467, -5.824], [-83.938, -5.961], [-95.003, -5.718], [-74.752, -5.897], [-69.209, -5.906], [-72.783, -5.861], [-67.938, -5.837], [-79.154, -5.981], [-68.446, -5.864], [-85.367, -5.937], [-79.102, -5.937], [-66.691, -5.859], [-73.639, -5.931], [-77.794, -5.945], [-75.682, -5.955], [-71.573, -5.899], [-70.807, -5.865], [-65.181, -5.885], [-64.532, -5.897], [-63.349, -5.855], [-65.944, -5.834], [-62.454, -5.845], [-67.5, -5.88], [-66.5, -5.88], [-65.5, -5.88], [-64.5, -5.88], [-62.5, -5.88], [-65.467, -5.824], [-64.467, -5.824], [-63.467, -5.824], [-98.222, -5.701]]
cup43 = [[-55.45, -6.22], [-55.47, -6.22], [-59.183, -5.881], [-70.196, -5.352]]
cup44 = [[-75.282, -5.456], [-55.47, -6.22]]
allshots = [cup11, cup21, cup22, cup31, cup32, cup33, cup41, cup42, cup43, cup44]

# counts the number of times the corresponding shot has made it into that cup
cup11count = [1, 1, 5, 1]
cup21count = [1, 1, 2, 1, 2, 2, 1, 2, 1, 3, 2, 2, 2, 2, 4, 2, 2]
cup22count = [3, 2, 2, 4, 1]
cup31count = [3, 2, 1, 1, 4, 1, 2, 2]
cup32count = [3, 1, 5, 1, 2, 1, 1]
cup33count = [1, 3, 1]
cup41count = [1, 1, 2, 1]
cup42count = [3, 1, 1, 2, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 1]
cup43count = [2, 1, 1, 2]
cup44count = [2, 1]
allcounts = [cup11count, cup21count, cup22count, cup31count, cup32count, cup33count, cup41count, cup42count, cup43count, cup44count]

# ugly way to get a weighted random index for selecting the next shot
cup11index = [0, 1, 2, 3]
cup21index = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
cup22index = [0, 1, 2, 3, 4]
cup31index = [0, 1, 2, 3, 4, 5, 6, 7]
cup32index = [0, 1, 2, 3, 4, 5, 6]
cup33index = [0, 1, 2]
cup41index = [0, 1, 2, 3]
cup42index = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]
cup43index = [0, 1, 2, 3]
cup44index = [0, 1]
allshotindex = [cup11index, cup21index, cup22index, cup31index, cup32index, cup33index, cup41index, cup42index, cup43index, cup44index]

def addToShots(cup, cupcount, cupindex, pair):
    if pair not in cup:     # if this pair hasn't been recorded as making this shot before then add it
        cup.append(pair)
        cupcount.append(1)
        cupindex.append(len(cupindex)-1)
    else:           # if this shot has been used before, increase its reliability
        shot = cup.index(pair)
        cupcount[shot] += 1

def shootForRandomCup():
    # select a cup that is still in play
    cup = 0
    pair = []
    while cup == 0:
        i = random.randrange(10)  # random index
        cup = cups_left[i]  # choose random cup to aim for among cups left in play
        if not np.any(allcounts[i]):        # if cupcount for this cup is all zeros
            pair = random.choice(allshots[i])  # random [baseAngle, ballVelocity] for given cup
        else:
            norm = sum(allcounts[i])     # find sum of all elements in list
            normalized = []
            for element in allcounts[i]:        # normalize each element to make it a probability
                normalized.append(element/norm)
            pairIndex = np.random.choice(allshotindex[i], p=normalized)     # select a shot using the weights accumulated
            pair = allshots[i][pairIndex]
    return pair

def shootRandomly():
    baseAngle = random.uniform(-75, -48)       # -100 to -55
    ballVelocity = random.uniform(-6.3, -5)     # -6.5 to -5
    return [baseAngle, ballVelocity]

print("MOVE CURSOR OVER PLAY BUTTON IN COPPELIASIM")
input("Press Enter to continue...")
x, y = pyautogui.position()


pyautogui.click(x, y)
time.sleep(2)

while np.any(cups_left):
    pair = shootForRandomCup()      # used for full game
    #pair = shootRandomly()     # used when gathering data
    print(pair)
    
    pyautogui.click(882, 66)
    time.sleep(2)

    # SET UP CLIENT ----------------------------------
    sim.simxFinish(-1)  # closes all opened connections
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # starts a connection
    if clientID != -1:
        print("Connected to remote API server")
    else:
        print("Not connected to remote API server")

    sensors = fratDaddy_control.shoot(clientID, pair[0], pair[1], cups_left)
    time.sleep(2)

    for i in range(0, 10):      # if the sensor says that the ball landed in that cup, remove that cup from the game
        if sensors[i]:
            cups_left[i] = 0
            if i == 0: addToShots(cup11, cup11count, cup11index, pair)
            elif i == 1: addToShots(cup21, cup21count, cup21index, pair)
            elif i == 2: addToShots(cup22, cup22count, cup22index, pair)
            elif i == 3: addToShots(cup31, cup31count, cup31index, pair)
            elif i == 4: addToShots(cup32, cup32count, cup32index, pair)
            elif i == 5: addToShots(cup33, cup33count, cup33index, pair)
            elif i == 6: addToShots(cup41, cup41count, cup41index, pair)
            elif i == 7: addToShots(cup42, cup42count, cup42index, pair)
            elif i == 8: addToShots(cup43, cup43count, cup43index, pair)
            elif i == 9: addToShots(cup44, cup44count, cup44index, pair)
            else: print("why are we here?")

    if not np.any(cups_left):
        for i in range(0, 10):
            cups_left[i] = 1

print("ALL CUPS HAVE BEEN ELIMINATED")

# saves a file with all the cup inputs in case new inputs were found for cups with few datapoints
with open('cuprun.csv', 'w') as myfile:
    wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
    wr.writerow([cup11])
    wr.writerow([cup11count])
    wr.writerow([cup21])
    wr.writerow([cup21count])
    wr.writerow([cup22])
    wr.writerow([cup22count])
    wr.writerow([cup31])
    wr.writerow([cup31count])
    wr.writerow([cup32])
    wr.writerow([cup32count])
    wr.writerow([cup33])
    wr.writerow([cup33count])
    wr.writerow([cup41])
    wr.writerow([cup41count])
    wr.writerow([cup42])
    wr.writerow([cup42count])
    wr.writerow([cup43])
    wr.writerow([cup43count])
    wr.writerow([cup44])
    wr.writerow([cup44count])
