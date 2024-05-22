import numpy as np
import math
import random

def read(filename, errorprob = 0):
    with open(filename) as file:
        codesL = [line.rstrip() for line in file]
        codes = np.zeros(len(codesL))
        for i,z in enumerate(codesL):
            flip = random.uniform(0.0000001,1) < errorprob
            if flip:
                codes[i] = -int(z) + 1
            else:
                codes[i] = int(z)
        return codes

code0 = read("code0.txt")
code1 = read("code1.txt", 0.48)
best = (-math.inf, -1)

for i in range(len(code0)):
    shifted = np.roll(code1, -i)
    corval = np.correlate(code0, shifted)[0]
    #print(corval)
    if corval > best[0]:
        best = (corval, i)

print("code 1 needs to be shifted", best[1], "to the left to get the best alligment (",best[0],")")

#The bit-flip rate probability has to reach 48% in our testing to start to receive small errors in the shift of the best allignment