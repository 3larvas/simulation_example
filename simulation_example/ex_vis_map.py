import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
from time import sleep

show_animation = True

src = cv2.imread('map.PNG', cv2.IMREAD_GRAYSCALE)
h, w = src.shape[0:2]
arr_map = [[0 for col in range(w)] for row in range(h)]
x=[]
y=[]
cv2.imshow('original', src)
for i in range(h):
    # print("\n")
    for j in range(w):
        if src[i, j]!=0 :
            arr_map[i][j] = 1
            x.append(j)
            y.append(-i)
        else:
            arr_map[i][j] = 0

plt.scatter(x,y, 5)
plt.show()

cv2.waitKey(0)
cv2.destroyAllWindows()
