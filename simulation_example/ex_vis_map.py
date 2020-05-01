import cv2

src = cv2.imread('map.PNG', cv2.IMREAD_GRAYSCALE)
h, w = src.shape[0:2]
arr_map = [[0 for col in range(w)] for row in range(h)]
cv2.imshow('original', src)
for i in range(h):
    # print("\n")
    for j in range(w):
        if src[i, j]!=0 :
            # print(1 , end = '')
            arr_map[i][j] = 1
        else:
            # print(src[i, j] , end = '')
            arr_map[i][j] = 0


cv2.waitKey(0)
cv2.destroyAllWindows()
