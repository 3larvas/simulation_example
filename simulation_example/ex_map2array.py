import cv2

src = cv2.imread('map.PNG', cv2.IMREAD_GRAYSCALE)

cv2.imshow('original', src)
for i in range(2000):
    print("\n")
    for j in range(2000):
        if src[i, j]!=0 :
            print(1 , end = '')
        else:
            print(src[i, j] , end = '')

cv2.waitKey(0) 
cv2.destroyAllWindows()
