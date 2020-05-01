import cv2
import matplotlib.pyplot as plt
# local map scale
local_map_size = 50
# 초기 좌표
cur_x = 70
cur_y = 150

global_map = cv2.imread('map.PNG', cv2.IMREAD_GRAYSCALE)
h, w = global_map.shape[0:2]

arr_map = [[0 for col in range(w)] for row in range(h)]

# 이미지 배열에서 0이 아닌값을 1로 arr_map에 저장
for i in range(h):
    for j in range(w):
        if global_map[i, j]!=0 :
            arr_map[i][j] = 1
        else:
            arr_map[i][j] = 0

while True:
    # x, y축 스케일 고정
    plt.xlim([cur_x - local_map_size, cur_x + local_map_size])
    plt.ylim([-(cur_y - local_map_size), -(cur_y + local_map_size)])
    # 현재위치 표시
    cv2.line(global_map, (cur_x, cur_y), (cur_x, cur_y), (255, 0, 0), 10)
    # global map 출력
    cv2.imshow('original', global_map)
    # local map point 좌표 정보를 저장할 배열
    x = []
    y = []
    # 현재위치 기준 local map size 만큼 상하좌우 출력
    for i in range(cur_y - local_map_size, cur_y + local_map_size):
        for j in range(cur_x - local_map_size, cur_x + local_map_size):
            # 1인 배열은 x, y배열에 추가
            if arr_map[i][j] != 0:
                x.append(j)
                y.append(-i)
    # local map point 출력
    plt.scatter(x, y, 5)
    # y좌표를 1씩 증가
    cur_y += 1
    plt.pause(0.0001)
    plt.cla()


plt.show()
cv2.waitKey(0)
cv2.destroyAllWindows()

