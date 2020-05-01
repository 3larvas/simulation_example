#!/usr/bin/python2
# shebang 이 스크립트를 해석해 줄 인터프리터의 절대 경로 지정

'''
K-City Map (EPSG:4326) Parse out 126.77/37.2 10000-2484/1000000-379547
to GOD JI UNG
--FIX--
Line 25 -> (all_lane.csv) for all lane // (out_lane.csv) for contour lane
Line 65 for Lane Thickness
Line 69 for Resolution change

'''
import numpy as np
import matplotlib.pyplot as plt
import csv
import cv2
# 기준 좌표 설정
base_lat = 37.2378987227939#min37.2385806467945
base_lon = 126.771746659531#원점 위도126.772598099467
base_alt = 10.0
# 만들어진 map img load
map_img_loaded = cv2.imread('map_global.jpg', cv2.IMREAD_GRAYSCALE)
# 초기 좌표 설정
point_x=126.774489
point_y=37.242228
# local map size 정의
local_map_size=100
# global 배열 size 정의
global_height = 9900
global_width = 4000
# global 배열 선언
map_global = np.zeros([global_height, global_width], dtype=int)  # Global grid
# map img 생성용 np배열
img_map_global = np.zeros([global_height, global_width], dtype=int)  # Global grid

# local map point 좌표 정보 저장할 배열 선언
local_x = []
local_y = []
# local map np 배열
map_local = np.zeros([200, 200], dtype=int)  # local grid

# local map 정보 update
def local_map():
    x, y = local_change(point_x, point_y)
    # global map에 현재위치 표시
    cv2.line(map_img_loaded, (int(x/10), int(y/10)), (int(x/10), int(y/10)), (255, 0, 0), 3)
    cv2.imshow('global_map', map_img_loaded)
    for i in range(local_map_size * 2):
        for j in range(local_map_size * 2):
            if (map_global[y + i - local_map_size][x + j - local_map_size] == 1):  # Visualization
                local_x.append(x + j - local_map_size)
                local_y.append(y + i - local_map_size)
                map_local[i][j] = map_global[y + i - local_map_size][x + j - local_map_size]  # local grid map

    plt.title('local map')
    plt.scatter(local_x, local_y, marker=',', c='purple')
    plt.plot(x, y, 'c+', markersize=15)
    # 그래프 스케일 고정
    plt.xlim([x - local_map_size, x + local_map_size])
    plt.ylim([(y - local_map_size), (y + local_map_size)])
    plt.grid(True)
    plt.pause(0.01)
    plt.cla()

def local_change(x, y):  # 들어오는 값 다 우리좌표로 변환
    cx = int((float(x) - float(base_lon)) * 1000000)
    cy = int((float(y) - float(base_lat)) * 1000000)
    return (cx, cy)

def make_map():  # global map받아오기 csv 파싱
    with open('map_big.csv', mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for next_r in csv_reader:
            dy = int((float(next_r['Y']) - float(base_lat)) * 1000000)  # index화
            dx = int((float(next_r['X']) - float(base_lon)) * 1000000)  # 현재점에서 원점 빼기
            map_global[dy, dx] = 1
    # ============map 최초 실행 시 map img 생성============
            # img_map_global[dy, dx] = 255
            # img_map_global[dy+2, dx] = 255
            # img_map_global[dy, dx+2] = 255
    # cv2.imwrite('map_global.jpg', img_map_global)
    # ====================================================

if __name__ == "__main__":
    make_map()
    x, y = local_change(point_x, point_y)
    while True:
        point_y += 0.00001  # 임시로 Y축 상승
        local_map()
    plt.show()
    cv2.waitKey(0)
    cv2.destroyAllWindows()
