import geopandas as gpd
import matplotlib.pyplot as plt
# geopandas 패키지 설치 방법... 좀 복잡함...
# https://codedragon.tistory.com/9556
# https://pypi.org/project/geopandas/
# geopandas 패키지 설치 후 설치
# pip install descartes

plt.rcParams["font.family"] = 'NamumGothic'
plt.rcParams["figure.figsize"] = (10, 10)

file_nm = "HDMap/A1_LANE.shp"
k_city = gpd.read_file(file_nm)
k_city
ax = k_city.plot(color='purple', edgecolor="b")
ax.set_title("k-city")
ax.set_axis_off()
plt.show()