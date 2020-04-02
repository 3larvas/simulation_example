import geopandas as gpd
import matplotlib.pyplot as plt
# https://codedragon.tistory.com/9556
# https://pypi.org/project/geopandas/
# pip install descartes
plt.rcParams["font.family"] = 'NamumGothic'
plt.rcParams["figure.figsize"] = (10, 10)

file_nm = "HDMap/A1_LANE.shp"
k_city = gpd.read_file(file_nm)
k_city
ax = k_city.convex_hull.plot(color='purple', edgecolor="w")
ax.set_title("k-city")
ax.set_axis_off()
plt.show()