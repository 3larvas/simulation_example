import geopandas as gpd
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = 'NamumGothic'
plt.rcParams["figure.figsize"] = (10, 10)

file_nm = "HDMap/A1_LANE.shp"
k_city = gpd.read_file(file_nm)

ax = k_city.convex_hull.plot(color='purple', edgecolor="w")
ax.set_title("k-city")
ax.set_axis_iff()
plt.show()