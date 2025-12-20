import folium
import json
import numpy as np
import matplotlib as plt

def color_to_hex(color):
    return '#%02x%02x%02x' % (int(color[0]*255), int(color[1]*255), int(color[2]*255))

def get_feature_style(lvl: int):
    def feature_style(feature):
        cell_idx = feature["properties"]["cell_idxs"][lvl]
        if cell_idx != -1:
            return {
                "color": color_to_hex(colors[cell_idx]),
                "fill_color": color_to_hex(colors[cell_idx]),
                "radius": 8,
                "weight": 2
            }
        else:
            return {
                "color": "orange",
                "fill_color": "orange",
                "radius": 24,
                "weight": 4
            }
    return feature_style

def get_n_colors(n, cmap_name='vanimo'):
    if n <= 0:
        return []

    # Get the colormap object
    cmap = plt.colormaps[cmap_name]

    # Generate n evenly spaced numbers between 0 and 1
    # These numbers are used to sample the colormap
    indices = np.linspace(0, 1, n, endpoint=True)

    # Get the colors from the colormap
    colors = cmap(indices)

    # Convert to a list of colors (optional, but often clearer)
    return colors.tolist()

def get_center(data):
    coords = [feature['geometry']['coordinates'] for feature in data['features']]
    mean_lon = sum(p[0] for p in coords) / len(coords)
    mean_lat = sum(p[1] for p in coords) / len(coords)
    return [mean_lat, mean_lon]

n_cells_in_lowest_level = 8
level = 1
colors = get_n_colors(n_cells_in_lowest_level)

with open("/home/hendrik/Documents/GitHub/nigiri/cmake-build-release-clang-17-1/out.json") as f:
    d = json.load(f)
    m = folium.Map(location=get_center(d), zoom_start=4, tiles="Cartodb Positron")
    folium.GeoJson(
        d,
        marker=folium.Circle(radius=8, fill_color="black", fill_opacity=1, color="black", weight=2),
        style_function=get_feature_style(level)
    ).add_to(m)

m.show_in_browser()