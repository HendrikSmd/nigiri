import folium
import json
import numpy as np
import matplotlib as plt

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

def color_to_hex(color):
    return '#%02x%02x%02x' % (int(color[0]*255), int(color[1]*255), int(color[2]*255))

colors = get_n_colors(8)

m = folium.Map(location=[46.875, 8.481], zoom_start=4, tiles="Cartodb Positron")

def get_feature_style(feature):
    cell_idx = feature["properties"]["cell_idx"]
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


with open("/home/hendrik/Documents/GitHub/nigiri/cmake-build-debug-clang-17/out.json") as f:
    d = json.load(f)
    folium.GeoJson(
        d,
        marker=folium.Circle(radius=8, fill_color="black", fill_opacity=1, color="black", weight=2),
        style_function=get_feature_style
    ).add_to(m)

m.show_in_browser()