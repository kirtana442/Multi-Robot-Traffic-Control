# visualization.py
import networkx as nx
import matplotlib.pyplot as plt

def visualize_lane_heatmap(G: nx.DiGraph, save_path: str = "lane_heatmap.png"):
    """Matplotlib lane heatmap using edge colors (practical for graph-based map).
    Shows congestion + historical usage pressure."""
    pos = nx.get_node_attributes(G, 'pos')
    fig, ax = plt.subplots(figsize=(12, 8))
    nx.draw_networkx_nodes(G, pos, node_size=500, node_color="lightblue", ax=ax)
    nx.draw_networkx_labels(G, pos, font_size=10, ax=ax)
    edges = list(G.edges())
    colors = [G.edges[u, v]['lane'].congestion_score for u, v in edges]
    nx.draw_networkx_edges(G, pos, edgelist=edges, edge_color=colors,
                           edge_cmap=plt.cm.OrRd, width=4, ax=ax)
    sm = plt.cm.ScalarMappable(cmap=plt.cm.OrRd, norm=plt.Normalize(0, 1))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax)
    cbar.set_label('Lane Congestion Score (0-1)')
    ax.set_title("Lane Heatmap - Congestion & Usage Pressure")
    plt.savefig(save_path)
    plt.close(fig)
    print(f"Lane heatmap saved to {save_path}")