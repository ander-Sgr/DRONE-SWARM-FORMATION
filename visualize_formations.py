import os
import matplotlib.pyplot as plt

def plot_formation_vs_actual(client, formation_name, target_positions, initial_positions=None, save_folder="drone_plot"):
    """
    Dibuja:
    - Posición inicial
    - Posición objetivo
    - Posición final real
    - Trayectorias
    """
    os.makedirs(save_folder, exist_ok=True)

    fig, ax = plt.subplots()
    ax.set_title(f"Formation: {formation_name}")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True)

    for drone_name, visual_target in target_positions.items():
        tx, ty = visual_target[0], visual_target[1]
        ax.scatter(tx, ty, c='blue', marker='o', label='Objetivo' if drone_name == list(target_positions)[0] else "")
        ax.text(tx + 0.3, ty + 0.3, f"{drone_name} (target)", fontsize=8)

        pose = client.simGetObjectPose(drone_name).position
        rx, ry = pose.x_val, pose.y_val
        ax.scatter(rx, ry, c='red', marker='x', label='Real' if drone_name == list(target_positions)[0] else "")
        ax.text(rx + 0.3, ry + 0.3, f"{drone_name} (real)", fontsize=8)

        if initial_positions:
            ix, iy = initial_positions[drone_name][0], initial_positions[drone_name][1]
            ax.scatter(ix, iy, c='green', marker='^', label='Inicial' if drone_name == list(target_positions)[0] else "")
            ax.text(ix + 0.3, iy + 0.3, f"{drone_name} (start)", fontsize=8)

            # Trayectoria: inicio → target → real
            ax.plot([ix, tx, rx], [iy, ty, ry], 'gray', linestyle='--', linewidth=0.7)

    ax.legend()
    ax.axis('equal')
    filename = f"{formation_name}_plot.png"
    plt.savefig(os.path.join(save_folder, filename))
    plt.close()
