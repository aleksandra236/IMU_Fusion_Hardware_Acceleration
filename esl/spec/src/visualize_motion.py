#!/usr/bin/env python3
"""
Vizualizacija rekonstrukcije kretanja objekta iz senzorskih podataka.

Prikazuje:
1. 3D animaciju orijentacije objekta tokom vremena
2. Grafike Euler uglova (roll, pitch, yaw) kroz vreme
3. Trajektoriju orijentacije u 3D prostoru

Korišćenje:
    python3 visualize_motion.py orientation_data.csv
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
from scipy.spatial.transform import Rotation as R


def quaternion_to_rotation_matrix(qw, qx, qy, qz):
    """Konvertuje quaternion u 3x3 rotacionu matricu."""
    r = R.from_quat([qx, qy, qz, qw])  # scipy koristi [x, y, z, w] format
    return r.as_matrix()


def create_box_vertices(scale=1.0):
    """Kreira vertikse kutije (predstavlja senzor/objekat)."""
    # Dimenzije kutije (x, y, z)
    dx, dy, dz = 2.0 * scale, 1.0 * scale, 0.5 * scale
    
    vertices = np.array([
        [-dx/2, -dy/2, -dz/2],
        [ dx/2, -dy/2, -dz/2],
        [ dx/2,  dy/2, -dz/2],
        [-dx/2,  dy/2, -dz/2],
        [-dx/2, -dy/2,  dz/2],
        [ dx/2, -dy/2,  dz/2],
        [ dx/2,  dy/2,  dz/2],
        [-dx/2,  dy/2,  dz/2],
    ])
    return vertices


def get_box_faces(vertices):
    """Vraća lica kutije kao Poly3DCollection."""
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],  # donja
        [vertices[4], vertices[5], vertices[6], vertices[7]],  # gornja
        [vertices[0], vertices[1], vertices[5], vertices[4]],  # prednja
        [vertices[2], vertices[3], vertices[7], vertices[6]],  # zadnja
        [vertices[0], vertices[3], vertices[7], vertices[4]],  # leva
        [vertices[1], vertices[2], vertices[6], vertices[5]],  # desna
    ]
    return faces


def rotate_vertices(vertices, rotation_matrix):
    """Rotira vertikse koristeći rotacionu matricu."""
    return np.dot(vertices, rotation_matrix.T)


def draw_axes(ax, rotation_matrix, length=1.5, origin=np.array([0, 0, 0])):
    """Crta koordinatne ose rotirane prema trenutnoj orijentaciji."""
    axes_colors = ['r', 'g', 'b']  # X=crvena, Y=zelena, Z=plava
    axes_labels = ['X', 'Y', 'Z']
    
    for i in range(3):
        axis = np.zeros(3)
        axis[i] = length
        rotated_axis = np.dot(rotation_matrix, axis)
        ax.quiver(origin[0], origin[1], origin[2],
                  rotated_axis[0], rotated_axis[1], rotated_axis[2],
                  color=axes_colors[i], arrow_length_ratio=0.1, linewidth=2)


def load_data(filename):
    """Učitava podatke iz CSV fajla."""
    df = pd.read_csv(filename)
    return df


def plot_euler_angles(df, save_path=None):
    """Plotuje Euler uglove kroz vreme."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    
    time = df['time'].values
    
    # Roll
    axes[0].plot(time, df['roll'].values, 'r-', linewidth=1)
    axes[0].set_ylabel('Roll (°)', fontsize=12)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Rekonstrukcija orijentacije - Euler uglovi', fontsize=14)
    
    # Pitch
    axes[1].plot(time, df['pitch'].values, 'g-', linewidth=1)
    axes[1].set_ylabel('Pitch (°)', fontsize=12)
    axes[1].grid(True, alpha=0.3)
    
    # Yaw
    axes[2].plot(time, df['yaw'].values, 'b-', linewidth=1)
    axes[2].set_ylabel('Yaw (°)', fontsize=12)
    axes[2].set_xlabel('Vreme (s)', fontsize=12)
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Grafik Euler uglova sačuvan: {save_path}")
    
    return fig


def plot_3d_orientation_trajectory(df, save_path=None):
    """Plotuje trajektoriju orijentacije u 3D prostoru (roll, pitch, yaw)."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    roll = df['roll'].values
    pitch = df['pitch'].values
    yaw = df['yaw'].values
    time = df['time'].values
    
    # Normalizuj vreme za boju
    colors = plt.cm.viridis((time - time.min()) / (time.max() - time.min()))
    
    # Scatter plot sa bojom koja predstavlja vreme
    scatter = ax.scatter(roll, pitch, yaw, c=time, cmap='viridis', s=1, alpha=0.5)
    
    # Početna i krajnja tačka
    ax.scatter([roll[0]], [pitch[0]], [yaw[0]], color='green', s=100, marker='o', label='Početak')
    ax.scatter([roll[-1]], [pitch[-1]], [yaw[-1]], color='red', s=100, marker='s', label='Kraj')
    
    ax.set_xlabel('Roll (°)', fontsize=12)
    ax.set_ylabel('Pitch (°)', fontsize=12)
    ax.set_zlabel('Yaw (°)', fontsize=12)
    ax.set_title('Trajektorija orijentacije u prostoru uglova', fontsize=14)
    
    plt.colorbar(scatter, label='Vreme (s)', shrink=0.6)
    ax.legend()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"3D trajektorija sačuvana: {save_path}")
    
    return fig


def create_animation(df, output_file=None, fps=30, skip_frames=1):
    """Kreira animaciju 3D objekta koji se rotira prema podacima."""
    
    # Uzmi svaki n-ti frame za bržu animaciju
    df_anim = df.iloc[::skip_frames].reset_index(drop=True)
    num_frames = len(df_anim)
    
    # Calculate actual time step between frames for real-time playback
    if len(df_anim) > 1:
        avg_dt = (df_anim['time'].iloc[-1] - df_anim['time'].iloc[0]) / (len(df_anim) - 1)
        interval_ms = avg_dt * 1000  # Convert to milliseconds
    else:
        interval_ms = 1000 / fps
    
    print(f"Kreiranje animacije sa {num_frames} frameova...")
    print(f"Vreme između frameova: {interval_ms:.1f}ms (real-time)")
    
    fig = plt.figure(figsize=(12, 10))
    
    # 3D prikaz objekta
    ax1 = fig.add_subplot(221, projection='3d')
    
    # Euler uglovi - pojedinačni grafici
    ax2 = fig.add_subplot(222)
    ax3 = fig.add_subplot(223)
    ax4 = fig.add_subplot(224)
    
    # Podesi 3D ose
    ax1.set_xlim([-2.5, 2.5])
    ax1.set_ylim([-2.5, 2.5])
    ax1.set_zlim([-2.5, 2.5])
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('3D Orijentacija objekta')
    
    # Boje za lica kutije
    face_colors = ['cyan', 'cyan', 'magenta', 'magenta', 'yellow', 'yellow']
    
    # Podaci za plotovanje
    time_data = df_anim['time'].values
    roll_data = df_anim['roll'].values
    pitch_data = df_anim['pitch'].values
    yaw_data = df_anim['yaw'].values
    
    # Check for NaN or Inf values
    import numpy as np
    if not (np.isfinite(roll_data).all() and np.isfinite(pitch_data).all() and np.isfinite(yaw_data).all()):
        print("Warning: Data contains NaN or Inf values. Cleaning...")
        df_anim = df_anim.replace([np.inf, -np.inf], np.nan).dropna()
        time_data = df_anim['time'].values
        roll_data = df_anim['roll'].values
        pitch_data = df_anim['pitch'].values
        yaw_data = df_anim['yaw'].values
        num_frames = len(df_anim)
    
    # Inicijalni objekti za linije
    line_roll, = ax2.plot([], [], 'r-', linewidth=1)
    line_pitch, = ax3.plot([], [], 'g-', linewidth=1)
    line_yaw, = ax4.plot([], [], 'b-', linewidth=1)
    
    # Markeri za trenutnu poziciju
    marker_roll, = ax2.plot([], [], 'ro', markersize=8)
    marker_pitch, = ax3.plot([], [], 'go', markersize=8)
    marker_yaw, = ax4.plot([], [], 'bo', markersize=8)
    
    # Podesi ose za Euler grafike
    ax2.set_xlim([time_data.min(), time_data.max()])
    ax2.set_ylim([roll_data.min() - 5, roll_data.max() + 5])
    ax2.set_ylabel('Roll (°)')
    ax2.set_xlabel('Vreme (s)')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Roll')
    
    ax3.set_xlim([time_data.min(), time_data.max()])
    ax3.set_ylim([pitch_data.min() - 5, pitch_data.max() + 5])
    ax3.set_ylabel('Pitch (°)')
    ax3.set_xlabel('Vreme (s)')
    ax3.grid(True, alpha=0.3)
    ax3.set_title('Pitch')
    
    ax4.set_xlim([time_data.min(), time_data.max()])
    ax4.set_ylim([yaw_data.min() - 5, yaw_data.max() + 5])
    ax4.set_ylabel('Yaw (°)')
    ax4.set_xlabel('Vreme (s)')
    ax4.grid(True, alpha=0.3)
    ax4.set_title('Yaw')
    
    # Originalni vertiksi kutije
    base_vertices = create_box_vertices()
    
    def init():
        return []
    
    def update(frame):
        ax1.cla()
        
        # Podesi 3D ose
        ax1.set_xlim([-2.5, 2.5])
        ax1.set_ylim([-2.5, 2.5])
        ax1.set_zlim([-2.5, 2.5])
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        
        # Uzmi quaternion za ovaj frame
        qw = df_anim.iloc[frame]['qw']
        qx = df_anim.iloc[frame]['qx']
        qy = df_anim.iloc[frame]['qy']
        qz = df_anim.iloc[frame]['qz']
        t = df_anim.iloc[frame]['time']
        
        # Izračunaj rotacionu matricu
        rot_matrix = quaternion_to_rotation_matrix(qw, qx, qy, qz)
        
        # Rotiraj vertikse
        rotated_vertices = rotate_vertices(base_vertices, rot_matrix)
        
        # Nacrtaj kutiju
        faces = get_box_faces(rotated_vertices)
        box = Poly3DCollection(faces, alpha=0.7, linewidths=1, edgecolors='black')
        box.set_facecolor(face_colors)
        ax1.add_collection3d(box)
        
        # Nacrtaj koordinatne ose objekta
        draw_axes(ax1, rot_matrix)
        
        # Nacrtaj referentne ose (svetlije)
        ax1.quiver(0, 0, 0, 2, 0, 0, color='r', alpha=0.3, arrow_length_ratio=0.1)
        ax1.quiver(0, 0, 0, 0, 2, 0, color='g', alpha=0.3, arrow_length_ratio=0.1)
        ax1.quiver(0, 0, 0, 0, 0, 2, color='b', alpha=0.3, arrow_length_ratio=0.1)
        
        ax1.set_title(f'3D Orijentacija - t={t:.2f}s')
        
        # Ažuriraj Euler grafike
        line_roll.set_data(time_data[:frame+1], roll_data[:frame+1])
        line_pitch.set_data(time_data[:frame+1], pitch_data[:frame+1])
        line_yaw.set_data(time_data[:frame+1], yaw_data[:frame+1])
        
        # Ažuriraj markere
        marker_roll.set_data([time_data[frame]], [roll_data[frame]])
        marker_pitch.set_data([time_data[frame]], [pitch_data[frame]])
        marker_yaw.set_data([time_data[frame]], [yaw_data[frame]])
        
        return []
    
    ani = animation.FuncAnimation(fig, update, frames=num_frames,
                                   init_func=init, blit=False, interval=interval_ms)
    
    plt.tight_layout()
    
    if output_file:
        print(f"Čuvanje animacije u {output_file}...")
        try:
            # Calculate FPS for real-time playback
            save_fps = 1000.0 / interval_ms
            print(f"Saving at {save_fps:.2f} fps for real-time playback")
            ani.save(output_file, writer='pillow', fps=save_fps)
            print(f"Animacija sačuvana: {output_file}")
        except Exception as e:
            print(f"Greška pri čuvanju animacije: {e}")
            print("Pokušajte instalirati: pip install pillow")
    
    return fig, ani


def interactive_3d_view(df, frame_index=0):
    """Prikazuje interaktivni 3D prikaz za određeni frame."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Uzmi podatke za frame
    qw = df.iloc[frame_index]['qw']
    qx = df.iloc[frame_index]['qx']
    qy = df.iloc[frame_index]['qy']
    qz = df.iloc[frame_index]['qz']
    
    roll = df.iloc[frame_index]['roll']
    pitch = df.iloc[frame_index]['pitch']
    yaw = df.iloc[frame_index]['yaw']
    t = df.iloc[frame_index]['time']
    
    # Rotaciona matrica
    rot_matrix = quaternion_to_rotation_matrix(qw, qx, qy, qz)
    
    # Kutija
    base_vertices = create_box_vertices()
    rotated_vertices = rotate_vertices(base_vertices, rot_matrix)
    faces = get_box_faces(rotated_vertices)
    
    face_colors = ['cyan', 'cyan', 'magenta', 'magenta', 'yellow', 'yellow']
    box = Poly3DCollection(faces, alpha=0.7, linewidths=1, edgecolors='black')
    box.set_facecolor(face_colors)
    ax.add_collection3d(box)
    
    # Ose objekta
    draw_axes(ax, rot_matrix)
    
    # Referentne ose
    ax.quiver(0, 0, 0, 2, 0, 0, color='r', alpha=0.3, arrow_length_ratio=0.1, label='X ref')
    ax.quiver(0, 0, 0, 0, 2, 0, color='g', alpha=0.3, arrow_length_ratio=0.1, label='Y ref')
    ax.quiver(0, 0, 0, 0, 0, 2, color='b', alpha=0.3, arrow_length_ratio=0.1, label='Z ref')
    
    ax.set_xlim([-2.5, 2.5])
    ax.set_ylim([-2.5, 2.5])
    ax.set_zlim([-2.5, 2.5])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Orijentacija t={t:.2f}s\nRoll={roll:.1f}° Pitch={pitch:.1f}° Yaw={yaw:.1f}°')
    
    return fig


def main():
    if len(sys.argv) < 2:
        print("Korišćenje: python3 visualize_motion.py <orientation_data.csv> [opcije]")
        print("\nOpcije:")
        print("  --euler        Samo grafik Euler uglova")
        print("  --trajectory   Samo 3D trajektorija")
        print("  --animate      Kreiranje animacije (može biti sporo)")
        print("  --frame N      Prikaz jednog frame-a (N = broj frame-a)")
        print("  --skip N       Preskoči N frameova u animaciji (default: 10)")
        print("\nPrimer:")
        print("  python3 visualize_motion.py orientation_data.csv")
        print("  python3 visualize_motion.py orientation_data.csv --animate --skip 20")
        sys.exit(1)
    
    filename = sys.argv[1]
    
    # Parse opcije
    show_euler = '--euler' in sys.argv or len(sys.argv) == 2
    show_trajectory = '--trajectory' in sys.argv or len(sys.argv) == 2
    show_animation = '--animate' in sys.argv or len(sys.argv) == 2
    
    skip_frames = 10
    if '--skip' in sys.argv:
        idx = sys.argv.index('--skip')
        if idx + 1 < len(sys.argv):
            skip_frames = int(sys.argv[idx + 1])
    
    frame_index = None
    if '--frame' in sys.argv:
        idx = sys.argv.index('--frame')
        if idx + 1 < len(sys.argv):
            frame_index = int(sys.argv[idx + 1])
    
    # Učitaj podatke
    print(f"Učitavanje podataka iz {filename}...")
    df = load_data(filename)
    print(f"Učitano {len(df)} uzoraka.")
    print(f"Vremenski opseg: {df['time'].min():.2f}s - {df['time'].max():.2f}s")
    
    # Prikaži statistiku
    print("\n--- Statistika orijentacije ---")
    print(f"Roll:  min={df['roll'].min():.1f}°, max={df['roll'].max():.1f}°")
    print(f"Pitch: min={df['pitch'].min():.1f}°, max={df['pitch'].max():.1f}°")
    print(f"Yaw:   min={df['yaw'].min():.1f}°, max={df['yaw'].max():.1f}°")
    
    # Prikaz jednog frame-a
    if frame_index is not None:
        print(f"\nPrikaz frame-a {frame_index}...")
        interactive_3d_view(df, frame_index)
        plt.show()
        return
    
    # Plotovi
    if show_euler:
        print("\nKreiranje grafika Euler uglova...")
        plot_euler_angles(df, save_path='../../data/euler_angles.png')
    
    if show_trajectory:
        print("\nKreiranje 3D trajektorije...")
        plot_3d_orientation_trajectory(df, save_path='../../data/orientation_trajectory.png')
    
    if show_animation:
        print(f"\nKreiranje animacije (skip={skip_frames})...")
        fig, ani = create_animation(df, output_file='../../data/motion_animation.gif', 
                                    fps=30, skip_frames=skip_frames)
    
    print("\nPrikazivanje grafika...")
    plt.show()


if __name__ == "__main__":
    main()
