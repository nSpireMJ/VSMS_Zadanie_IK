import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import time

l1 = 2.0
l2 = 2.0

def inverse_kinematics(x, y):
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)

    if np.abs(cos_theta2) > 1:
        raise ValueError("Cieľ je mimo rozsah robota.")

    theta2 = np.arccos(cos_theta2)
    k1 = l1 + l2 * np.cos(theta2)
    k2 = l2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return theta1, theta2

def forward_kinematics(theta1, theta2):
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    return (0, x1, x2), (0, y1, y2)

# Trajektória kruhu
def generate_circle_trajectory(radius=1.5, num_points=100):
    theta = np.linspace(0, 2 * np.pi, num_points)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    return x, y

import numpy as np

def generate_square_trajectory(size=2.0, num_points_per_side=25):
    half = size / 2
    # Spodná strana (zľava doprava)
    bottom = [(x, -half) for x in np.linspace(-half, half, num_points_per_side)]
    # Pravá strana (zdola nahor)
    right = [(half, y) for y in np.linspace(-half, half, num_points_per_side)]
    # Horná strana (sprava doľava)
    top = [(x, half) for x in np.linspace(half, -half, num_points_per_side)]
    # Ľavá strana (zhora nadol)
    left = [(-half, y) for y in np.linspace(half, -half, num_points_per_side)]
    # Spojenie všetkých strán
    square = bottom + right + top + left
    # Rozdelenie na X a Y pole
    x, y = zip(*square)
    return np.array(x), np.array(y)

def compute_position_error(x_ref, y_ref, x_ach, y_ach):
    errors = np.sqrt((np.array(x_ref) - np.array(x_ach))**2 + (np.array(y_ref) - np.array(y_ach))**2)
    return errors





# Sledovanie trajektórie
def track_trajectory(x_traj, y_traj):
    achieved_x, achieved_y = [], []
    for x, y in zip(x_traj, y_traj):
        try:
            theta1, theta2 = inverse_kinematics(x, y)
            x_fk, y_fk = forward_kinematics(theta1, theta2)
            achieved_x.append(x_fk[-1])
            achieved_y.append(y_fk[-1])
        except Exception as e:
            print(f"Chyba pre bod ({x:.2f}, {y:.2f}): {e}")
            achieved_x.append(np.nan)
            achieved_y.append(np.nan)
    return achieved_x, achieved_y

# --- Hlavný program ---
if __name__ == '__main__':
    offset_x = 2.5  # offset v x pre posun trajektórie
    # 1. KRUH
    # Kruh posunutý dopredu
    x_circle, y_circle = generate_circle_trajectory(radius=1.5)
    x_circle += offset_x  # posun pred robota
    ach_x_circle, ach_y_circle = track_trajectory(x_circle, y_circle)


    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")
    p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=40, cameraPitch=-30, cameraTargetPosition=[2.5, 0, 0])

    # Vytvorenie jednoduchého ramena (2 kĺby, 2 články)
    base = p.loadURDF("urdf/2dof_planar_robot.urdf", basePosition=[0, 0, 0])  # dummy base pre ukotvenie

    # Vykreslenie celej trajektórie dopredu (napr. ako červená tenká čiara)
    for i in range(len(x_circle) - 1):
        p.addUserDebugLine(
            [x_circle[i], y_circle[i], 0.05],
            [x_circle[i + 1], y_circle[i + 1], 0.05],
            [1, 0, 0], 1
        )

    for x, y in zip(x_circle, y_circle):
        try:
            theta1, theta2 = inverse_kinematics(x, y)
            x_coords, y_coords = forward_kinematics(theta1, theta2)



            # Vykreslenie ramena ako 2 čiary
            p.addUserDebugLine([0, 0, 0.1], [x_coords[1], y_coords[1], 0.1], [0, 1, 0], 5, lifeTime = 0.15)
            p.addUserDebugLine([x_coords[1], y_coords[1], 0.1], [x_coords[2], y_coords[2], 0.1], [1, 0, 0], 5, lifeTime = 0.15)
            p.stepSimulation()
            time.sleep(0.02)
        except Exception as e:
            print(f"Chyba: {e}")
            continue

    plt.figure(figsize=(8, 6))
    plt.plot(x_circle, y_circle, 'g--', label='Predpísaná trajektória (kruh)')
    plt.plot(ach_x_circle, ach_y_circle, 'r-', label='Dosiahnutá trajektória')
    plt.title('Sledovanie trajektórie pomocou IK (kruh)')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

    p.disconnect()

    

    # 2. ŠTVOREC
    # Štvorec posunutý dopredu
    x_square, y_square = generate_square_trajectory(size=2.0)
    x_square += offset_x  # posun pred robota
    ach_x_square, ach_y_square = track_trajectory(x_square, y_square)

    



    physicsClient = p.connect(p.GUI)
    
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")
    p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=40, cameraPitch=-30, cameraTargetPosition=[2.5, 0, 0])

    # Vytvorenie jednoduchého ramena (2 kĺby, 2 články)
    base = p.loadURDF("urdf/2dof_planar_robot.urdf", basePosition=[0, 0, 0])  # dummy base pre ukotvenie

    for i in range(len(x_square) - 1):
        p.addUserDebugLine(
            [x_square[i], y_square[i], 0.05],
            [x_square[i + 1], y_square[i + 1], 0.05],
            [1, 0, 0], 1
        )

    for x, y in zip(x_square, y_square):
        try:
            theta1, theta2 = inverse_kinematics(x, y)
            x_coords, y_coords = forward_kinematics(theta1, theta2)



            # Vykreslenie ramena ako 2 čiary
            p.addUserDebugLine([0, 0, 0.1], [x_coords[1], y_coords[1], 0.1], [0, 1, 0], 5, lifeTime = 0.15)
            p.addUserDebugLine([x_coords[1], y_coords[1], 0.1], [x_coords[2], y_coords[2], 0.1], [1, 0, 0], 5, lifeTime = 0.15)
            p.stepSimulation()
            time.sleep(0.02)
        except Exception as e:
            print(f"Chyba: {e}")
            continue
    
    plt.figure(figsize=(8, 6))
    plt.plot(x_square, y_square, 'g--', label='Predpísaná trajektória (štvorec)')
    plt.plot(ach_x_square, ach_y_square, 'r-', label='Dosiahnutá trajektória')
    plt.title('Sledovanie trajektórie pomocou IK (štvorec)')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

    p.disconnect()

    # Odchylka predpisanej a dosiahnutej trajektorie v kazdom bode

    errors_circle = compute_position_error(x_circle, y_circle, ach_x_circle, ach_y_circle)
    errors_square = compute_position_error(x_square, y_square, ach_x_square, ach_y_square)

    print("Priemerná chyba (kruh):", np.nanmean(errors_circle))
    print("Maximálna chyba (kruh):", np.nanmax(errors_circle))
    print("Priemerná chyba (štvorec):", np.nanmean(errors_square))
    print("Maximálna chyba (štvorec):", np.nanmax(errors_square))

    plt.figure(figsize=(10, 4))
    plt.plot(errors_circle, label='Chyba - kruh')
    plt.plot(errors_square, label='Chyba - štvorec')
    plt.title('Pozičná chyba v čase')
    plt.xlabel('Index bodu na trajektórii')
    plt.ylabel('Chyba')
    plt.legend()
    plt.grid(True)
    plt.show()
