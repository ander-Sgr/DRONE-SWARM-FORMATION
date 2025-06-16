import airsim
import time
import math
import numpy as np
from visualize_formations import *


def get_navigation_offset(client: airsim.MultirotorClient, drone_name):
    # Pose visual (posición real en el mundo)
    visual_pose = client.simGetObjectPose(drone_name).position
    visual_pos = np.array([visual_pose.x_val, visual_pose.y_val, visual_pose.z_val])

    # Posición estimada por AirSim (estado interno del dron)
    nav_state = client.getMultirotorState(vehicle_name=drone_name)
    nav_pose = nav_state.kinematics_estimated.position
    nav_pos = np.array([nav_pose.x_val, nav_pose.y_val, nav_pose.z_val])

    # Offset entre la posición visual y la posición interna
    offset = visual_pos - nav_pos
    return offset


def _fly_to_positions(
    client: airsim.MultirotorClient, target_positions: dict, drone_names, speed: float
):
    for drone_name in drone_names:
        offset = get_navigation_offset(client, drone_name)
        visual_target = target_positions[drone_name]
        corrected_target = visual_target - offset

        print(f"Moviendo {drone_name} a {corrected_target}")
        client.moveToPositionAsync(
            corrected_target[0],
            corrected_target[1],
            corrected_target[2],
            velocity=speed,
            vehicle_name=drone_name,
        ).join()


def fly_triangle(
    client: airsim.MultirotorClient,
    drones_in_formation: list,
    center_xyz: tuple,
    speed: float,
    lado: float = 10.0,
):
    if len(drones_in_formation) < 3:
        print(
            "[ERROR] Se necesitan al menos 3 drones para un triángulo. Se proporcionaron: ",
            drones_in_formation,
        )
        return

    h = (math.sqrt(3) / 2) * lado
    cx, cy, cz = center_xyz

    # Posiciones visuales del triángulo equilátero
    pos_dict = {
        drones_in_formation[0]: np.array([cx, cy + (2 / 3) * h, cz]),
        drones_in_formation[1]: np.array([cx - (lado / 2), cy - (1 / 3) * h, cz]),
        drones_in_formation[2]: np.array([cx + (lado / 2), cy - (1 / 3) * h, cz]),
    }

    # Mover cada dron a su posición deseada
    _fly_to_positions(client, pos_dict, drones_in_formation, speed)

    print(f"[INFO] Formación Triángulo comandada para: {list(pos_dict.keys())}")


def fly_rhombus(
    client: airsim.MultirotorClient,
    drones_in_formation: list,
    center_xyz: tuple,
    speed: float,
    diag1: float = 10.0,  # Diagonal horizontal
    diag2: float = 6.0,
):  # Diagonal vertical
    if len(drones_in_formation) < 4:
        print(
            "[ERROR] Se necesitan al menos 4 drones para un rombo. Se proporcionaron: ",
            drones_in_formation,
        )
        return

    cx, cy, cz = center_xyz
    pos_dict = {
        drones_in_formation[0]: np.array([cx, cy + diag2 / 2, cz]),  # Arriba
        drones_in_formation[1]: np.array([cx + diag1 / 2, cy, cz]),  # Derecha
        drones_in_formation[2]: np.array([cx, cy - diag2 / 2, cz]),  # Abajo
        drones_in_formation[3]: np.array([cx - diag1 / 2, cy, cz]),  # Izquierda
    }
    _fly_to_positions(client, pos_dict, drones_in_formation, speed)
    print(f"[INFO] Formación Rombo comandada para: {list(pos_dict.keys())}")


def fly_circle(
    client: airsim.MultirotorClient,
    drones_in_formation: list,
    radius: float,
    center,
    speed
):
    positions = {}
    angle_step = 2 * np.pi / len(drones_in_formation)
    for i in range(len(drones_in_formation)):
        angle = i * angle_step
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        z = center[2]
        drone_name = drones_in_formation[i]
        positions[drone_name] = np.array([x, y, z])
    _fly_to_positions(client, positions, drones_in_formation, speed)


def fly_v_formation(
    client: airsim.MultirotorClient,
    drones_in_formation: list,
    center_xyz: tuple,  # Posición del líder
    speed: float,
    leader_drone_name: str,
    spacing_x: float = 2.0,
    spacing_y: float = 2.0,
    rotation_deg: float = 0.0  # Ángulo de rotación en grados
):
    num_drones = len(drones_in_formation)
    if num_drones == 0:
        print("[ERROR] No hay drones para la formación en V.")
        return
    if leader_drone_name not in drones_in_formation:
        print(f"[ERROR] Dron líder '{leader_drone_name}' no está en la lista. Usando {drones_in_formation[0]} como líder.")
        leader_drone_name = drones_in_formation[0]

    pos_dict = {}
    leader_cx, leader_cy, leader_cz = center_xyz
    pos_dict[leader_drone_name] = (leader_cx, leader_cy, leader_cz)

    # Convertir grados a radianes para la rotación
    theta = math.radians(rotation_deg)
    rotation_matrix = np.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta),  math.cos(theta)]
    ])

    follower_index = 0
    for drone_name in drones_in_formation:
        if drone_name == leader_drone_name:
            continue

        wing_index = (follower_index // 2) + 1
        is_left = (follower_index % 2 == 0)

        local_x = -wing_index * spacing_x if is_left else wing_index * spacing_x
        local_y = -wing_index * spacing_y

        # Rotar el vector [local_x, local_y]
        rotated_xy = rotation_matrix @ np.array([local_x, local_y])
        tx = leader_cx + rotated_xy[0]
        ty = leader_cy + rotated_xy[1]
        tz = leader_cz

        pos_dict[drone_name] = (tx, ty, tz)
        follower_index += 1

    _fly_to_positions(client, pos_dict, drones_in_formation, speed)
    print(
        f"[INFO] Formación V rotada {rotation_deg}° comandada. Líder: {leader_drone_name}"
    )



def fly_star_formation(client: airsim.MultirotorClient,
                       drones_in_formation: list,
                       center_xyz: tuple,
                       speed: float,
                       outer_radius: float = 12.0, # Radio de las puntas
                       inner_radius_ratio: float = 0.5, # Proporción del radio interior respecto al exterior
                       num_points: int = 5, # Número de puntas de la estrella
                       rotation_deg: float = 90.0): # Rotación inicial para que una punta apunte "hacia adelante" (eje Y positivo)
    """
    Formación de Estrella Regular (por ejemplo, 5 puntas).
    Requiere al menos 2 * num_points drones para una estrella simétrica.
    """
    num_drones = len(drones_in_formation)
    if num_drones < num_points * 2:
        print(f"[WARN] Se necesitan al menos {num_points * 2} drones para una estrella de {num_points} puntas simétrica. Se proporcionaron: {num_drones}. La formación será incompleta.")
    if num_drones == 0:
        print("[ERROR] No hay drones para formar una estrella.")
        return

    cx, cy, cz= center_xyz
    z = cz
    pos_dict = {}
    inner_radius = outer_radius * inner_radius_ratio

    for i in range(num_points):
        # Ángulo para el punto exterior (punta)
        outer_angle_deg = rotation_deg + (i * (360 / num_points))
        x_outer = cx + outer_radius * math.cos(math.radians(outer_angle_deg))
        y_outer = cy + outer_radius * math.sin(math.radians(outer_angle_deg))

        # Ángulo para el punto interior (valle entre puntas)
        inner_angle_deg = rotation_deg + (i * (360 / num_points)) + (360 / (num_points * 2)) # Medio camino entre puntas
        x_inner = cx + inner_radius * math.cos(math.radians(inner_angle_deg))
        y_inner = cy + inner_radius * math.sin(math.radians(inner_angle_deg))

        # Asignar drones
        if 2 * i < num_drones: # Dron para la punta
            pos_dict[drones_in_formation[2 * i]] = (x_outer, y_outer, z)
        if (2 * i + 1) < num_drones: # Dron para el punto interior
            pos_dict[drones_in_formation[2 * i + 1]] = (x_inner, y_inner, z)

    _fly_to_positions(client, pos_dict, drones_in_formation, speed)
    print(f"[INFO] Formación Estrella comandada para: {list(pos_dict.keys())}")



def fly_path_circle(
    client: airsim.MultirotorClient,
    drone_name: str,  # Solo un dron para esta función
    path_center_xy: tuple,
    altitude: float,  # Altitud deseada (positiva)
    speed: float,
    radius: float = 10.0,
    segments: int = 20,
    trail_color_rgba: list = [1.0, 0.0, 0.0, 1.0],  # Rojo por defecto
    trail_thickness: float = 5.0,
    trail_duration: float = 60.0,  # Cuánto tiempo permanece el rastro
    is_trail_persistent: bool = True,
):  # Si el rastro debe ser persistente

    print(f"[INFO] Dron {drone_name} iniciando vuelo en círculo con rastro.")

    # 1. Mover el dron a la altitud y al punto de inicio del círculo primero
    # (Opcional, si quieres un movimiento inicial más controlado antes del path)
    # start_angle_rad = 0
    # start_x = path_center_xy[0] + radius * math.cos(start_angle_rad)
    # start_y = path_center_xy[1] + radius * math.sin(start_angle_rad)
    # print(f"[INFO] {drone_name} moviéndose al punto de inicio del círculo: ({start_x:.2f}, {start_y:.2f}, {-abs(altitude):.2f})")
    # client.moveToPositionAsync(start_x, start_y, -abs(altitude), speed, vehicle_name=drone_name).join()
    # time.sleep(1) # Estabilizar

    # 2. Construir la trayectoria para moveOnPathAsync
    path_points_airsim = []
    for i in range(segments + 1):  # +1 para cerrar el círculo
        angle_rad = (2 * math.pi * i) / segments
        wx = path_center_xy[0] + radius * math.cos(angle_rad)
        wy = path_center_xy[1] + radius * math.sin(angle_rad)
        path_points_airsim.append(airsim.Vector3r(wx, wy, -abs(altitude)))

    # Dibujar la trayectoria planeada como referencia (opcional)
    client.simPlotLineStrip(
        points=path_points_airsim,
        color_rgba=[0.0, 0.0, 1.0, 0.5],  # Azul semi-transparente para el plan
        thickness=trail_thickness / 2,
        duration=trail_duration,
        is_persistent=is_trail_persistent,
    )

    # 3. Ejecutar el movimiento a lo largo del camino
    print(f"[INFO] {drone_name} iniciando moveOnPathAsync para el círculo.")
    # lookahead = -1 (controlador PID simple) adaptative_lookahead = 1 (más suave)
    path_future = client.moveOnPathAsync(
        path_points_airsim,
        speed,
        timeout_sec=300,  # Tiempo largo para completar el path
        vehicle_name=drone_name,
        lookahead=-1,  # -1 para modo PID, >0 para lookahead basado en trayectoria
        adaptive_lookahead=1,
    )  # 0 para lookahead fijo, 1 para adaptativo

    # 4. Dibujar el rastro mientras el dron se mueve
    # Esto es una forma simple. Para un rastro muy suave, podrías necesitar un hilo/timer.
    last_plotted_pos = client.getMultirotorState(
        vehicle_name=drone_name
    ).kinematics_estimated.position

    # Bucle para dibujar rastro mientras el dron se mueve en el path
    while not path_future:
        current_pos = client.getMultirotorState(
            vehicle_name=drone_name
        ).kinematics_estimated.position
        # Comprobar si el dron se ha movido lo suficiente para dibujar un nuevo segmento
        # Evita dibujar demasiadas líneas si el dron está casi quieto o el sleep es muy corto.
        # Puedes usar np.linalg.norm(current_pos.to_numpy_array()[:2] - last_plotted_pos.to_numpy_array()[:2]) > 0.1 (por ejemplo)

        # Dibujar línea desde la última posición a la actual
        client.simPlotLineList(
            points=[last_plotted_pos, current_pos],
            color_rgba=trail_color_rgba,
            thickness=trail_thickness,
            duration=trail_duration,
            is_persistent=is_trail_persistent,
        )
        last_plotted_pos = current_pos
        time.sleep(0.1)  # Frecuencia de actualización del rastro

    path_future.join()  # Asegurarse de que el movimiento ha terminado.
    print(f"[INFO] {drone_name} completó el círculo con rastro.")
