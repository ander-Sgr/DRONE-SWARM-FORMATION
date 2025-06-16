import airsim
import time
import math
import numpy as np


from formations import (
    fly_triangle, fly_rhombus, fly_v_formation, fly_path_circle,
    fly_circle, fly_star_formation
)

from logger import log_result
from navigation import fly_v_formation_with_avoidance

# current params for the formations
FORMATION_CENTER_XY =  np.array([10, 8, -3.0]) # Centro XYZ en el mundo (ej. 10m delante del origen)
FORMATION_ALTITUDE = 3 # Altitud deseada
SPEED = 4
HOLD_DURATION = 10 # tiempo para mantener la formacion en segundos
ALL_FORMATIONS = ["triangle", "rhombus", "circle", "v", "star"]

# numero de drones a usar
DRONES = [f"Drone{i}" for i in range(1, 11)]

'''
def get_drift(client):
    targets = get_target_positions()

    if not targets:
        print("[ERROR] No se encontraron posiciones objetivo. ¿Se ejecutó una figura?")
        return 0.0

    total_drift = 0
    for name, (tx, ty, tz) in targets.items():
        pos = client.getMultirotorState(vehicle_name=name).kinematics_estimated.position
        dx = pos.x_val - tx
        dy = pos.y_val - ty
        dz = pos.z_val - tz
        drift = math.sqrt(dx**2 + dy**2 + dz**2)
        total_drift += drift

    promedio = total_drift / len(targets)
    return round(promedio, 3) 
'''

# enable the control and arm the drones for the init
def enable_drones(drones, client):
    futures = []
    for drone_name in drones:
        client.enableApiControl(True, drone_name)
        client.armDisarm(True, drone_name)
        print(f"[INFO] {drone_name} despegando a {3}m...")
        futures.append(client.moveToZAsync(3, 3, vehicle_name=drone_name))

    for f in futures:
        f.join()



def select_shape(input_shape):
    if input_shape == "all":
        return ALL_FORMATIONS
    elif input_shape in ALL_FORMATIONS or input_shape == "path_circle":
        return [input_shape]
    else:
        print("[ERROR] Figura no válida.")
        return []


def get_active_drones_for_formation(formation_name, all_drones):
    if formation_name == "triangle":
        return all_drones[:3]
    elif formation_name == "rhombus":
        return all_drones[:4]
    elif formation_name in ["circle", "star"]:
        return all_drones
    elif formation_name == "v":
        return all_drones[:7]
    elif formation_name == "path_circle":
        return [all_drones[0]]
    else:
        return []

        

def run_formation(client, formations):
    for shape in formations:
        print(f"\n[INFO] ----- Iniciando Formación: {shape.upper()} -----")
        drones = get_active_drones_for_formation(shape, DRONES)

        if not drones:
            print(f"[ERROR] No hay drones activos definidos para '{shape}'.")
            continue

        print(f"[INFO] Usando {len(drones)} drones: {drones}")
        start_time = time.time()

        if shape == "triangle":
            fly_triangle(client, drones, FORMATION_CENTER_XY, SPEED)
        elif shape == "rhombus":
            fly_rhombus(client, drones, FORMATION_CENTER_XY, SPEED)
        elif shape == "circle":
            fly_circle(client, drones, 6.0, FORMATION_CENTER_XY, SPEED)
        elif shape == "v":
            fly_v_formation_with_avoidance(client, drones, initial_center=FORMATION_CENTER_XY, speed=3.0)
        elif shape == "star":
            fly_star_formation(client, drones, FORMATION_CENTER_XY, SPEED)
        elif shape == "path_circle":
            pos = client.getMultirotorState(vehicle_name=drones[0]).kinematics_estimated.position
            center = (pos.x_val + 10, pos.y_val)
            fly_path_circle(client, drones[0], center, altitude=FORMATION_ALTITUDE, speed=2, radius=8, segments=25)

        elapsed = time.time() - start_time
        print(f"[INFO] Comandos enviados en {elapsed:.2f}s. Manteniendo formación {HOLD_DURATION}s...")
        time.sleep(HOLD_DURATION)

        # drift = get_drift(client)
        log_result("resultados.csv", {
            "figura": shape,
            "tiempo_cmd_formacion": round(elapsed, 2),
            "tiempo_hold": HOLD_DURATION,
          #  "drift_promedio": drift,
            "n_drones_activos": len(drones),
            "altitud_formacion": FORMATION_ALTITUDE,
            "velocidad_mov": SPEED,
            "centro_formacion_x": FORMATION_CENTER_XY[0],
            "centro_formacion_y": FORMATION_CENTER_XY[1],
        })
        print(f"[OK] Formación '{shape}' completada. Drift medio: m")



def land_and_cleanup(client):
    print("\n[INFO] ----- ATERRIZAJE Y FINALIZACIÓN -----")

    # Guardamos los futuros de aterrizaje para luego esperar todos juntos
    landing_futures = []

    for drone in DRONES:
        try:
            state = client.getMultirotorState(vehicle_name=drone)
            if state.landed_state == airsim.LandedState.Flying:
                print(f"[INFO] Iniciando aterrizaje de {drone}...")
                # Lanzamos el aterrizaje async sin join todavía
                future = client.landAsync(vehicle_name=drone)
                landing_futures.append((drone, future))
            else:
                print(f"[INFO] {drone} ya estaba aterrizado.")
        except Exception as e:
            print(f"[ERROR] Error al iniciar aterrizaje de {drone}: {e}")

    # Esperamos a que terminen todos los aterrizajes
    for drone, future in landing_futures:
        try:
            future.join()
            new_state = client.getMultirotorState(vehicle_name=drone)
            z = new_state.kinematics_estimated.position.z_val
            if new_state.landed_state == airsim.LandedState.Landed or z >= -1.5:
                print(f"[INFO] {drone} aterrizó con z = {z:.2f}")
            else:
                print(f"[WARN] {drone} puede no haber aterrizado del todo (z = {z:.2f})")
        except Exception as e:
            print(f"[ERROR] Error en aterrizaje final de {drone}: {e}")

    # Desarmar y liberar control
    for drone in DRONES:
        client.armDisarm(False, drone)
        client.enableApiControl(False, drone)

    print("[INFO] Todos los drones aterrizados y desarmados correctamente.")


def main():
    client = airsim.MultirotorClient(ip="192.168.1.128")
    client.confirmConnection()
    client.simSetWind(airsim.Vector3r(2, 0, 0))

    # init the drones
    client.reset()
    enable_drones(DRONES, client)
    print("\n[TEST] Comprobando sensores de distancia:")
    
    user_shape = input("Figura (triangle / rhombus / circle / v / star / path_circle / all): ").strip().lower()
    formations = select_shape(user_shape)
    if not formations:
        client.reset()
        return

    run_formation(client, formations)
    land_and_cleanup(client)

if __name__ == "__main__":
    main()