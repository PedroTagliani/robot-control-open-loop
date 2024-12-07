import serial
import time
from modules.interpolation import position_quintic_interpolation
from modules.inverse_kinematics import inverse_kinematics
from modules.forward_kinematics import forward_kinematics, forward_kinematics_xyz_euler_repr
from math import radians
import numpy as np

# Define the home joint poosition of the robot
homeJointAngles = [90.0, 80.0, 0.0, 120.0]

# Define the position [x, y, z, pitchAngle] in the home position
homePosition = forward_kinematics_xyz_euler_repr(homeJointAngles)

# The encoders' placement isn't set exactly as it should be, so the measured obtained from them won't be so precise
# Hence, it is required to set a margin error to compare them with another angles
# Unit = [deg]
marginError = 10

# Save the last pose and angles (this needs to be updated in each movement)
lastPose = homePosition.copy()
lastAngles = homeJointAngles.copy()

def start_communication_with_robot(port):
    # Create the serial communication object
    ser = serial.Serial(port, 115200)
    time.sleep(4)  # Wait for the connection to establish

    return ser

def is_robot_awake(ser):

    # Ask the robot if it is turned on by choosing option "e"
    ser.write(f"{"e"}\n".encode("utf-8"))

    # Verify that the robot is turned on
    while True:
        if ser.in_waiting > 0:  # Check if there's incoming data
            response = ser.readline().decode().strip()  # Read the line, decode it to string, and strip whitespace

            if response == "on":
                print("El robot está prendido.")
                return True
                # Ok, it's turned on 
            else:
                print("El robot está apagado o no responde.")
                quit()

def read_encoders(ser):
    
    # Send a message to ask for the encoders
    ser.write("a\n".encode("utf-8"))

    while True:
        # Read a line from the serial connection
        if ser.in_waiting > 0:  # Check if there's incoming data
            # serial_port.in_waiting -----------> Indicate the number of bytes currently available in the input buffer
            response = ser.readline().decode().strip()  # Read the line, decode it to string, and strip whitespace
            qValues = list(map(float, response.split(',')))
            break
    
    print(f"Lectura de encoders: {qValues}")

    return qValues

def is_robot_home():

    # Establish the serial communication between the robot and the ESP32
    ser = start_communication_with_robot('COM5')

    awake = is_robot_awake(ser)

    if awake:

        # Get the current angles
        currentJointAngles = read_encoders(ser)

        # Close serial port
        ser.close()

        # We obtained the current angles, now we need to check if they correspond to the home position
        boolCheckq1 = (currentJointAngles[0] <= homeJointAngles[0] + marginError) and (currentJointAngles[0] >= homeJointAngles[0] - marginError)
        boolCheckq2 = (currentJointAngles[1] <= homeJointAngles[1] + marginError) and (currentJointAngles[1] >= homeJointAngles[1] - marginError)
        boolCheckq3 = (currentJointAngles[2] <= homeJointAngles[2] + marginError) and (currentJointAngles[2] >= homeJointAngles[2] - marginError)
        boolCheckq4 = (currentJointAngles[3] <= homeJointAngles[3] + marginError) and (currentJointAngles[3] >= homeJointAngles[3] - marginError)

        # If all the values are within the magin error, then return true. Otherwise, false
        if (boolCheckq1 and boolCheckq2 and boolCheckq3 and boolCheckq4):
            print('El robot está en posición de home.')
            print(f"El robot se encuentra en: {lastPose} ({lastAngles}).\n")
            return True
        else:
            print('El robot no está en posición de home.')
            return False
    else:
        print('El robot no está encendido.')
        return False

def send_angle(ser, point, option):

    print('Iniciando movimiento...')

    # Send a message to tell the robot a movement action is coming
    ser.write(f"{option}\n".encode("utf-8"))

    # Make sure the robot understood the choice
    while True:
        if ser.in_waiting > 0:  # Check if there's incoming data
            response = ser.readline().decode().strip()  # Read the line, decode it to string, and strip whitespace
            
            if response == "ok":
                break
                # Ok, we can start to send the angles
            else:
                print("El robot no entendió la opción seleccionada.")

    if (option == "b"):

        x = point[0]
        y = point[1]
        z = point[2]
        pitchAngle = radians(point[3])

        # print([x,y,z,pitchAngle])

        # Compute the inverse kinematics to obtain the joint angles
        # [x, y, z, pitchAngle] to [q1, q2, q3, q4]
        desiredJointAngles = inverse_kinematics(x, y, z, pitchAngle)

        # Send the calculated angles to the robot
        anglesToSend = ",".join(map(str, desiredJointAngles)) + "\n"
        ser.write(anglesToSend.encode('utf-8'))

        # It's needed to wait till the robot confirms the movement has been performed
        while True:
            # Read a line from the serial connection
            if ser.in_waiting > 0:  # Check if there's incoming data
                response = ser.readline().decode().strip()  # Read the line, decode it to string, and strip whitespace
                if response == "done":
                    break
                else:
                    print('El robot no llegó a destino. Reiniciar la partida.')
                    quit()
        
        # Send a message to notify the robot that the trajectory is complete
        ser.write("completed\n".encode("utf-8"))

    elif (option == "d"):

        # It's needed to wait till the robot confirms the movement has been performed
        while True:
            # Read a line from the serial connection
            if ser.in_waiting > 0:  # Check if there's incoming data
                response = ser.readline().decode().strip()  # Read the line, decode it to string, and strip whitespace

                if response == "done":
                    break
                else:
                    print('El robot no llegó a destino. Reiniciar la partida.')

    print('Movimiento finalizado.')
    qval = read_encoders(ser)
    
    lastPose = point
    lastAngles = inverse_kinematics(point[0], point[1], point[2], radians(point[3]))

    print(f"El robot se encuentra en: {lastPose} ({lastAngles}).\n")



def go_to_point(point):

    # Establish the serial communication between the robot and the ESP32
    ser = start_communication_with_robot('COM5')

    # Check if the robot has been turned on
    isAwake = is_robot_awake(ser)

    if isAwake:
        
        # Point = [x, y, z, pitchAngle]

        send_angle(ser, point, "b")
        
        # Close serial port
        ser.close()
    
    else:
        print("El robot no está en la posición de home. Tiene que reiniciar el juego y acomodar el brazo.")

def go_back_home():

    # Establish the serial communication between the robot and the ESP32
    ser = start_communication_with_robot('COM5')

    # Check if the robot has been turned on
    isAwake = is_robot_awake(ser)

    if isAwake:
        
        # Point = [x, y, z, pitchAngle]

        send_angle(ser, homePosition, "d")
        
        # Close serial port
        ser.close()
    
    else:
        print("El robot no está en la posición de home. Tiene que reiniciar el juego y acomodar el brazo.")


if __name__ == '__main__':

    isHome = is_robot_home()

    if isHome:
        keepGoing = True
        while keepGoing:
            x1 = float(input("Ingrese x1 [cm]: "))
            y1 = float(input("Ingrese y1 [cm]: "))
            z1 = float(input("Ingrese z1 [cm]: "))
            pitchAngle1 = float(input("Ingrese pitchAngle1 [°]: "))

            # q1 = float(input("Ingrese q1 [°]: "))
            # q2 = float(input("Ingrese q2 [°]: "))
            # q3 = float(input("Ingrese q3 [°]: "))
            # q4 = float(input("Ingrese q4 [°]: "))

            pose1 = [x1, y1, z1, pitchAngle1]
            # pose1 = forward_kinematics_xyz_euler_repr([q1, q2, q3, q4])

            # [28.12, -10.24, 21.48, -32.0] ([70, 60, 40, 70])
            # [27.35, -9.95, 13.03, -2.0] ([70, 50, 46, 43])
            # [23.92, 13.81, 14.3, -8.0] ([120, 60, 30, 56])
            # [24.58, 0.0, 13.6, 0.0] ([90, 70, 20, 48])
            # [24.98, 9.09, 12.66, 0.0] ([110, 60, 30, 48])

            go_to_point(pose1)

            decision = input("¿Desea ingresar otro punto? Escriba 'si' o 'no': ")

            if decision == 'no':
                print("Finaliza el programa. Antes de terminar, el robot vuelve al home.")
                keepGoing = False
                go_back_home()
            elif decision == 'si':
                print("Seguimos...")
                decision2 = input("¿Volver al home? Escriba 'si' o 'no': ")
                if decision2 == 'si':
                    print('El robot vuelve al home.')
                    go_back_home()
                elif decision2 == 'no':
                    print('El robot no vuelve al home.')
                else:
                    print('Opción inválida se vuelve al home.')
                    go_back_home()
            else:
                print("Opción inválida. Se termina el programa.")
                keepGoing = False
    else:
        quit()