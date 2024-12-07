from math import degrees, radians, sin, cos, sqrt, atan2, pi

# Variables relacionadas a cada una de las longitudes intervinientes en los cálculos (todo en centímetros)
l1 = 14.638
l2 = 12.725
l3 = 10.222
l4 = 11.200
l5 = 8.200

def inverse_kinematics(px, py, pz, pitchAngle): # El pitchAngle recibido está en radianes
    try:
        r = sqrt((px**2) + ((-py)**2))

        # Expresión para la variable articular q1:
        q1 = atan2(px,-py) # Prestar atención al signo de py, el cual está relacionado al lugar desde donde parte q1

        l5x = sin((-pitchAngle))*l5 # Se le cambia el signo al ángulo pitch porque Y0 se analizó desde atrás
        l5y = cos((-pitchAngle))*l5

        l4y = sin((-pitchAngle))*l4
        l4x = cos((-pitchAngle))*l4

        j = pz + l5y - l4y

        b = sqrt((l1-j)**2 + (r-l4x-l5x)**2)

        # alpha1 = sp.atan((l1-j)/(r-l4x-l5x))
        alpha1 = atan2(l1-j,r-l4x-l5x)

        # beta2 = sp.atan((l1-j)/(r-l4x-l5x))
        beta2 = atan2(l1-j,r-l4x-l5x)

        cos_alpha2 = (l2**2 + b**2 - l3**2)/(2*l2*b)

        cos_theta3 = (l2**2 + l3**2 -b**2)/(2*l2*l3)

        cos_beta1 = (l3**2 + b**2 - l2**2)/(2*l3*b)

        alpha2_sol1 = atan2(sqrt(1 - cos_alpha2**2), cos_alpha2)
        alpha2_sol2 = atan2(-sqrt(1 - cos_alpha2**2), cos_alpha2)

        theta3_sol1 = atan2(sqrt(1 - cos_theta3**2), cos_theta3)
        theta3_sol2 = atan2(-sqrt(1 - cos_theta3**2), cos_theta3)

        beta1_sol1 = atan2(sqrt(1 - cos_beta1**2), cos_beta1)
        beta1_sol2 = atan2(-sqrt(1 - cos_beta1**2), cos_beta1)

        theta2 = alpha2_sol1 - alpha1

        theta4 = beta1_sol1 + beta2

        # Expresiones para las variables articulares q2, q3 y q4
        q2 = theta2
        q3 = (pi/2 + radians(28)) - (pi - theta3_sol1) # El pi/2 + radians(28) corresponde a la nueva posición de 0° del servomotor en relación a los 0° originales según el dibujo
        q4 = theta4 + (-pitchAngle) # Se le cambia el signo al ángulo pitch porque Y0 se analizó desde atrás
        q4 = q4 + radians(20) # Due to the offset of the gripper servomotor
        # Ojo con el tema de los ángulos q. En el dibujo de cinemática inversa, prestar atención a dónde está definido el cero y el sentido de giro para cada q.
        # Lo que está plasmado en el dibujo que hicimos de cinemática inversa es diferente a este caso. Ahí q2 partía desde la vertical, en este caso parte desde la horizontal,
        # por lo que q2 va a ser igual a theta2. Por otro lado, q1 y q3 también son diferentes. Solamente coincide q4 en ambos casos.
        # O sea, todos los cálculos son los mismos, menos el cálculo de los q.

        # Se pasa de radianes a grados
        q1 = degrees(q1)
        q2 = degrees(q2)
        q3 = degrees(q3)
        q4 = degrees(q4)

        return [q1,q2,q3,q4]
    except:
        print('Esa pose no es alcanzable.')
        quit() # Revisar esta función

if __name__ == '__main__':
    # Ejemplo de aplicación
    px = 24.98
    py = 9.09
    pz = 12.66

    # Se define la orientación del efector final respecto a la terna base (solo es posible controlar el ángulo de cabeceo):
    pitchAngle = radians(0)

    qList = inverse_kinematics(px, py, pz, pitchAngle)

    print('Valores de las variables articulares para la pose dada del efecto final:')

    for index,q in enumerate(qList):
        print(f'    q{index+1}: {q}°.')