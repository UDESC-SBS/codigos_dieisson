import numpy as np

# Definição dos parâmetros de DH
a1 = 2.0
a2 = 1.5
d3 = 1.0

# Definição das variáveis simbólicas
theta1, theta2, theta3 = np.pi/4, np.pi/3, np.pi/6
alpha1, alpha2 = np.pi/2, np.pi/2

# Definição das matrizes de transformação homogêneas para cada junta
H1 = np.array([
    [np.cos(theta1), -np.sin(theta1)*np.cos(alpha1), np.sin(theta1)*np.sin(alpha1), a1*np.cos(theta1)],
    [np.sin(theta1), np.cos(theta1)*np.cos(alpha1), -np.cos(theta1)*np.sin(alpha1), a1*np.sin(theta1)],
    [0, np.sin(alpha1), np.cos(alpha1), 0],
    [0, 0, 0, 1]
])

H2 = np.array([
    [np.cos(theta2), -np.sin(theta2)*np.cos(alpha2), np.sin(theta2)*np.sin(alpha2), a2*np.cos(theta2)],
    [np.sin(theta2), np.cos(theta2)*np.cos(alpha2), -np.cos(theta2)*np.sin(alpha2), a2*np.sin(theta2)],
    [0, np.sin(alpha2), np.cos(alpha2), 0],
    [0, 0, 0, 1]
])

H3 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, d3],
    [0, 0, 0, 1]
])

# Cálculo da matriz de transformação homogênea total
T = H1 @ H2 @ H3
print(T)
# Cálculo da posição da extremidade do efetuador final em relação à base do robô
x, y, z, _ = T[:, 3]

# Cálculo da orientação da extremidade do efetuador final em relação à base do robô
R = T[:3, :3]
R = R / np.linalg.norm(R, axis=0)

print("Posição da extremidade do efetuador final em relação à base do robô:")
print(f"x = {x}, y = {y}, z = {z}")
print("Orientação da extremidade do efetuador final em relação à base do robô:")
print(R)
