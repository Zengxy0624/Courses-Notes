import numpy as np

# 根据图片重新定义的矩阵角色
A = np.array([
    [-0.02291216, 0.02942084, -0.99930448, 0.81232591],
    [-0.87617726, 0.48077103, 0.03424362, 0.88112258],
    [0.48144412, 0.87635246, 0.01476237, -0.85539508],
    [0.0, 0.0, 0.0, 1.0]
])  # T_global_robot

B = np.array([
    [0.25298867, -0.17481602, -0.95154406, -0.64999270],
    [0.96351883, -0.04326338, 0.26412070, -0.31275932],
    [-0.08733954, -0.98365016, 0.15749338, 0.82314936],
    [0.0, 0.0, 0.0, 1.0]
])  # T_robot_sensor

p_sensor = np.array([0.24044249, 0.53306805, 0.60568494])

p_sensor_homogeneous = np.append(p_sensor, 1)
# 应用修正后的变换顺序 A @ B @ p
p_global_homogeneous = A @ B @ p_sensor_homogeneous

p_global = p_global_homogeneous[:3]

print("The CORRECTED coordinates in the global frame are:")
print(p_global)
print("hello from test.py")
