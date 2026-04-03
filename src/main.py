from robot_2dof import PlanarRobot2DOF
import numpy as np

def main():
    # 1. Inisialisasi robot dengan panjang L1=4, L2=3
    robot = PlanarRobot2DOF(l1=4, l2=3)

    print("====================================")
    print("        FORWARD KINEMATICS          ")
    print("====================================")
    # Konversi derajat ke radian
    t1, t2 = np.radians(30), np.radians(45)
    print("Input Sudut:")
    print(f"Theta 1 = 30° ({t1:.2f} rad)")
    print(f"Theta 2 = 45° ({t2:.2f} rad)")
    
    x, y = robot.forward_kinematics(t1, t2, plot=True)
    print(f"Hasil Posisi End-Effector: x = {x:.2f}, y = {y:.2f}\n")

    print("====================================")
    print("        INVERSE KINEMATICS          ")
    print("====================================")
    target_x, target_y = 5.0, 4.0
    print(f"Target Posisi End-Effector: x = {target_x}, y = {target_y}")
    
    try:
        sol_down, sol_up = robot.inverse_kinematics(target_x, target_y, plot=True)
        print("Solusi 1 (Elbow Down):")
        print(f"  Theta 1 = {np.degrees(sol_down[0]):.2f}°, Theta 2 = {np.degrees(sol_down[1]):.2f}°")
        print("Solusi 2 (Elbow Up):")
        print(f"  Theta 1 = {np.degrees(sol_up[0]):.2f}°, Theta 2 = {np.degrees(sol_up[1]):.2f}°")
    except ValueError as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()