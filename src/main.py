from robot_2dof import PlanarRobot2DOF
import numpy as np

def main():
    # Inisialisasi robot dengan L1=4, L2=3
    robot = PlanarRobot2DOF(l1=4, l2=3)

    print("=================================================")
    print("   PERBANDINGAN LANGSUNG: FK vs IK               ")
    print("=================================================")
    print("Membuka jendela visualisasi...")
    
    # Kita berikan input sudut secara acak (misal: sendi 1 naik 30°, sendi 2 turun -45°)
    theta1 = np.radians(30)
    theta2 = np.radians(-45)
    
    # Fungsi ini akan menjalankan FK, lalu mengambil hasilnya untuk menjalankan IK,
    # dan menampilkan keduanya bersebelahan agar mudah dianalisis.
    robot.compare_fk_ik(theta1, theta2)
    
    print("Visualisasi selesai ditutup.")

if __name__ == "__main__":
    main()