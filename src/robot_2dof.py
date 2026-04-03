import numpy as np
import matplotlib.pyplot as plt

class PlanarRobot2DOF:
    def __init__(self, l1, l2):
        """Inisialisasi panjang link/lengan robot."""
        self.l1 = l1
        self.l2 = l2

    def forward_kinematics(self, theta1, theta2, plot=False):
        """Menghitung posisi end-effector dari sudut sendi (dalam radian)."""
        # Posisi sendi kedua (elbow)
        x1 = self.l1 * np.cos(theta1)
        y1 = self.l1 * np.sin(theta1)
        
        # Posisi ujung lengan (end-effector)
        x2 = x1 + self.l2 * np.cos(theta1 + theta2)
        y2 = y1 + self.l2 * np.sin(theta1 + theta2)
        
        if plot:
            self._plot_robot(theta1, theta2, x1, y1, x2, y2, "Forward Kinematics")
            
        return x2, y2

    def inverse_kinematics(self, x, y, plot=False):
        """Menghitung sudut sendi dari target posisi (x, y)."""
        # Validasi menggunakan hukum kosinus
        cos_theta2 = (x**2 + y**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        
        if abs(cos_theta2) > 1:
            raise ValueError(f"Koordinat ({x}, {y}) di luar jangkauan robot (Unreachable)!")
            
        # IK memiliki dua kemungkinan solusi: Elbow Down (+) dan Elbow Up (-)
        theta2_down = np.arccos(cos_theta2)
        theta2_up = -np.arccos(cos_theta2)
        
        # Menghitung Theta 1 untuk masing-masing solusi
        theta1_down = np.arctan2(y, x) - np.arctan2(self.l2 * np.sin(theta2_down), self.l1 + self.l2 * np.cos(theta2_down))
        theta1_up = np.arctan2(y, x) - np.arctan2(self.l2 * np.sin(theta2_up), self.l1 + self.l2 * np.cos(theta2_up))
        
        if plot:
            # Memvisualisasikan salah satu solusi (Elbow Down)
            self._plot_robot(theta1_down, theta2_down, 
                             self.l1 * np.cos(theta1_down), self.l1 * np.sin(theta1_down), 
                             x, y, "Inverse Kinematics (Solusi: Elbow Down)")
                             
        return (theta1_down, theta2_down), (theta1_up, theta2_up)

    def _plot_robot(self, t1, t2, x1, y1, x2, y2, title):
        """Fungsi internal untuk menggambar lengan robot."""
        plt.figure(figsize=(6, 6))
        
        # Gambar garis antar sendi
        plt.plot([0, x1, x2], [0, y1, y2], 'ro-', linewidth=4, markersize=8, label='Robot Link')
        # Gambar titik sendi
        plt.plot(0, 0, 'ks', markersize=10, label='Base Origin')
        plt.plot(x2, y2, 'b*', markersize=12, label='End-Effector')
        
        # Konfigurasi plot
        limit = self.l1 + self.l2 + 1
        plt.xlim([-limit, limit])
        plt.ylim([-limit, limit])
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.axhline(0, color='black', linewidth=1)
        plt.axvline(0, color='black', linewidth=1)
        plt.title(title)
        plt.legend()
        plt.xlabel("X Axis")
        plt.ylabel("Y Axis")
        plt.show()