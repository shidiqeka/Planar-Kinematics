import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class PlanarRobot2DOF:
    def compare_fk_ik(self, t1, t2):
        """Membandingkan FK dan IK secara bersamaan dalam satu figur (Side-by-Side)."""
        # 1. Jalankan FK untuk mencari posisi dari sudut yang diberikan
        x_target, y_target = self.forward_kinematics(t1, t2, plot=False)
        
        # 2. Jalankan IK menggunakan posisi hasil FK tadi
        (t1_down, t2_down), (t1_up, t2_up) = self.inverse_kinematics(x_target, y_target, plot=False)
        
        # 3. Buat Kanvas Plot Side-by-Side
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        limit = self.l1 + self.l2 + 1
        
        def setup_axis(ax, title):
            ax.set_xlim([-limit, limit])
            ax.set_ylim([-limit, limit])
            ax.grid(True, linestyle='--', alpha=0.7)
            ax.axhline(0, color='black', linewidth=1)
            ax.axvline(0, color='black', linewidth=1)
            ax.set_title(title, pad=15, fontweight='bold')
            ax.set_xlabel("X Axis")
            ax.set_ylabel("Y Axis")

        # --- SUBPLOT 1: Forward Kinematics (KIRI) ---
        title_fk = f"FORWARD KINEMATICS\nInput: Sudut (θ1={np.degrees(t1):.0f}°, θ2={np.degrees(t2):.0f}°)\nOutput: Posisi (X={x_target:.2f}, Y={y_target:.2f})"
        setup_axis(ax1, title_fk)
        
        x1_fk = self.l1 * np.cos(t1)
        y1_fk = self.l1 * np.sin(t1)
        
        ax1.plot([0, x1_fk, x_target], [0, y1_fk, y_target], 'ro-', linewidth=4, markersize=8, label='Pose Robot (Satu-satunya)')
        ax1.plot(0, 0, 'ks', markersize=10, label='Base')
        ax1.plot(x_target, y_target, 'b*', markersize=12, label='End-Effector')
        ax1.legend(loc='lower right')

        # --- SUBPLOT 2: Inverse Kinematics (KANAN) ---
        title_ik = f"INVERSE KINEMATICS\nInput: Posisi (X={x_target:.2f}, Y={y_target:.2f})\nOutput: Sudut (2 Kemungkinan Solusi)"
        setup_axis(ax2, title_ik)
        
        # Gambar Solusi 1 (Elbow Down - Hijau)
        x1_down = self.l1 * np.cos(t1_down)
        y1_down = self.l1 * np.sin(t1_down)
        ax2.plot([0, x1_down, x_target], [0, y1_down, y_target], 'go-', linewidth=3, markersize=8, label='Solusi 1: Elbow Down')
        
        # Gambar Solusi 2 (Elbow Up - Ungu putus-putus)
        x1_up = self.l1 * np.cos(t1_up)
        y1_up = self.l1 * np.sin(t1_up)
        ax2.plot([0, x1_up, x_target], [0, y1_up, y_target], 'mo--', linewidth=3, markersize=8, label='Solusi 2: Elbow Up')
        
        ax2.plot(0, 0, 'ks', markersize=10)
        ax2.plot(x_target, y_target, 'b*', markersize=12)
        ax2.legend(loc='lower right')

        plt.tight_layout()
        plt.show()
    def __init__(self, l1, l2):
        """Inisialisasi panjang link/lengan robot."""
        self.l1 = l1
        self.l2 = l2

    def forward_kinematics(self, theta1, theta2, plot=False):
        """Menghitung posisi end-effector dari sudut sendi (dalam radian)."""
        x1 = self.l1 * np.cos(theta1)
        y1 = self.l1 * np.sin(theta1)
        x2 = x1 + self.l2 * np.cos(theta1 + theta2)
        y2 = y1 + self.l2 * np.sin(theta1 + theta2)
        
        if plot:
            self._plot_robot(theta1, theta2, x1, y1, x2, y2, "Forward Kinematics")
            
        return x2, y2

    def inverse_kinematics(self, x, y, plot=False):
        """Menghitung sudut sendi dari target posisi (x, y)."""
        cos_theta2 = (x**2 + y**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        
        if abs(cos_theta2) > 1:
            raise ValueError(f"Koordinat ({x}, {y}) di luar jangkauan robot (Unreachable)!")
            
        theta2_down = np.arccos(cos_theta2)
        theta2_up = -np.arccos(cos_theta2)
        
        theta1_down = np.arctan2(y, x) - np.arctan2(self.l2 * np.sin(theta2_down), self.l1 + self.l2 * np.cos(theta2_down))
        theta1_up = np.arctan2(y, x) - np.arctan2(self.l2 * np.sin(theta2_up), self.l1 + self.l2 * np.cos(theta2_up))
        
        if plot:
            self._plot_robot(theta1_down, theta2_down, 
                             self.l1 * np.cos(theta1_down), self.l1 * np.sin(theta1_down), 
                             x, y, "Inverse Kinematics (Solusi: Elbow Down)")
                             
        return (theta1_down, theta2_down), (theta1_up, theta2_up)

    def animate_movement(self, theta_start, theta_end, steps=60, interval=50):
        """
        Membuat animasi gerakan robot dari posisi awal ke target.
        theta_start: tuple (theta1, theta2) awal dalam radian
        theta_end: tuple (theta1, theta2) akhir dalam radian
        steps: jumlah frame (semakin besar semakin halus)
        interval: jeda antar frame dalam milidetik
        """
        # Interpolasi linier untuk menghasilkan pergerakan sendi yang mulus
        t1_vals = np.linspace(theta_start[0], theta_end[0], steps)
        t2_vals = np.linspace(theta_start[1], theta_end[1], steps)

        fig, ax = plt.subplots(figsize=(6, 6))
        
        # Konfigurasi batas grafik
        limit = self.l1 + self.l2 + 1
        ax.set_xlim([-limit, limit])
        ax.set_ylim([-limit, limit])
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.axhline(0, color='black', linewidth=1)
        ax.axvline(0, color='black', linewidth=1)
        ax.set_title("Animasi Pergerakan Robot")
        ax.set_xlabel("X Axis")
        ax.set_ylabel("Y Axis")

        # Inisialisasi elemen grafis yang akan dianimasikan
        line, = ax.plot([], [], 'ro-', linewidth=4, markersize=8, label='Robot Link')
        ax.plot(0, 0, 'ks', markersize=10, label='Base') # Base tidak bergerak
        end_eff, = ax.plot([], [], 'b*', markersize=12, label='End-Effector')
        ax.legend()

        def update(frame):
            """Fungsi untuk mengupdate gambar setiap frame."""
            t1 = t1_vals[frame]
            t2 = t2_vals[frame]
            
            # Hitung FK untuk posisi saat ini
            x1 = self.l1 * np.cos(t1)
            y1 = self.l1 * np.sin(t1)
            x2 = x1 + self.l2 * np.cos(t1 + t2)
            y2 = y1 + self.l2 * np.sin(t1 + t2)
            
            # Update data koordinat pada elemen grafis
            line.set_data([0, x1, x2], [0, y1, y2])
            end_eff.set_data([x2], [y2])
            return line, end_eff

        # Menjalankan animasi
        ani = animation.FuncAnimation(fig, update, frames=steps, interval=interval, blit=True)
        plt.show()

    def _plot_robot(self, t1, t2, x1, y1, x2, y2, title):
        """Fungsi internal untuk menggambar lengan robot (Statis)."""
        plt.figure(figsize=(6, 6))
        plt.plot([0, x1, x2], [0, y1, y2], 'ro-', linewidth=4, markersize=8, label='Robot Link')
        plt.plot(0, 0, 'ks', markersize=10, label='Base Origin')
        plt.plot(x2, y2, 'b*', markersize=12, label='End-Effector')
        
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