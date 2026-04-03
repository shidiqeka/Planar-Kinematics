# Planar 2-DOF Robot Kinematics (OOP Approach)

Repositori ini berisi implementasi matematis dan simulasi visual untuk **Forward Kinematics (FK)** dan **Inverse Kinematics (IK)** pada lengan robot planar dengan 2 derajat kebebasan (2-DOF). 

Berbeda dengan implementasi prosedural biasa, proyek ini dibangun menggunakan paradigma **Object-Oriented Programming (OOP)** di Python menggunakan pustaka `numpy` untuk komputasi matriks/trigonometri dan `matplotlib` untuk visualisasi grafik.

## 📌 Teori Dasar

### 1. Forward Kinematics (FK)
Mencari posisi akhir (end-effector) $(x, y)$ berdasarkan masukan sudut sendi $\theta_1$ dan $\theta_2$.
$$x = L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2)$$
$$y = L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2)$$

### 2. Inverse Kinematics (IK)
Mencari sudut sendi $\theta_1$ dan $\theta_2$ yang dibutuhkan untuk mencapai target koordinat $(x, y)$. 
Solusi ini menghitung hukum kosinus untuk mendapatkan $\theta_2$:
$$\cos(\theta_2) = \frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}$$
$$\theta_2 = \pm \arccos(\cos(\theta_2))$$
*Catatan: Nilai $\pm$ merepresentasikan dua kemungkinan konfigurasi lengan: Elbow-Up dan Elbow-Down.*

Selanjutnya, $\theta_1$ didapatkan dengan:
$$\theta_1 = \text{atan2}(y, x) - \text{atan2}(L_2 \sin(\theta_2), L_1 + L_2 \cos(\theta_2))$$

## 🚀 Cara Menjalankan Program

1. Install pustaka yang diperlukan:
   ```bash
   pip install -r requirements.txt