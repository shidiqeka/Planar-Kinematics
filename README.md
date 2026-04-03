# Planar 2-DOF Robot: Forward vs Inverse Kinematics

Repositori ini berisi implementasi simulasi matematis untuk pergerakan lengan robot planar dengan 2 Derajat Kebebasan (2-DOF). Proyek ini dibangun menggunakan bahasa Python dengan pendekatan **Object-Oriented Programming (OOP)** untuk struktur kode yang modular, serta `matplotlib` untuk visualisasi komparatif.

## 🎯 Tujuan Proyek

Proyek ini bertujuan untuk membuktikan secara matematis dan visual perbedaan mendasar antara **Forward Kinematics (FK)** dan **Inverse Kinematics (IK)** pada robot manipulator. 

Program ini menampilkan perbandingan visual secara langsung (*side-by-side*) yang mendemonstrasikan bahwa:
1. **Satu kombinasi sudut** hanya akan menghasilkan **satu posisi spesifik** (Karakteristik unik FK).
2. **Satu titik posisi target** dapat dicapai dengan **lebih dari satu kombinasi sudut/pose** (Karakteristik multi-solusi IK).

---

## 📌 Teori Dasar & Perbedaan Konsep

### 1. Forward Kinematics (FK)
* **Deskripsi:** Proses mencari letak/koordinat ujung lengan robot (*end-effector*) berdasarkan sudut putar masing-masing sendi yang diketahui.
* **Input:** Sudut Sendi 1 (θ1) dan Sudut Sendi 2 (θ2).
* **Output:** Koordinat Posisi (X, Y). Tepat hanya ada 1 solusi absolut.
* **Persamaan Matematis:**
  * X = L1 * cos(θ1) + L2 * cos(θ1 + θ2)
  * Y = L1 * sin(θ1) + L2 * sin(θ1 + θ2)

### 2. Inverse Kinematics (IK)
* **Deskripsi:** Proses membalik logika FK, yaitu mencari berapa derajat masing-masing sendi harus ditekuk agar ujung lengan robot bisa mencapai titik target yang diinginkan.
* **Input:** Titik Target Koordinat (X, Y).
* **Output:** Sudut Sendi 1 (θ1) dan Sudut Sendi 2 (θ2).
* **Karakteristik Solusi:** Pada robot 2-DOF, IK memiliki dua kemungkinan konfigurasi pose untuk mencapai titik yang sama, yaitu **Elbow Down** (siku ke bawah) dan **Elbow Up** (siku ke atas).
* **Persamaan Matematis:** Diselesaikan menggunakan kombinasi Hukum Kosinus dan fungsi trigonometri arc-tangent.

---
## 📊 Hasil Visualisasi

Berikut adalah hasil perbandingan antara Forward dan Inverse Kinematics yang dihasilkan oleh program:

![Perbandingan FK vs IK](assets/hasil.png)

*Gambar di atas menunjukkan perbandingan sisi-ke-sisi antara perhitungan posisi (FK) dan perhitungan sudut sendi (IK).*
## 🚀 Cara Menjalankan Program

Pastikan Anda sudah menginstal Python di sistem Anda.

1. **Kloning repositori ini:**
   ```bash
   git clone [https://github.com/shidiqeka/Planar-Kinematics.git](https://github.com/shidiqeka/Planar-Kinematics.git)
   cd Planar-Kinematics