# PulseLab — DC Motor Control Dashboard

Dashboard web modern bernuansa pink-biru untuk Arduino Nano + encoder + driver motor, dibuat untuk cocok dengan protokol serial sketch yang disertakan pengguna.

## Fitur
- Kontrol mode `0 STOP`, `1 OPEN LOOP`, `2 CLOSED LOOP RPM`, `3 CLOSED LOOP POSISI`.
- Koneksi USB serial via Web Serial API pada Chrome/Edge desktop.
- Grafik telemetri target vs aktual (RPM/sudut) serta PWM.
- PID tuning untuk RPM dan posisi.
- Estimasi overshoot, settling time, steady-state error, dan rise time dari data telemetri yang masuk.
- Status mode, PWM output, RPM terukur, dan sudut real-time.

## Protokol serial yang digunakan
Baud rate **115200**, perintah diakhiri newline:
- `MODE:0` … `MODE:3`
- `SET_PWM:100`
- `SET_RPM:150`
- `SET_ANG:90`
- `RESET_ORIGIN`
- `PID_RPM:1.2,0.5,0.01`
- `PID_POS:5,0.2,0.1`

Dashboard membaca telemetri sketch dengan format:
`DATA:Mode,CurrentRPM,TargetRPM,CurrentAngle,TargetAngle,CurrentPWM`

## Menjalankan lokal
1. Buka `index.html` dengan server lokal (lebih baik daripada membuka file langsung).
2. Opsi mudah: gunakan VS Code + ekstensi Live Server, lalu klik **Go Live**.
3. Buka URL localhost di Chrome/Edge, tekan **Hubungkan USB**, pilih port Arduino Nano.
4. Tutup Arduino Serial Monitor/IDE Serial Plotter sebelum koneksi karena satu port serial hanya dapat dipakai satu aplikasi pada saat yang sama.

## Publikasi GitHub Pages (`https://...`)
1. Buat repository baru di GitHub, misalnya `pulselab-motor-dc`.
2. Unggah `index.html`, `style.css`, `app.js`, dan `README.md` (file `.ino` boleh ikut diunggah sebagai dokumentasi).
3. Buka **Settings → Pages**.
4. Pada **Build and deployment**, pilih **Deploy from a branch**.
5. Branch `main`, folder `/(root)`, lalu **Save**.
6. Tunggu deployment selesai. Alamat biasanya `https://USERNAME.github.io/NAMA-REPOSITORY/`.
   - Ganti `USERNAME` dan `NAMA-REPOSITORY` dengan milikmu; URL final belum dapat ditentukan sebelum nama akun/repository diketahui.

## Catatan penting
- Web Serial memerlukan Chrome/Edge yang mendukungnya dan konteks aman HTTPS atau `localhost`. GitHub Pages menyediakan HTTPS.
- Halaman web tidak bisa tersambung ke USB dari HP Android seperti dashboard desktop; gunakan laptop/PC dengan Chrome/Edge.
- Browser tidak dapat otomatis mengambil port USB tanpa tindakan pengguna. Klik tombol koneksi dan pilih port.
- Tombol STOP mengirim `MODE:0` dan `SET_PWM:0`; tetap sediakan pemutus daya fisik.
- Angka analisis transien adalah estimasi berdasarkan sampel yang tersedia, bukan hasil uji laboratorium otomatis.
- Pastikan protokol sketch cocok dengan firmware Arduino yang sedang terpasang.
