# Waste Management System


Manajemen sampah yang efektif menjadi tantangan besar di daerah perkotaan akibat pembuangan yang tidak tepat dan kurangnya sistem pemantauan yang efisien. Tempat sampah yang meluap dan tidak adanya data real-time dapat menyebabkan masalah lingkungan dan kesehatan. Proyek ini mengusulkan Waste Management System berbasis IoT untuk mengatasi tantangan ini dengan mengintegrasikan pemantauan real-time terhadap tingkat kepenuhan, berat, dan lokasi sampah. Sistem ini bertujuan menyediakan manajemen tempat sampah yang otomatis dan pemantauan lingkungan yang lebih efisien serta berkelanjutan.

## Hardware design and implementation details

Perangkat keras proyek ini terdiri dari komponen-komponen berikut:

- **ESP32**: Berfungsi sebagai mikrokontroler utama untuk pemrosesan dan komunikasi.
- **Sensor Ultrasonik HC-SR04**: Mengukur tingkat kepenuhan tempat sampah.
- **Load Cell**: Menghitung berat sampah di dalam tempat sampah.
- **Modul GPS Neo-6M**: Melacak lokasi tempat sampah.
- **Motor Servo**: Mengotomasi pembukaan dan penutupan tutup tempat sampah.
- **Relay**: Mengontrol motor servo.
- **Baterai 3,7V**: Menyediakan daya untuk sistem.

Komponen-komponen ini dihubungkan untuk membentuk rangkaian di mana ESP32 mengumpulkan data dari sensor dan mengontrol servo berdasarkan kondisi yang telah ditentukan.

Berikut skema infrastrukturnya :  
![8BoSL.png](https://s6.imgcdn.dev/8BoSL.png)

## Software implementation details

Implementasi software pada proyek IoT tong sampah pintar dilakukan dengan mengintegrasikan hardware dan software melalui platform IoT, Firebase, dan Blynk. Sistem menggunakan ESP32 sebagai mikrokontroler utama untuk mengumpulkan data dari berbagai sensor, seperti sensor ultrasonik untuk mendeteksi kepenuhan, load cell untuk mengukur berat sampah, dan modul GPS untuk melacak lokasi.

Data dari sensor dikirimkan ke Firebase Realtime Database menggunakan koneksi WiFi yang dikonfigurasi pada ESP32. Firebase berfungsi sebagai media penyimpanan dan pertukaran data, sehingga data dapat diakses oleh interface web yang dirancang menggunakan HTML dan CSS. interface ini memungkinkan pengguna untuk memonitor status tong sampah, termasuk tingkat kepenuhan, berat, dan lokasi GPS secara real-time.


Berikut flowchart kami :
![8BLeo.jpg](https://s6.imgcdn.dev/8BLeo.jpg)

Flowchart proyek IoT tong sampah pintar menggambarkan alur kerja mulai dari pengambilan data oleh sensor ultrasonik, load cell, dan GPS, yang diproses oleh ESP32 untuk menentukan tingkat kepenuhan, berat, dan lokasi tong sampah. Data ini dikirim ke Firebase Realtime Database melalui WiFi untuk disimpan dan ditampilkan di aplikasi Blynk serta interfaceweb sederhana. Sistem juga dilengkapi mekanisme penutupan otomatis menggunakan servo motor jika tong penuh. Alur ini memastikan data tong sampah dapat dimonitor secara real-time oleh pengguna untuk pengelolaan yang efisien.


## Test results and performance evaluation

### Pengujian

- **Sensor Ultrasonik**: Diuji untuk pengukuran tingkat kepenuhan tempat sampah dengan akurasi pada berbagai jarak.
- **Servo Motor**: Diuji untuk memastikan keandalan dalam membuka dan menutup tutup tempat sampah.
- **Modul GPS**: Diuji untuk pelacakan lokasi yang presisi dan pembaruan data secara real-time.
- **Integrasi Firebase**: Diuji untuk memastikan data dapat diunggah dan diambil secara real-time tanpa gangguan.
- **Integrasi Blynk**: Diuji untuk memastikan kendali dan pemantauan yang responsif melalui aplikasi mobile.
- **Koneksi WiFi ESP32**: Diuji untuk kestabilan koneksi ke jaringan WiFi.

### Hasil

- **Sensor Ultrasonik**: Berhasil mengukur tingkat kepenuhan tempat sampah dengan akurasi tinggi.
- **Servo Motor**: Berfungsi dengan andal untuk membuka dan menutup tutup tempat sampah berdasarkan kondisi kepenuhan.
- **Modul GPS**: Memberikan pembaruan lokasi tempat sampah secara tepat dan cepat.
- **Integrasi Firebase**: Memastikan sinkronisasi data real-time antara sistem dan basis data berjalan lancar.
- **Integrasi Blynk**: Memungkinkan pemantauan dan pengendalian jarak jauh yang mulus melalui antarmuka pengguna.
- **Koneksi WiFi ESP32**: Berhasil terhubung dan mempertahankan koneksi yang stabil ke jaringan WiFi.

## Conclusion

Proyek ini berhasil mengembangkan Waste Management System berbasis IoT yang mampu melakukan pemantauan real-time dan otomatisasi. Sistem ini mengatasi tantangan utama dalam manajemen sampah, menyediakan solusi yang dapat diskalakan dan berkelanjutan.

### Future Work : 
- Mengintegrasikan solar panel untuk meningkatkan efisiensi daya.
- Menerapkan AI untuk penjadwalan pengumpulan sampah secara prediktif.
