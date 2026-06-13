# Nur-o-link v3.5

![Nur-o-link v3.5 Projesi](v3.5/images/nurolink-v3_5.png)

Nur-o-link v3.5, ESP32 mikrodenetleyici platformu üzerine inşa edilmiş, 7 farklı vites kademesiyle sürüş kabiliyeti ve 4 eklemli (4-DOF) hassas robotik kol kontrolü sunan gelişmiş bir sistemdir. Sistem, Wi-Fi üzerinden RemoteXY mobil uygulaması ile haberleşir.

## Teknik Gereksinimler ve Donanım
* **Mikrodenetleyici:** ESP32 Dev Module (WROOM-32).
* **Yazılım Ortamı:** Arduino IDE (ESP32 by Espressif Systems v3.3.8).
* **Kütüphaneler:** * RemoteXY v4.1.9
    * ESP32Servo v3.2.0
    * VL53L1X [by Pololu] v1.3.1
* **Donanım Bileşenleri:** * Motor Sürücü: TB6612FNG (4x DC Motor).
    * Robotik Kol: 4x Servo Motor (8-Yönlü Kontrol).
    * Sensör: VL53L1X Lazer Mesafe Sensörü.
    * Geri Bildirim: Pasif Buzzer (Frekans Modülasyonu).

## Çalışma Modları

[Detaylı Kullanım Kılavuzu](v3.5/kumanda/nur-o-link_v3_5_kumanda.pdf)

### 1. Sürüş Modu (DRIVE_MODE)

![Nur-o-link v3.5 Kumanda-1 (DRIVE_MODE)](v3.5/images/nurolink-v3_5-kumanda-1.jpeg)

Sistem başlangıçta bu mod ile açılır.
* **Hareket Kontrolü:** D-Pad tuşları ile yönlendirme sağlanır. İleri sürüş esnasında dinamik engel koruma sistemi (VL53L1X) 500mm mesafe sınırında otomatik frenleme yapar.
* **Vites ve Hız:** Üçgen butonu vites artırırken, Çarpı butonu vites düşürür (7 kademeli vites, PWM 60-255).
* **Özel Komutlar:** Kare butonu manuel korna işlevi görür (2000Hz), Daire butonu tüm servo motorları dik açıya getirir.
* **Mod Geçişi ve Güvenlik:** "Kontrol Modu" butonu ARM_MODE moduna geçişi sağlar. "Tüm Motorları" anahtarı tüm sistemin enerjisini keser.

### 2. Kol Kontrol Modu (ARM_MODE)

![Nur-o-link v3.5 Kumanda-2 (ARM_MODE)](v3.5/images/nurolink-v3_5-kumanda-2.jpeg)

Bu modda manuel servo kontrolü sağlanır.
* **1. Servo (Taban):** D-Pad Sol/Sağ tuşları ile 25° - 155° aralığında hareket ettirilir.
* **2. Servo (Omuz):** D-Pad Yukarı/Aşağı tuşları ile 90° - 180° aralığında kontrol edilir.
* **3. Servo (Dirsek):** Üçgen ve Çarpı tuşları ile 40° - 90° aralığında 2 derecelik hassas adımlarla hareket eder.
* **4. Servo (Kıskaç):** Kare ve Daire tuşları ile 65° (açık) - 130° (kapalı) aralığında kontrol edilir.
* **Servo Reset:** Özel "L" butonuna basılarak tüm servo motorlar dik açıya getirilir.

## Yazılım Mimarisi
Sistem, yüksek performans ve kararlılık için "Non-blocking (Bloklamayan) State Machine" mimarisi ile kodlanmıştır. ESP32'nin çoklu görev yetenekleri ve optimize edilmiş döngü yapısı ile gecikmesiz bir kontrol deneyimi hedeflenmiştir.

---
**Geliştirici:** Abdulkadir GÜNGÖR (a.kadir.gungor.86@gmail.com)