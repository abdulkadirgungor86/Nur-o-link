# Nur-o-link v2.7

![Nur-o-link v2.7 Projesi](v2.7/images/nurolink-v2_7.png)

Nur-o-link v2.7, ESP32 mikrodenetleyici platformu üzerine inşa edilmiş, 7 farklı vites kademesiyle sürüş kabiliyeti ve 4 eklemli (4-DOF) robotik kol kontrolü sunan gelişmiş bir sistemdir. Sistem, Bluetooth üzerinden Dabble mobil uygulaması ile haberleşir.

## Teknik Gereksinimler ve Donanım
* Mikrodenetleyici: ESP32 Dev Module (WROOM-32) kullanılmaktadır.
* Yazılım Ortamı: Arduino IDE üzerinde ESP32 Core v2.0.11 sürümü ile uyumludur; daha güncel sürümlerde hata alınabilir.
* Kütüphaneler: DabbleESP32 v1.5.1 ve ESP32Servo v3.1.3 sürümlerinin kullanılması önerilir.
* Sürücü ve Sensörler: Motor kontrolü için TB6612FNG entegresi, mesafe ölçümü için HC-SR04 ultrasonik sensörü ve geri bildirim için pasif buzzer sistemi sisteme entegredir.

## Çalışma Modları

![Nur-o-link v2.7 Kumanda](v2.7/images/nurolink-v2_7-kumanda.jpg)

[Detaylı Kullanım Kılavuzu](v2.7/kumanda/nur-o-link_v2_7_kumanda.pdf)

### 1. Sürüş Modu (DRIVE_MODE)
Sistem başlangıçta bu mod ile açılır.
* Hareket Kontrolü: D-Pad tuşları ile yönlendirme sağlanır. İleri sürüş esnasında dinamik engel koruma sistemi aktiftir.
* Vites ve Hız: Üçgen butonu vites artırırken, Çarpı butonu vites düşürür. Vites seviyesi değiştikçe durma mesafesi otomatik olarak güncellenir.
* Özel Komutlar: Kare butonu korna işlevi görür, Daire butonu ise robot kolunu otomatik olarak başlangıç konumuna (90 derece) getirir.
* Mod Geçişi ve Güvenlik: SELECT butonuna kısa basılarak ARM_MODE moduna geçilir. SELECT butonuna 1.2 saniye basılı tutmak engel koruma sistemini açar veya kapatır. START butonuna 1.2 saniye basmak sistemi kilitler ve servoların enerjisini keser.

### 2. Kol Kontrol Modu (ARM_MODE)
Bu modda manuel servo kontrolü sağlanır.
* 1. Servo (Taban): D-Pad Sol/Sağ tuşları ile 25 derece ile 155 derece arasında hareket ettirilir.
* 2. Servo (Omuz): D-Pad Yukarı/Aşağı tuşları ile 85 derece ile 160 derece arasında kontrol edilir.
* 3. Servo (Dirsek): Üçgen ve Çarpı tuşları ile 40 derece ile 80 derece arasında hassas hareket ettirilir.
* 4. Servo (Pençe): Kare ve Daire tuşları ile 70 derece ile 120 derece arasında açılıp kapatılır.
* Toplama ve Çıkış: SELECT butonuna 1.2 saniye basıldığında kol asenkron olarak güvenli (90 derece) konumuna döner. START butonuna kısa basıldığında DRIVE_MODE moduna geri dönülür.

---
**Geliştirici:** Abdulkadir GÜNGÖR (a.kadir.gungor.86@gmail.com)
