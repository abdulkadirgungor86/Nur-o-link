/*******************************************************************************
 * PROJE ADI          : Nur-o-link v3.5
 * SİSTEM TANIMI      : (Gelişmiş) 7 Vitesli ve 4 Eklemli (8 Yönlü) Kontrol Sistemi
 * YAZILIM MİMARİSİ   : Non-blocking (Bloklamayan) State Machine (Durum Makinesi)
 * GELİŞTİRİCİ        : Abdulkadir GÜNGÖR (a.kadir.gungor.86@gmail.com)
 * TARİH              : 2026-04-20
 * * TEKNİK GEREKSİNİM  : Arduino IDE - ESP32 (by Espressif Systems) 3.3.8
 *                        ESP32 Kart Sürümü v3.3.8
 ** BOARD VE KÜTÜPHANE SÜRÜMLERİ:
 *                        1) En Son "ESP32 v3.3.8" Sorunsuz Çalıştı. 
 *                        2) En Son "RemoteXY v4.1.9" Sorunsuz Çalıştı. 
 *                        -> En Son "RemoteXY mobile app - ANDROID 4.18.03" Sorunsuz Çalıştı.
 *                        3) En Son "ESP32Servo v3.2.0" Sorunsuz Çalıştı. 
 *                        4) En Son "VL53L1X [by Pololu] v1.3.1" Sorunsuz Çalıştı.                  
 * * DONANIM BİLEŞENLERİ (REX-8in1-V2 Donanım Seti) **
 * - Mikrodenetleyici : ESP32 Dev Module (WROOM-32)
 * - Motor Sürücü     : TB6612FNG Entegre Sürücü (4x DC Motor Destekli)
 * - Robotik Kol      : 4x Servo Motor (4-DOF / 8-Yönlü Kontrol)
 * - Sensörler        : VL53L1X Laser Ranging Sensor Module
 * - Geri Bildirim    : Pasif Buzzer (Frekans Modülasyonu Destekli)
 * * KONTROL ARA YÜZÜ   : RemoteXY (Wifi üzerinden Özel Tasarım Arayüz) [Wifi bağlantısının kopması daha zor]
 *******************************************************************************/

#include <Arduino.h>                // ESP32 temel kütüphanesi
#include <ESP32Servo.h>             // Servo motorların ESP32 ile PWM kontrolü için kütüphane
#include <Wire.h>                  // I2C haberleşme protokolü kütüphanesi (Sensör için)
#include <VL53L1X.h>               // Lazer mesafe sensörü kütüphanesi

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
#define REMOTEXY__DEBUGLOG    // Seri port üzerinden RemoteXY hata ayıklama günlüklerini aktif eder

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__WIFI_POINT // RemoteXY bağlantı modunu Wi-Fi Erişim Noktası (Access Point) olarak ayarlar

#include <WiFi.h> // ESP32 Wi-Fi fonksiyonlarını kullanabilmek için gerekli kütüphane

// RemoteXY connection settings 
#define REMOTEXY_WIFI_SSID "Nur-o-link v3.5" // Oluşturulacak Wi-Fi ağının adını tanımlar
#define REMOTEXY_WIFI_PASSWORD "00000000" // Wi-Fi ağının şifresini tanımlar
#define REMOTEXY_SERVER_PORT 6377 // RemoteXY haberleşmesi için sunucu portunu belirler
#define REMOTEXY_ACCESS_PASSWORD "0000" // Arayüze erişim için güvenlik şifresini tanımlar


#include <RemoteXY.h> // RemoteXY ana kütüphanesini projeye dahil eder

// RemoteXY GUI configuration  
#pragma pack(push, 1)  // Bellek hizalamasını 1 bayt olarak ayarlar (yapı verileri sıkıştırılır)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   // Arayüz grafik tasarım verilerini flash bellekte tutan dizi
  { 255,22,0,37,0,243,2,19,0,0,0,78,117,114,45,111,45,108,105,110,
  107,0,31,1,200,84,2,3,2,28,0,1,38,47,14,31,2,14,31,32,
  226,134,147,32,0,1,7,37,33,12,2,14,31,226,134,144,0,1,48,35,
  33,13,2,14,31,226,134,146,0,1,36,6,14,31,2,14,31,32,226,134,
  145,32,0,71,92,23,30,30,56,8,36,24,135,0,0,128,63,0,0,224,
  64,0,0,128,63,0,0,128,63,0,0,128,63,24,0,67,102,44,10,8,
  121,24,24,129,81,3,26,4,64,24,75,111,110,116,114,111,108,32,77,111,
  100,117,58,0,129,79,11,27,4,64,24,69,110,103,101,108,32,83,101,110,
  115,111,114,117,58,0,129,80,15,26,4,64,24,69,110,103,101,108,32,77,
  101,115,97,102,101,58,0,129,71,7,35,4,64,24,77,111,116,111,114,108,
  97,114,196,177,110,32,68,117,114,117,109,117,58,0,129,118,15,6,3,64,
  13,109,109,0,67,105,15,13,4,81,13,67,107,11,15,4,64,13,11,67,
  107,7,15,4,64,13,11,130,72,57,70,26,11,25,131,78,65,16,16,0,
  13,13,31,75,111,108,0,6,129,76,61,19,3,64,31,75,111,110,116,114,
  111,108,32,77,111,100,117,0,10,100,65,16,16,48,1,13,31,75,97,112,
  97,116,0,31,32,65,195,167,32,0,129,98,61,20,3,64,31,69,110,103,
  101,108,32,83,101,110,115,111,114,117,0,10,122,65,16,16,48,1,13,31,
  75,97,112,97,116,0,31,32,65,195,167,32,0,129,121,61,19,3,64,31,
  84,195,188,109,32,77,111,116,111,114,108,97,114,196,177,0,129,107,3,11,
  4,64,13,83,195,188,114,195,188,197,159,0,130,96,59,1,22,27,31,130,
  119,59,1,22,27,31,1,136,29,23,23,0,14,31,226,151,187,0,1,154,
  8,23,23,0,14,31,226,150,179,0,1,172,28,23,23,0,14,31,226,151,
  175,0,1,155,49,23,23,0,14,31,226,156,151,0,21,0,129,81,3,26,
  4,64,24,75,111,110,116,114,111,108,32,77,111,100,117,58,0,129,71,7,
  35,4,64,24,77,111,116,111,114,108,97,114,196,177,110,32,68,117,114,117,
  109,117,58,0,130,72,57,70,26,11,25,131,78,65,16,16,0,13,13,31,
  83,195,188,114,195,188,197,159,0,9,129,76,61,19,3,64,31,75,111,110,
  116,114,111,108,32,77,111,100,117,0,129,98,61,19,3,64,31,84,195,188,
  109,32,77,111,116,111,114,108,97,114,196,177,0,129,121,61,20,3,64,31,
  83,101,114,118,111,32,77,111,116,111,114,108,97,114,0,129,107,3,6,4,
  64,13,75,111,108,0,130,96,59,1,22,27,31,130,119,59,1,22,27,31,
  1,122,65,16,16,0,13,31,226,136,159,0,10,100,65,16,16,48,1,13,
  31,75,97,112,97,116,0,31,32,65,195,167,32,0,1,38,47,14,31,2,
  14,31,32,226,134,147,32,0,1,7,37,33,12,2,14,31,226,134,144,0,
  1,48,35,33,13,2,14,31,226,134,146,0,1,36,6,14,31,2,14,31,
  32,226,134,145,32,0,1,136,29,23,23,0,14,31,226,151,187,0,1,154,
  8,23,23,0,14,31,226,150,179,0,1,172,28,23,23,0,14,31,226,151,
  175,0,1,155,49,23,23,0,14,31,226,156,151,0,67,107,7,15,4,64,
  13,11 };
  
// this structure defines all the variables and events of your control interface 
struct { // Mobil arayüzdeki değişkenleri ve olayları tutan ana RemoteXY yapısı

    // input variables
  uint8_t page_0; // Sayfa 0 görünürse 1, değilse 0 değerini alan değişken
  uint8_t page_1; // Sayfa 1 görünürse 1, değilse 0 değerini alan değişken
  uint8_t backBtn_p0; // Sayfa 0'daki geri butonunun basılma durumu (1: Basılı, 0: Değil)
  uint8_t leftBtn_p0; // Sayfa 0'daki sol butonunun basılma durumu (1: Basılı, 0: Değil)
  uint8_t rightBtn_p0; // Sayfa 0'daki sağ butonunun basılma durumu (1: Basılı, 0: Değil)
  uint8_t forwardBtn_p0; // Sayfa 0'daki ileri butonunun basılma durumu (1: Basılı, 0: Değil)
  uint8_t distancePSwitch_p0; // Sayfa 0'daki mesafe sensörü açma/kapama anahtarı durumu (1: Açık, 0: Kapalı)
  uint8_t motorPSwitch_p0; // Sayfa 0'daki motor gücü açma/kapama anahtarı durumu (1: Açık, 0: Kapalı)
  uint8_t squareBtn_p0; // Sayfa 0'daki kare butonunun (korna) basılma durumu (1: Basılı, 0: Değil)
  uint8_t triangleBtn_p0; // Sayfa 0'daki üçgen butonunun (vites artırma) basılma durumu (1: Basılı, 0: Değil)
  uint8_t circleBtn_p0; // Sayfa 0'daki daire butonunun (kol resetleme) basılma durumu (1: Basılı, 0: Değil)
  uint8_t xBtn_p0; // Sayfa 0'daki X butonunun (vites düşürme) basılma durumu (1: Basılı, 0: Değil)
  uint8_t verticalBtn_p1; // Sayfa 1'deki dikey konumlandırma (reset) butonunun basılma durumu (1: Basılı, 0: Değil)
  uint8_t motorPSwitch_p1; // Sayfa 1'deki motor gücü anahtarı durumu (1: Açık, 0: Kapalı)
  uint8_t backBtn_p1; // Sayfa 1'deki geri butonunun basılma durumu (1: Basılı, 0: Değil)
  uint8_t leftBtn_p1; // Sayfa 1'deki sol butonunun basılma durumu (1: Basılı, 0: Değil)
  uint8_t rightBtn_p1; // Sayfa 1'deki sağ butonunun basılma durumu (1: Basılı, 0: Değil)
  uint8_t forwardBtn_p1; // Sayfa 1'deki ileri butonunun basılma durumu (1: Basılı, 0: Değil)
  uint8_t squareBtn_p1; // Sayfa 1'deki kare butonunun basılma durumu (1: Basılı, 0: Değil)
  uint8_t triangleBtn_p1; // Sayfa 1'deki üçgen butonunun basılma durumu (1: Basılı, 0: Değil)
  uint8_t circleBtn_p1; // Sayfa 1'deki daire butonunun basılma durumu (1: Basılı, 0: Değil)
  uint8_t xBtn_p1; // Sayfa 1'deki X butonunun basılma durumu (1: Basılı, 0: Değil)

    // output variables
  int8_t gearInstrument_p0; // Sayfa 0'daki vites göstergesi bileşeni değeri (1 - 7 arası)
  int8_t gearNumber_p0; // Sayfa 0'daki vites numarasını gösteren sayısal değer (-128 ile 127 arası)
  int16_t distanceNumber_p0; // Sayfa 0'daki mesafe değerini gösteren sayısal değer (-32768 ile 32767 arası)
  char distanceStr_p0[11]; // Sayfa 0'daki mesafe durumunu gösteren metin dizisi (maks 10 karakter + null)
  char motorStr_p0[11]; // Sayfa 0'daki motor durumunu gösteren metin dizisi (maks 10 karakter + null)
  char motorStr_p1[11]; // Sayfa 1'deki motor durumunu gösteren metin dizisi (maks 10 karakter + null)

    // other variable
  uint8_t connect_flag;  // Uygulama bağlantı durum bayrağı (1: Bağlı, 0: Bağlantı yok)

} RemoteXY;   // RemoteXY nesne yapısı sonu
#pragma pack(pop) // Bellek sıkıştırma ayarını varsayılana döndürür
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

/* --- DONANIM PİNLERİ --- */
#define MOTOR_PWM_PIN   13 // Tüm DC motorların hız kontrolü için ortak PWM pini
#define MOTOR_A1_PIN    15 // A motoru yön kontrol pini 1
#define MOTOR_A2_PIN    23 // A motoru yön kontrol pini 2
#define MOTOR_B1_PIN    32 // B motoru yön kontrol pini 1
#define MOTOR_B2_PIN    33 // B motoru yön kontrol pini 2
#define MOTOR_C1_PIN     5 // C motoru yön kontrol pini 1
#define MOTOR_C2_PIN     4 // C motoru yön kontrol pini 2
#define MOTOR_D1_PIN    27 // D motoru yön kontrol pini 1
#define MOTOR_D2_PIN    14 // D motoru yön kontrol pini 2
#define SERVO_M1         2 // 1. Servo motor (Taban) kontrol pini
#define SERVO_M2        26 // 2. Servo motor (Omuz) kontrol pini
#define SERVO_M3        18 // 3. Servo motor (Dirsek) kontrol pini
#define SERVO_M4        19 // 4. Servo motor (Kıskaç) kontrol pini
#define BUZZER_PIN      25 // Sesli uyarılar için buzzer bağlantı pini

/* --- PWM KANAL ATAMASI (Sadece DC Motor) --- */
#define MOTOR_PWM_CH    4   // DC Motorların hız kontrolü için kullanılan LEDC PWM kanalı

/* --- SABİTLER --- */
const int SERVO_SOFT_DELAY_MS  = 15; // Yumuşak servo hareketi için adımlar arası bekleme süresi (milisaniye)
const int BRAKE_DISTANCE_MM    = 500; // Otomatik frenleme tetikleme mesafesi sınır değeri (milimetre)

enum MoveDirection { STOPPED, FWD, BWD, LFT, RGT }; // Hareket yönlerini tutan numaralandırma sınıfı (DUR, İLERİ, GERİ, SOL, SAĞ)

bool isResetting      = false; // Robot kolun ana konuma dönme işleminin aktiflik durumu
bool isArmPowered     = false; // Servo motorların enerjili (bağlı) olma durumu
bool isFirstBrake     = false; // Frenleme döngüsünün ilk adımda olduğunu belirten kontrol bayrağı
bool isSensorInit     = false; // Mesafe sensörünün başarıyla başlatılma durumu
bool isSensorRunning  = false; // Sensörün anlık çalışma durumunu tutar
bool isUpPressed      = false; // Vites artırma butonuna basılı kalma durum kontrolü
bool isDownPressed    = false; // Vites düşürme butonuna basılı kalma durum kontrolü
bool obstacleBuzzerActive = false; // Sensör engeli nedeniyle ötüyorsa true olur

unsigned long lastResetStepMs   = 0; // Kol sıfırlama adımları için son zaman kaydı
unsigned long brakeStartTime    = 0; // Frenleme işleminin başladığı anın milisaniye zaman kaydı
unsigned long audioAutoCutoffMs = 0; // Buzzer sesinin otomatik olarak kapatılacağı hedef milisaniye zamanı
unsigned long lastPacketMs      = 0; // RemoteXY üzerinden alınan son geçerli hareket paketinin zamanı
unsigned long servoUpdateMs     = 0; // Servo motor pozisyonlarının son güncellenme zamanı

struct RobotArm { // Robot kol eklemlerini ve nesnelerini bir arada tutan yapı
  int pos1 = 90, pos2 = 90, pos3 = 90, pos4 = 90; // 4 adet servonun anlık açısal pozisyonları (varsayılan 90 derece)
  Servo m1, m2, m3, m4; // 4 adet servo motor kontrol nesnesi
} arm; // arm adında global robot kol nesnesi oluşturulması

const int GEAR_PWM_TABLE[] = {60, 92, 125, 157, 190, 222, 255}; // 7 farklı vitese karşılık gelen PWM (hız) değerleri tablosu
int activeGearIdx  = 0; // Aktif olarak seçili olan vitesin indeks değeri (0 - 6 arası)
int activeMotorPWM = 60; // Motorlara uygulanacak aktif PWM hız değeri (başlangıçta 1. vites hızı)

MoveDirection lastActiveDir = STOPPED; // Aracın durmadan önceki son aktif hareket yönü

VL53L1X laserSensor; // Lazer mesafe sensörü sürücü nesnesi

/* =====================================================================
 *  BUZZER FONKSİYONLARI
 * ===================================================================== */
void stopBuzzerHard() { // Buzzer sesini tamamen kesen donanımsal susturma fonksiyonu
  // PWM'i tamamen devreden çıkar, pini normal dijital pine çek
  ledcDetach(BUZZER_PIN); // Buzzer pinine bağlı olan PWM sinyalini keser
  digitalWrite(BUZZER_PIN, LOW); // Pindeki voltajı sıfıra çekerek akımı durdurur
  pinMode(BUZZER_PIN, OUTPUT); // Pini tekrar standart çıkış moduna alır
} // Fonksiyon sonu

void playAlertTone(int duration) { // Belirli frekansta ve sürede ses üreten fonksiyon
  // Sadece ihtiyaç anında PWM bağla
  ledcAttach(BUZZER_PIN, 2000, 8); // Buzzer pinine 2000 Hz frekansta ve 8 bit çözünürlükte PWM bağlar
  ledcWriteTone(BUZZER_PIN, 2000); // Tanımlanan frekansta ton çalmaya başlar
  audioAutoCutoffMs = (duration > 0) ? millis() + duration : 0; // Süre verilmişse gelecekteki kapanma zamanını hesaplar
} // Fonksiyon sonu

void handleBuzzerAutoStop() { // Zaman aşımı ve güvenlik kurallarına göre buzzerı kontrol eden döngü fonksiyonu
  // 1. ZAMAN KONTROLÜ
  if (audioAutoCutoffMs > 0 && millis() >= audioAutoCutoffMs) { // Eğer süre tanımlanmışsa ve o süre dolmuşsa
    stopBuzzerHard(); // Buzzerı donanımsal olarak sustur
    audioAutoCutoffMs = 0; // Kapanma zamanlayıcısını sıfırla
  } // Zaman kontrolü sonu

  // 2. SAYFA GÜVENLİĞİ (KORNA İÇİN EKLEME)
  // Eğer Sayfa 0'da değilse;  kare butonuna bağlı veya engelden dolayı ses aktifse sustur
  if (RemoteXY.page_0 == 0) { // Kullanıcı kontrol sayfasında (Sayfa 0) değilse
    if (obstacleBuzzerActive || (RemoteXY.squareBtn_p0 == 1)) { // Engel uyarısı veya manuel korna basılı kalmışsa
        stopBuzzerHard(); // Güvenlik amacıyla buzzerı hemen sustur
        obstacleBuzzerActive = false; // Engel ses bayrağını temizle
        // Not: manualBuzzerActive static olduğu için burada sıfırlayamazsınız 
        // ama stopBuzzerHard() donanımı susturacaktır.
    } // İç kontrol sonu
  } // Sayfa kontrolü sonu

  // 3. BAĞLANTI KONTROLÜ
  if (RemoteXY.connect_flag == 0) { // Eğer mobil uygulama ile Wi-Fi bağlantısı kopmuşsa
    stopBuzzerHard(); // Güvenlik nedeniyle buzzerı derhal sustur
    obstacleBuzzerActive = false; // Engel uyarı ses durumunu sıfırla
  } // Bağlantı kontrolü sonu
} // Fonksiyon sonu

/* =====================================================================
 *  LAZER SENSÖR
 * ===================================================================== */
void vl53l1xSensorSetup() { // Lazer mesafe sensörünün başlangıç ayarlarını yapan fonksiyon
  Wire.begin(); // I2C haberleşme hattını başlatır
  // I2C hızını 400kHz yaparak veri transfer süresini düşürüyoruz
  Wire.setClock(400000); // I2C veri transfer hızını yüksek hıza (Fast Mode) ayarlar

  laserSensor.setTimeout(500); // 0 yerine makul bir timeout (Hata ayıklama için önemli) // Sensör yanıt vermediğinde kilitlenmeyi önlemek için 500ms zaman aşımı tanımlar
  isSensorInit = laserSensor.init(); // Sensörü donanımsal olarak başlatır ve sonucu değişkene aktarır

  if (isSensorInit) { // Sensör başarıyla başlatıldıysa ayarlara geç
    // 1. MOD: 1.3 metreye kadar en yüksek kararlılık
    laserSensor.setDistanceMode(VL53L1X::Short); // Kısa mesafe modunu seçer (Gürültüsüz ve kararlı ölçüm için)

    // 2. ZAMAN BÜTÇESİ: Ölçüm için ayrılan süre (Mikrosaniye cinsinden)
    // Short modda 20ms (20000) en düşük ve en hızlı sınırdır.
    laserSensor.setMeasurementTimingBudget(20000); // Her bir ölçüm periyodu için 20 milisaniye süre ayırır

    // 3. SÜREKLİ OKUMA: Ölçümler arası bekleme (Milisaniye cinsinden)
    // Kararlılık için bu değer Timing Budget'a eşit veya büyük olmalıdır.
    // 20ms bütçe için 20ms periyot = Saniyede tam 50 ölçüm (50Hz).
    laserSensor.startContinuous(20); // 20ms aralıklarla kesintisiz otomatik ölçüm döngüsünü başlatır
  } // Koşul sonu
} // Fonksiyon sonu

void vl53l1xSensorDistance() { // Sensörden mesafe verisi okuyan ve hataları yorumlayan fonksiyon
  uint16_t distance = laserSensor.read(); // Sensörün en son yaptığı ölçüm değerini milimetre cinsinden okur
  uint8_t  status   = laserSensor.ranging_data.range_status; // Ölçümün doğruluk/hata durum kodunu alır

  switch (status) { // Durum koduna göre dallanma işlemi yap
    case VL53L1X::RangeValid: // Ölçüm tamamen geçerli ve hatasız ise
      RemoteXY.distanceNumber_p0 = (distance >= 1000) ? 1000 : distance; // Mesafe 1000mm'den büyükse 1000'e sabitle, değilse ölçüleni yaz
      strcpy(RemoteXY.distanceStr_p0, "Açık"); // Ekranda durum bilgisi olarak "Açık" metnini göster
      break; // Durumdan çık
    case VL53L1X::SigmaFail: // Işık gürültüsü/sigma sınır hatası durumu
      RemoteXY.distanceNumber_p0 = -9999; // Hata kodu olarak sayısal alana -9999 yazar
      strcpy(RemoteXY.distanceStr_p0, "Hata 1!"); // Ekrana Sigma hatasını simgeleyen metni kopyalar
      break; // Durumdan çık
    case VL53L1X::SignalFail: // Yansıyan sinyal gücü yetersiz hatası durumu
      RemoteXY.distanceNumber_p0 = -9999; // Hata kodu olarak sayısal alana -9999 yazar
      strcpy(RemoteXY.distanceStr_p0, "Hata 2!"); // Ekrana Sinyal hatasını simgeleyen metni kopyalar
      break; // Durumdan çık
    case VL53L1X::HardwareFail: // Donanımsal iç arıza hatası durumu
      RemoteXY.distanceNumber_p0 = -9999; // Hata kodu olarak sayısal alana -9999 yazar
      strcpy(RemoteXY.distanceStr_p0, "Hata 5!"); // Ekrana Donanım hatasını simgeleyen metni kopyalar
      break; // Durumdan çık
    case VL53L1X::WrapTargetFail: // Menzil dışı / sinyalin yanlış hedeften dönme hatası durumu
      RemoteXY.distanceNumber_p0 = -9999; // Hata kodu olarak sayısal alana -9999 yazar
      strcpy(RemoteXY.distanceStr_p0, "Hata 7!"); // Ekrana bu özel hatayı simgeleyen metni kopyalar
      break; // Durumdan çık
    default: // Yukarıdaki standart hata kodlarının dışındaki bilinmeyen durumlar için
      if      (distance <= 100)  { strcpy(RemoteXY.distanceStr_p0, "Açık ?");  RemoteXY.distanceNumber_p0 = 0; } // Çok yakınsa mesafe 0 yapılır
      else if (distance >= 1000) { strcpy(RemoteXY.distanceStr_p0, "Açık ?"); RemoteXY.distanceNumber_p0 = 1000; } // Çok uzaksa mesafe 1000 yapılır
      else { strcpy(RemoteXY.distanceStr_p0, "Hata ?!"); RemoteXY.distanceNumber_p0 = distance; }                    // Diğer durumlarda ham mesafe korunur
      break; // Durumdan çık
  } // Switch sonu
} // Fonksiyon sonu

/* =====================================================================
 *  DC MOTOR
 * ===================================================================== */
void setMotorPins(int p1,int p2,int p3,int p4,int p5,int p6,int p7,int p8) { // Sürücü entegre pinlerine lojik seviyeleri toplu yazan fonksiyon
  digitalWrite(MOTOR_A1_PIN, p1); digitalWrite(MOTOR_A2_PIN, p2); // A Motorunun yön kontrol pinlerini ayarlar
  digitalWrite(MOTOR_B1_PIN, p3); digitalWrite(MOTOR_B2_PIN, p4); // B Motorunun yön kontrol pinlerini ayarlar
  digitalWrite(MOTOR_C1_PIN, p5); digitalWrite(MOTOR_C2_PIN, p6); // C Motorunun yön kontrol pinlerini ayarlar
  digitalWrite(MOTOR_D1_PIN, p7); digitalWrite(MOTOR_D2_PIN, p8); // D Motorunun yön kontrol pinlerini ayarlar
} // Fonksiyon sonu

void applyDrivePower(MoveDirection dir) { // Aracı yön komutlarına göre hareket ettiren veya durduran ana sürüş fonksiyonu
  if (dir == STOPPED) { // Eğer gelen komut DUR yönü ise
    if (isFirstBrake) { // Durma komutu yeni geldiyse ve ilk kez frenleme yapılacaksa
      brakeStartTime = millis(); // Zamanlayıcıyı her durumda başlat // Fren süresini ölçmek için kronometreyi başlatır
      isFirstBrake = false;      // Sadece bir kez girilmesini sağla // İlk fren işlemlerinin bittiğini işaretler

      if (lastActiveDir == FWD || lastActiveDir == BWD) { // Eğer araç durmadan hemen önce İleri veya Geri yönde gidiyorsa
        // Durum 1: İleri/Geri ivmesi var -> Orantılı Fren Yap
        ledcWrite(MOTOR_PWM_PIN, activeMotorPWM); // Mevcut hız seviyesini koruyarak PWM sinyali gönderir
        setMotorPins(1, 1, 1, 1, 1, 1, 1, 1);  // Tüm motor uçlarını HIGH (1) yaparak motorları elektriksel kilide (sert frene) alır
      } else { // Araç sağa/sola dönüyorsa veya zaten durma eğilimindeyse
        // Durum 2: Sağ/Sol veya zaten duruyordu -> Hemen Enerjiyi Kes
        ledcWrite(MOTOR_PWM_PIN, 0); // PWM sinyalini keserek gücü sıfırlar
        setMotorPins(0, 0, 0, 0, 0, 0, 0, 0); // Tüm motor uçlarını boşta bırakarak enerjisiz duruş sağlar
      } // İç yön kontrolü sonu
    } // İlk fren kontrolü sonu

    // Her iki durum için de 100 ms koruma süresi
    // Bu süre zarfında isFirstBrake false olduğu için yukarıya tekrar girmez.
    if (millis() - brakeStartTime > 100) { // Frenleme anından itibaren 100 milisaniye süre geçtiyse
      ledcWrite(MOTOR_PWM_PIN, 0); // Motorların PWM hız enerjisini tamamen keser
      setMotorPins(0, 0, 0, 0, 0, 0, 0, 0); // Motor pinlerini tamamen sıfırlayarak motorları serbest bırakır
    } // Süre kontrolü sonu
  } // Durma durumu sonu
  else { // Eğer aktif bir hareket komutu varsa (FWD, BWD, LFT, RGT)
    // Durum 3: Hareket Komutu (Buton Basılı)
    ledcWrite(MOTOR_PWM_PIN, activeMotorPWM); // Seçili vitese ait PWM hız değerini donanıma yazar
    switch (dir) { // İstenen hareket yönüne göre motor pin kombinasyonlarını ayarla
      case FWD: setMotorPins(1,0,1,0,1,0,1,0); break; // Tüm motorları ileri yönde döndürür
      case BWD: setMotorPins(0,1,0,1,0,1,0,1); break; // Tüm motorları geri yönde döndürür
      case LFT: setMotorPins(0,1,0,1,1,0,1,0); break; // Sol motorları geri, sağ motorları ileri döndürerek kendi ekseninde sola döndürür
      case RGT: setMotorPins(1,0,1,0,0,1,0,1); break; // Sol motorları ileri, sağ motorları geri döndürerek kendi ekseninde sağa döndürür
    } // Switch sonu
    
    lastActiveDir = dir; // Son aktif yön bilgisini mevcut yön ile günceller
    isFirstBrake = true; // Buton bırakıldığında fren mekanizmasını hazırla // Bir sonraki durma komutu için fren bayrağını hazır hale getirir
  } // Hareket durumu sonu
} // Fonksiyon sonu
/* =====================================================================
 *  SERVO YÖNETİMİ
 * ===================================================================== */
void handleServoAttach() { // Servo motorları piniyle ilişkilendirip aktif hale getiren fonksiyon
  if (!isArmPowered) { // Eğer servolar halihazırda enerjili ve bağlı değilse işlemleri yap
    // 1. ÖNCE FREKANSI SABİTLE (50Hz standarttır)
    arm.m1.setPeriodHertz(50); // 1. Servo için PWM frekansını standart 50Hz (20ms periyot) olarak ayarlar
    arm.m2.setPeriodHertz(50); // 2. Servo için PWM frekansını standart 50Hz olarak ayarlar
    arm.m3.setPeriodHertz(50); // 3. Servo için PWM frekansını standart 50Hz olarak ayarlar
    arm.m4.setPeriodHertz(50); // 4. Servo için PWM frekansını standart 50Hz olarak ayarlar

    // 2. SONRA PİNLERE BAĞLA
    arm.m1.attach(SERVO_M1, 700, 2300); // 1. Servoyu ilgili pine bağlar, min-max sinyal genişliğini mikrosaniye olarak sınırlar
    arm.m2.attach(SERVO_M2, 700, 2300); // 2. Servoyu ilgili pine bağlar, sinyal sınırlarını tanımlar
    arm.m3.attach(SERVO_M3, 700, 2300); // 3. Servoyu ilgili pine bağlar, sinyal sınırlarını tanımlar
    arm.m4.attach(SERVO_M4, 700, 2300); // 4. Servoyu ilgili pine bağlar, sinyal sınırlarını tanımlar

    // 3. MEVCUT POZİSYONLARA GÖNDER
    arm.m1.write(arm.pos1); // 1. Servoyu hafızadaki en son kalınan açı değerine döndürür
    arm.m2.write(arm.pos2); // 2. Servoyu hafızadaki en son kalınan açı değerine döndürür
    arm.m3.write(arm.pos3); // 3. Servoyu hafızadaki en son kalınan açı değerine döndürür
    arm.m4.write(arm.pos4); // 4. Servoyu hafızadaki en son kalınan açı değerine döndürür

    isArmPowered = true; // Servoların başarıyla bağlandığını ve enerjilendiğini işaretler
  } // Koşul sonu
} // Fonksiyon sonu

void handleServoDetach() { // Servo motorların sinyal bağını kopararak boşa çıkaran fonksiyon
  if (isArmPowered) { // Eğer servo motorlar bağlı ve enerjili durumdaysa
    arm.m1.detach(); arm.m2.detach(); // 1. ve 2. servoların pin bağlantısını keserek torku kaldırır
    arm.m3.detach(); arm.m4.detach(); // 3. ve 4. servoların pin bağlantısını keserek motorları serbest bırakır
    isArmPowered = false; // Sistemde servoların enerjisiz olduğunu işaretler
  } // Koşul sonu
} // Fonksiyon sonu

void stopAllMotors() { // Sürüş motorlarını ve servo motorları toplu olarak durduran acil kapatma fonksiyonu
  applyDrivePower(STOPPED); // DC motorların gücünü keser ve frenleme prosedürünü uygular
  handleServoDetach(); // Tüm servo motorları boşa çıkararak mekanik yükü ve akım tüketimini sıfırlar
} // Fonksiyon sonu

/* =====================================================================
 *  OTOMATİK RESET (Kolu ev konum Konumuna getir)
 * ===================================================================== */
void handleAsyncReset() { // Robot kolunu ana/başlangıç konumuna adım adım (bloklamadan) getiren fonksiyon
  if (!isResetting || !isArmPowered ||
      RemoteXY.motorPSwitch_p0 == 0 ||
      RemoteXY.motorPSwitch_p1 == 0) return; // Reset aktif değilse, servolar bağlı değilse veya motor gücü kapalıysa fonksiyondan çık

  if (millis() - lastResetStepMs >= SERVO_SOFT_DELAY_MS) { // Yumuşak hareket için belirlenen adım süresi (15ms) geçtiyse
    bool allAtHome = true; // Tüm eklemlerin başlangıç konumuna (90 derece) ulaştığını varsayan kontrol değişkeni
    bool moved     = false;        // Bu adımda herhangi bir servonun hareket edip etmediğini tutan değişken

    if      (arm.pos1 < 90) { arm.pos1++; allAtHome = false; moved = true; } // 1. servo açı değeri 90'dan küçükse 1 artır, hedefe ulaşılmadığını belirt
    else if (arm.pos1 > 90) { arm.pos1--; allAtHome = false; moved = true; } // 1. servo açı değeri 90'dan büyükse 1 azalt, hedefe ulaşılmadığını belirt

    if      (arm.pos2 < 90) { arm.pos2++; allAtHome = false; moved = true; } // 2. servo açı değeri 90'dan küçükse 1 artır
    else if (arm.pos2 > 90) { arm.pos2--; allAtHome = false; moved = true; } // 2. servo açı değeri 90'dan büyükse 1 azalt

    if      (arm.pos3 < 90) { arm.pos3++; allAtHome = false; moved = true; } // 3. servo açı değeri 90'dan küçükse 1 artır
    else if (arm.pos3 > 90) { arm.pos3--; allAtHome = false; moved = true; } // 3. servo açı değeri 90'dan büyükse 1 azalt

    if      (arm.pos4 < 90) { arm.pos4++; allAtHome = false; moved = true; } // 4. servo açı değeri 90'dan küçükse 1 artır
    else if (arm.pos4 > 90) { arm.pos4--; allAtHome = false; moved = true; } // 4. servo açı değeri 90'dan büyükse 1 azalt

    // Yalnızca değişiklik varsa servolara yaz
    if (moved) { // Eğer bu döngü adımında en az bir motorun açısı değiştiyse
      arm.m1.write(arm.pos1); // 1. servonun yeni adım açısını motora gönderir
      arm.m2.write(arm.pos2); // 2. servonun yeni adım açısını motora gönderir
      arm.m3.write(arm.pos3); // 3. servonun yeni adım açısını motora gönderir
      arm.m4.write(arm.pos4); // 4. servonun yeni adım açısını motora gönderir
    } // Hareket kontrolü sonu

    if (allAtHome) { // Eğer tüm eklemler tam olarak 90 derece konumuna ulaştıysa
      isResetting = false; // Sıfırlama/Reset işlemini başarıyla sonlandırır
      playAlertTone(100); // İşlemin bittiğini bildirmek için 100ms süreli bir uyarı sesi çalar
    } // Ev konumu kontrolü sonu
    lastResetStepMs = millis(); // Bir sonraki adım zamanlaması için kronometreyi günceller
  } // Zaman kontrolü sonu
} // Fonksiyon sonu

/* =====================================================================
 *  SAYFA 0 — SÜRÜŞ SAYFASI
 * ===================================================================== */
void page0Funcs() { // Mobil arayüzdeki ilk sayfa (Sürüş Ekranı) kontrollerini yöneten fonksiyon
  static bool manualBuzzerActive = false; // Kare butonu (korna) durum takibi // Fonksiyon çağrıları arasında korna durumunu koruyan yerel değişken

  // --- MOTOR VE SÜRÜŞ KONTROLLERİ ---
  if (RemoteXY.motorPSwitch_p0 == 1) { // Eğer Sürüş sayfasındaki motor güç anahtarı açık ise
    handleServoAttach(); // Gerekliyse servo motorların pin bağlantılarını yapıp enerjilendirir
    MoveDirection nextDir = STOPPED; // Başlangıçta aracın bir sonraki yönünü DUR olarak belirler

    if (RemoteXY.forwardBtn_p0 == 1) { // Arayüzden İLERİ butonuna basılıyorsa
      if (isSensorInit && isSensorRunning && RemoteXY.distancePSwitch_p0 == 1) { // Sensör aktif, çalışır durumda ve mesafe anahtarı açıksa
        vl53l1xSensorSetup(); // Not: Kodda bu şekilde, sensörden güncel mesafe verisini okur
        
        // --- ENGEL VARSA SÜREKLİ ÖT ---
        if (RemoteXY.distanceNumber_p0 <= BRAKE_DISTANCE_MM) { // Mesafe sınır değerden (500mm) küçük veya eşitse (Engel varsa)
          nextDir = STOPPED; // Güvenlik amacıyla bir sonraki yönü DUR olarak zorla ayarlar (Aracı yürütmez)
          if (!obstacleBuzzerActive) { // Eğer engel alarm sesi hali hazırda başlamamışsa
            playAlertTone(0); // Kesintisiz çalmayı başlat // Buzzerı sürekli çalacak şekilde sıfır süreyle tetikler
            obstacleBuzzerActive = true; // Engel sesinin devrede olduğunu işaretler
          } // Alarm aktiflik kontrolü sonu
        } 
        else { // Sürüş yönünde engel yoksa veya güvenli mesafedeyse
          nextDir = FWD; // Aracın yönünü İLERİ (Forward) olarak belirler
          // Engel kalktı: Sadece korna da basılı değilse sustur
          if (obstacleBuzzerActive) { // Eğer önceden kalma bir engel sesi aktif ise
            if (RemoteXY.squareBtn_p0 == 0) stopBuzzerHard(); // Kullanıcı kornaya basmıyorsa ses donanımını tamamen sustur
            obstacleBuzzerActive = false; // Engel ses durumunu temizler
          } // Engel ses kontrolü sonu
        } // Mesafe kıyaslama sonu
      } else { // Sensör kapalıysa veya arızalıysa engel kontrolü yapmadan doğrudan ilerle
        nextDir = FWD; // Yönü İLERİ olarak belirler
      } // Sensör aktiflik kontrolü sonu
    } // İleri butonu basılma sonu
    // İleri gitme bırakıldıysa veya başka yöne basıldıysa engel sesini denetle
    else { // İleri butonuna basılmıyor, araç başka bir yöne gidiyor veya duruyorsa
      if (obstacleBuzzerActive) { // Önceden kalan engel sesi hala aktif ise
        if (RemoteXY.squareBtn_p0 == 0) stopBuzzerHard(); // Kullanıcı kornaya basmıyorsa sesi kes
        obstacleBuzzerActive = false; // Engel ses durumunu kapatır
      } // Engel ses sonu
      
      if      (RemoteXY.backBtn_p0  == 1) nextDir = BWD; // Geri butonuna basılıyorsa yönü GERİ yapar
      else if (RemoteXY.leftBtn_p0  == 1) nextDir = LFT; // Sol butonuna basılıyorsa yönü SOL yapar
      else if (RemoteXY.rightBtn_p0 == 1) nextDir = RGT; // Sağ butonuna basılıyorsa yönü SAĞ yapar
    } // Yön yönlendirmeleri sonu

    applyDrivePower(nextDir); // Belirlenen nihai yön komutunu motor sürücüye ve pinlere uygular

    // Reset butonu (Daire)
    if (RemoteXY.circleBtn_p0 == 1) { // Eğer daire (reset) butonuna basıldıysa
      isResetting    = true; // Kol otomatik reset döngüsünü başlatır
      lastResetStepMs = millis(); // Reset zamanlayıcı başlangıcını kaydeder
    } // Daire butonu sonu
  } // Motor anahtarı açık kontrolü sonu

  // --- VİTES YÜKSELTME (ÜÇGEN) ---
  if (RemoteXY.triangleBtn_p0 == 1) { // Üçgen (Vites Artırma) butonuna basıldıysa
    if (!isUpPressed) { // Butona yeni basılmışsa (Basılı tutma koruması)
      if (activeGearIdx < 6) { // Eğer maksimum vitese (7. vites, indeks 6) henüz ulaşılmadıysa
        activeGearIdx++; // Vites indeksini bir artırır
        activeMotorPWM              = GEAR_PWM_TABLE[activeGearIdx]; // Yeni vitese ait hız değerini tablodan seçer
        RemoteXY.gearInstrument_p0  = activeGearIdx + 1; // Arayüzdeki görsel vites göstergesini günceller (1 - 7)
        RemoteXY.gearNumber_p0      = RemoteXY.gearInstrument_p0; // Sayısal vites değer alanını günceller
        playAlertTone(activeGearIdx == 6 ? 600 : 100); // En üst vitese ulaştıysa uzun (600ms), normal artışsa kısa (100ms) ses verir
      } else playAlertTone(600);   // Zaten en üst vitesteyken basılıyorsa hata/uyarı amaçlı uzun ses verir
      isUpPressed = true; // Butona basıldığını kilitler (Tekrar girmesini engeller)
    } // Buton kilidi sonu
  } else isUpPressed = false; // Buton bırakıldığında kilidi açar

  // --- VİTES DÜŞÜRME (X) ---
  if (RemoteXY.xBtn_p0 == 1) { // X (Vites Düşürme) butonuna basıldıysa
    if (!isDownPressed) { // Butona yeni basılmışsa (Basılı tutma koruması)
      if (activeGearIdx > 0) { // Eğer minimum vitesten (1. vites, indeks 0) daha büyük bir vitesteyse
          activeGearIdx--; // Vites indeksini bir azaltır
          activeMotorPWM              = GEAR_PWM_TABLE[activeGearIdx]; // Yeni düşen vitese ait hız değerini tablodan atar
          RemoteXY.gearInstrument_p0  = activeGearIdx + 1; // Mobil arayüzdeki vites göstergesini aşağı çeker
          RemoteXY.gearNumber_p0      = RemoteXY.gearInstrument_p0; // Sayısal gösterge değerini günceller
          playAlertTone(activeGearIdx == 0 ? 600 : 100); // 1. vitese (en alta) düştüyse uzun (600ms), normal düşüşse kısa (100ms) ses verir
      } else playAlertTone(600);   // Zaten en alt vitesteyken basılıyorsa uyarı amaçlı uzun ses verir
      isDownPressed = true; // Butona basılma durumunu kilitler
    } // Buton kilidi sonu
  } else isDownPressed = false; // Buton bırakıldığında kilidi kaldırır

  // --- MANUEL KORNA (KARE BUTONU) ---
  if (RemoteXY.squareBtn_p0 == 1) { // Kare (Korna) butonuna basılıyorsa
    if (!manualBuzzerActive) { // Korna sesi henüz başlatılmadıysa
      playAlertTone(0); // Buton basılıyken kesintisiz çal // Buzzerı kesintisiz/sürekli çalma modunda tetikler
      manualBuzzerActive = true; // Kornanın manuel olarak aktif edildiğini işaretler
    } // İç kontrol sonu
  } else { // Kare butonuna basılmıyorsa (Bırakıldıysa)
    if (manualBuzzerActive) { // Eğer korna sesi aktif durumda kalmışsa
      // Korna bırakıldı: Sadece engel uyarısı da yoksa sustur
      if (!obstacleBuzzerActive) stopBuzzerHard(); // Eğer o sırada sensörden gelen acil bir engel uyarısı da yoksa sesi keser
      manualBuzzerActive = false; // Manuel korna durum bayrağını kapatır
    } // İç kontrol sonu
  } // Korna kontrolü sonu
} // Fonksiyon sonu

/* =====================================================================
 *  SAYFA 1 — KOL SAYFASI
 * ===================================================================== */
void page1Funcs() { // Mobil arayüzdeki ikinci sayfa (Robot Kol Ekranı) kontrollerini yöneten fonksiyon
  if (RemoteXY.motorPSwitch_p1 == 1) { // Eğer robot kol sayfasındaki motor güç anahtarı açık ise
    handleServoAttach(); // Servo motorların pin bağlantılarını kontrol eder ve aktif tutar

    if (millis() - servoUpdateMs >= 15) { // Her 15 milisaniyede bir çalışan yumuşak servo hareket kontrol penceresi
      bool isMoving = false; // Bu zaman diliminde herhangi bir eklemin hareket edip etmediğini tutan değişken

      // Manuel tuş basılıysa resetlemeyi iptal et
      if (RemoteXY.rightBtn_p1   || RemoteXY.leftBtn_p1    ||
          RemoteXY.forwardBtn_p1 || RemoteXY.backBtn_p1    ||
          RemoteXY.xBtn_p1       || RemoteXY.triangleBtn_p1||
          RemoteXY.squareBtn_p1  || RemoteXY.circleBtn_p1) { // Eğer koldaki yön veya eylem tuşlarından herhangi birine basılıyorsa
        if (isResetting) isResetting = false; // Eğer arka planda otomatik konum sıfırlama (reset) yapılıyorsa manuel müdahale ile iptal et
      } // İptal kontrolü sonu

      if (!isResetting) { // Eğer sistem otomatik sıfırlama modunda değilse manuel komutları işle
        // Taban (Sağ-Sol)
        if      (RemoteXY.rightBtn_p1 == 1 && arm.pos1 < 155) { arm.pos1++; isMoving = true; } // Sağ buton basılı ve açı sınırı aşılmadıysa taban açısını 1 artır
        else if (RemoteXY.leftBtn_p1  == 1 && arm.pos1 > 25)  { arm.pos1--; isMoving = true; } // Sol buton basılı ve açı sınırı aşılmadıysa taban açısını 1 azalt

        // Omuz (Yukarı-Aşağı)
        if      (RemoteXY.forwardBtn_p1 == 1 && arm.pos2 > 90)  { arm.pos2--; isMoving = true; } // İleri buton basılı ve omuz sınırı aşılmadıysa omuzu yukarı kaldır (açıyı azalt)
        else if (RemoteXY.backBtn_p1    == 1 && arm.pos2 < 180) { arm.pos2++; isMoving = true; } // Geri buton basılı ve omuz sınırı aşılmadıysa omuzu aşağı indir (açıyı artır)

        // Dirsek (2'şer derece adım)
        if      (RemoteXY.xBtn_p1       == 1 && arm.pos3 > 40) { arm.pos3 -= 2; isMoving = true; } // X butonu basılı ve limit içindeyse dirseği 2 derece kapat/aşağı indir
        else if (RemoteXY.triangleBtn_p1 == 1 && arm.pos3 < 90) { arm.pos3 += 2; isMoving = true; } // Üçgen butonu basılı ve limit içindeyse dirseği 2 derece aç/yukarı kaldır

        // Kıskaç (Tut-Bırak)
        if      (RemoteXY.squareBtn_p1  == 1 && arm.pos4 < 130) { arm.pos4++; isMoving = true; } // Kare butonu basılıysa kıskacı kapatma yönünde 1 derece hareket ettir
        else if (RemoteXY.circleBtn_p1  == 1 && arm.pos4 > 65)  { arm.pos4--; isMoving = true; } // Daire butonu basılıysa kıskacı açma yönünde 1 derece hareket ettir

        if (isMoving) { // Eğer yukarıdaki kontroller sonucu en az bir eklem açısı değiştiyse
          arm.m1.write(arm.pos1); // Güncel açı değerini 1. servo motora yazar
          arm.m2.write(arm.pos2); // Güncel açı değerini 2. servo motora yazar
          arm.m3.write(arm.pos3); // Güncel açı değerini 3. servo motora yazar
          arm.m4.write(arm.pos4); // Güncel açı değerini 4. servo motora yazar
        } // Yazma kontrolü sonu
      } // Reset durumu kontrolü sonu
      servoUpdateMs = millis(); // Bir sonraki servo kontrol periyodu için zamanı günceller
    } // Zaman penceresi sonu

    if (RemoteXY.verticalBtn_p1 == 1) { // Eğer Kol sayfasındaki dikey konumlandırma (reset) butonuna basılırsa
      isResetting    = true; // Kolu otomatik olarak başlangıç ev konumuna çekme sürecini başlatır
      lastResetStepMs = millis(); // Reset zamanlayıcı başlangıç anını kaydeder
    } // Reset butonu sonu
  } // Motor güç anahtarı kontrolü sonu
} // Fonksiyon sonu

/* =====================================================================
 *  SETUP
 * ===================================================================== */
void setup() { // Kart ilk açıldığında veya resetlendiğinde tek bir kez çalışan başlangıç ayar fonksiyonu
  RemoteXY_Init(); // RemoteXY haberleşme motorunu ve Wi-Fi ağ altyapısını başlatır
  Serial.begin(115200); // Bilgisayar bağlantısı / hata ayıklama için seri portu 115200 baud hızında başlatır

  // ÖNCE: Tüm PWM yapılarını temizle
  ledcDetach(BUZZER_PIN); // Buzzer pinine önceden atanmış olabilecek eski PWM sürücü bağlarını koparır
  pinMode(BUZZER_PIN, OUTPUT); // Buzzer pinini standart dijital çıkış olarak tanımlar
  digitalWrite(BUZZER_PIN, LOW); // İlk açılışta buzzerın ötmesini önlemek için pini sıfıra çeker

  // SONRA: Servoları güvenli timerlara ata (0 ve 3'ü pas geçelim)
  // v3.x sürümlerinde Timer 0 genelde WiFi/System için kritiktir.
  ESP32PWM::allocateTimer(1); // Servo kütüphanesi için donanımsal Timer 1'i ayırır ve kilitler
  ESP32PWM::allocateTimer(2); // Servo kütüphanesi için donanımsal Timer 2'i ayırır ve kilitler

  // Motor PWM - Kanal bazlı (Kanal 4 güvenlidir)
  ledcAttachChannel(MOTOR_PWM_PIN, 5000, 8, MOTOR_PWM_CH); // Motor PWM pinini 5000Hz frekans, 8-bit çözünürlükle Kanal 4'e bağlar

  // Diğer donanımlar
  vl53l1xSensorSetup(); // Lazer mesafe sensörünün I2C ve iç çalışma mod yapılandırmalarını yapar
  handleServoAttach(); // Robot kol servo motorlarını ilgili pinlere bağlar ve hazır konuma getirir

  // Motor yön pinleri
  int pSet[] = {MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN,
                MOTOR_C1_PIN, MOTOR_C2_PIN, MOTOR_D1_PIN, MOTOR_D2_PIN}; // Motor sürücü yön pinlerini içeren bir dizi oluşturur
  for (int p : pSet) pinMode(p, OUTPUT); // Dizideki tüm yön pinlerini döngüyle tek tek çıkış modu olarak ayarlar

  applyDrivePower(STOPPED); // İlk açılış anında aracın kontrolsüz hareket etmesini önlemek için motorları durdurur

  RemoteXY.gearInstrument_p0 = activeGearIdx + 1; // Arayüz vites grafik kadranını başlangıç vitesine (1) ayarlar
  RemoteXY.gearNumber_p0     = RemoteXY.gearInstrument_p0; // Arayüz sayısal vites alanına başlangıç değerini (1) yazar

  if (isSensorInit) { // Eğer lazer sensör donanımı başarıyla başlatılabildiyse
    strcpy(RemoteXY.distanceStr_p0, "Açık"); // Mobil ekrandaki mesafe durum yazısını "Açık" yapar
    RemoteXY.distancePSwitch_p0 = 1; // Mobil ekrandaki sensör anahtarını açık (1) konumuna getirir
  } else { // Sensör başlatılamadıysa (Donanım hatası veya bağlantı yoksa)
    strcpy(RemoteXY.distanceStr_p0, "Hata 0!"); // Ekrandaki durum yazısına "Hata 0!" yazar
    RemoteXY.distanceNumber_p0 = -9999; // Sayısal mesafe alanına hata belirtmek için -9999 yazar
  } // Sensör kontrol sonu

  RemoteXY.motorPSwitch_p0 = 1; // Sürüş sayfası motor gücü anahtarını başlangıçta açık konuma getirir
  RemoteXY.motorPSwitch_p1 = 1; // Kol sayfası motor gücü anahtarını başlangıçta açık konuma getirir
  strcpy(RemoteXY.motorStr_p0, "Açık"); // Sürüş sayfası motor durum metnini "Açık" olarak belirler
  strcpy(RemoteXY.motorStr_p1, "Açık"); // Kol sayfası motor durum metnini "Açık" olarak belirler
} // Setup fonksiyonu sonu

/* =====================================================================
 *  LOOP
 * ===================================================================== */
void loop() { // Kart çalıştığı sürece sonsuz bir döngüde kesintisiz çalışan ana fonksiyon
  RemoteXYEngine.handler(); // Mobil uygulama ile ESP32 arasındaki Wi-Fi veri paket trafiğini işler ve arayüzü günceller
  handleBuzzerAutoStop(); // Buzzerın zaman aşımı sürelerini kontrol eder ve zamanı dolan sesleri kapatır

  if (RemoteXY.connect_flag == 0) { ledcWrite(MOTOR_PWM_PIN, 0); setMotorPins(0, 0, 0, 0, 0, 0, 0, 0); return; } // Bağlantı koptuysa motor enerjisini ve pinlerini anında kesip döngüyü burada bitirir
                                                                                                                 // Sert fren yapmak yüksek akım çekmeye devam eder. Oyüzden yumuşak fren yapıldı.
  else if (RemoteXY.connect_flag == 1) { // Eğer mobil cihazla Wi-Fi bağlantısı sorunsuz kurulmuşsa
    if( RemoteXY.forwardBtn_p0 == 1 || RemoteXY.backBtn_p0 == 1 || RemoteXY.leftBtn_p0 == 1 || RemoteXY.rightBtn_p0 == 1 ) { // Sürüş butonlarından herhangi birine basılıyorsa
       lastPacketMs = millis(); // Bağlantının canlı olduğunu doğrulamak için son hareket paket zamanını günceller
    } // İç buton kontrol sonu
  } // Bağlantı durum kontrolü sonu

  if (millis() - lastPacketMs > 150) applyDrivePower(STOPPED); // Eğer arayüzden 150ms boyunca yeni bir hareket verisi gelmezse güvenli duruşa geçer

  if (RemoteXY.page_0 == 1) { // Kullanıcı mobil uygulamada SÜRÜŞ sayfasını (Sayfa 0) görüntülüyorsa
    if (isSensorInit && !isSensorRunning) { // Sensör donanımı var ama arka planda aktif okuma yapmıyorsa
      laserSensor.startContinuous(20); // Mesafe sensörünü 20ms periyotla sürekli okuma moduna geçirir
      isSensorRunning = true; // Sensörün okuma yaptığını işaretler
    } // Sensör çalışma kontrolü sonu

    if (RemoteXY.motorPSwitch_p0 == 0) { // Sürüş sayfasındaki motor güç anahtarı kapatıldıysa
      RemoteXY.motorPSwitch_p1 = 0; // Kol sayfasındaki motor güç anahtarını da otomatik olarak kapatır
      strcpy(RemoteXY.motorStr_p0, "Kapalı"); // Sürüş ekranındaki motor durum metnini "Kapalı" yapar
      strcpy(RemoteXY.motorStr_p1, "Kapalı"); // Kol ekranındaki motor durum metnini "Kapalı" yapar
      stopAllMotors(); // Sistemdeki tüm yürüyen motorları ve robot kol servolarını durdurup boşa çıkarır
    } else { // Sürüş sayfasındaki motor güç anahtarı açık ise
      RemoteXY.motorPSwitch_p1 = 1; // Kol sayfasındaki güç anahtarını da açık konuma senkronize eder
      strcpy(RemoteXY.motorStr_p0, "Açık"); // Sürüş ekranındaki motor durum metnini "Açık" yapar
      strcpy(RemoteXY.motorStr_p1, "Açık"); // Kol ekranındaki motor durum metnini "Açık" yapar
    } // Güç anahtarı kontrol sonu

    if (isSensorInit && isSensorRunning) { // Mesafe sensörü sorunsuz ayarlandıysa ve çalışıyorsa
      if (RemoteXY.distancePSwitch_p0 == 1) { // Arayüzdeki mesafe sensör anahtarı kullanıcı tarafından açıldıysa
        vl53l1xSensorDistance(); // Lazer sensörden mesafeyi oku ve arayüzdeki değişkenlere yaz
      } else { // Mesafe sensör anahtarı kapatıldıysa
        strcpy(RemoteXY.distanceStr_p0, "Kapalı"); // Ekranda mesafe bilgisi yerine "Kapalı" metnini göster
        RemoteXY.distanceNumber_p0 = 0; // Mesafe sayısal değer alanını sıfırlar
      } // Sensör anahtar kontrolü sonu
    } else { // Sensör donanımsal olarak hiç yoksa veya arızalıysa
      strcpy(RemoteXY.distanceStr_p0, "Hata 0!"); // Ekrandaki veri alanına kalıcı donanım hatası yazar
      RemoteXY.distanceNumber_p0 = -9999; // Sayısal alana arıza kodu olan -9999 değerini atar
    } // Sensör donanım kontrolü sonu

    page0Funcs(); // Sürüş sayfasına ait vites, korna ve yön buton eylemlerini çalıştırır
  } // Sayfa 0 sonu
  
  // ---- SAYFA 1: KOL ----
  else if (RemoteXY.page_1 == 1) { // Kullanıcı mobil uygulamada ROBOT KOL sayfasını (Sayfa 1) görüntülüyorsa
    if (isSensorInit && isSensorRunning) { // Sensör arka planda hala sürekli ölçüm modunda çalışıyorsa
      laserSensor.stopContinuous(); // Güç tasarrufu ve işlemci rahatlatması için sürekli ölçümü durdurur
      isSensorRunning = false; // Sensörün durdurulduğunu işaretler
    } // Sensör kontrol sonu

    if (RemoteXY.motorPSwitch_p1 == 0) { // Kol sayfasındaki motor güç anahtarı kapatıldıysa
      RemoteXY.motorPSwitch_p0 = 0; // Sürüş sayfasındaki motor güç anahtarını da kapatarak senkronize eder
      strcpy(RemoteXY.motorStr_p0, "Kapalı"); // Sürüş ekranı durum metnini "Kapalı" olarak günceller
      strcpy(RemoteXY.motorStr_p1, "Kapalı"); // Kol ekranı durum metnini "Kapalı" olarak günceller
      stopAllMotors(); // Güvenlik amacıyla tüm DC motorları ve servoları tamamen durdurur
    } else { // Kol sayfasındaki motor güç anahtarı açık ise
      RemoteXY.motorPSwitch_p0 = 1; // Sürüş sayfasındaki anahtarı da açık konuma getirir
      strcpy(RemoteXY.motorStr_p0, "Açık"); // Sürüş ekranı durum metnini "Açık" yapar
      strcpy(RemoteXY.motorStr_p1, "Açık"); // Kol ekranı durum metnini "Açık" yapar
    } // Güç anahtarı kontrol sonu
    page1Funcs(); // Robot kolun eklem butonlarını, sınırlarını ve manuel hareket kodlarını çalıştırır
  } // Sayfa 1 sonu

  handleAsyncReset(); // Eğer reset/sıfırlama emri verilmişse kolu ev konumuna adım adım getiren asenkron döngüyü sürdürür
} // Ana loop fonksiyonu sonu