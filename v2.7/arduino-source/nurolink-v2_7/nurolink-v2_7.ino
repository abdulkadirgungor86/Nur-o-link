/*******************************************************************************
 * PROJE ADI          : Nur-o-link v2.7
 * SİSTEM TANIMI      : (Gelişmiş) 7 Vitesli ve 4 Eklemli (8 Yönlü) Kontrol Sistemi
 * YAZILIM MİMARİSİ   : Non-blocking (Bloklamayan) State Machine (Durum Makinesi)
 * GELİŞTİRİCİ        : Abdulkadir GÜNGÖR (a.kadir.gungor.86@gmail.com)
 * TARİH              : 2026-03-31
 * * TEKNİK GEREKSİNİM  : Arduino IDE - ESP32 Core 2.0.11 
 *                        ESP32 Kart Sürümü v2.0.11 Olmalıdır (Güncel Sürümlerle Çalışmaz. Hata verir!)
 ** BOARD VE KÜTÜPHANE SÜRÜMLERİ:
 *                        1) En Son "ESP32 v2.0.11" Sorunsuz Çalıştı. 
 *                        2) En Son "DabbleESP32 v1.5.1" Sorunsuz Çalıştı. 
 *                        3) En Son "ESP32Servo v3.1.3" Sorunsuz Çalıştı.                     
 * * DONANIM BİLEŞENLERİ (REX-8in1-V2 Donanım Seti) **
 * - Mikrodenetleyici : ESP32 Dev Module (WROOM-32)
 * - Motor Sürücü     : TB6612FNG Entegre Sürücü (4x DC Motor Destekli)
 * - Robotik Kol      : 4x Servo Motor (4-DOF / 8-Yönlü Kontrol)
 * - Sensörler        : HC-SR04 Ultrasonik Mesafe Sensörü
 * - Geri Bildirim    : Pasif Buzzer (Frekans Modülasyonu Destekli)
 * * KONTROL ARA YÜZÜ   : Dabble App (Bluetooth üzerinden Gamepad Modülü)
 *******************************************************************************/

#define PROJECT_NAME "Nur-o-link v2.7" // Bluetooth cihaz adını tanımlar
#define CUSTOM_SETTINGS                // Dabble kütüphanesini özelleştirmek için kullanılır
#define INCLUDE_GAMEPAD_MODULE         // Sadece Gamepad (oyun kolu) modülünü hafızaya dahil eder

#include <DabbleESP32.h>               // Bluetooth üzerinden Gamepad verisi almak için gerekli kütüphane
#include <Arduino.h>                   // ESP32 temel fonksiyonlarını içeren standart kütüphane
#include <ESP32Servo.h>                // ESP32 işlemcisiyle uyumlu servo motor kontrol kütüphanesi

/* --- [I/O PİN TANIMLAMALARI] --- */
#define MOTOR_PWM_PIN 13               // TB6612FNG motor sürücüsünün genel hız kontrol pini
#define MOTOR_A1_PIN 15                // Ön Sol Motorun yön kontrol pini (+)
#define MOTOR_A2_PIN 23                // Ön Sol Motorun yön kontrol pini (-)
#define MOTOR_B1_PIN 32                // Ön Sağ Motorun yön kontrol pini (+)
#define MOTOR_B2_PIN 33                // Ön Sağ Motorun yön kontrol pini (-)
#define MOTOR_C1_PIN 5                 // Arka Sol Motorun yön kontrol pini (+)
#define MOTOR_C2_PIN 4                 // Arka Sol Motorun yön kontrol pini (-)
#define MOTOR_D1_PIN 27                // Arka Sağ Motorun yön kontrol pini (+)
#define MOTOR_D2_PIN 14                // Arka Sağ Motorun yön kontrol pini (-)

#define TRIG_PIN 17                    // HC-SR04 ultrasonik sensörün tetikleme pini
#define ECHO_PIN 16                    // HC-SR04 ultrasonik sensörün geri dönüş (eko) pini
#define BUZZER_PIN 25                  // Sesli uyarılar için kullanılan buzzer pini

/* --- [SABİTLER VE DEĞİŞKENLER] --- */
const int SERVO_SOFT_DELAY_MS = 20;    // Servo motorların adım adım yumuşak hareket etme hızı
const int MOTOR_CHAN = 4;              // DC motorlar için atanan ESP32 PWM kanalı
const int BUZZER_CHAN = 5;             // Buzzer ses tonu üretimi için atanan ESP32 PWM kanalı

const int GEAR_PWM_TABLE[] = {60, 92, 125, 157, 190, 222, 255}; // 7 kademeli hız tablosu (0-255 arası)
int activeGearIdx = 0;                 // Sistemin açılıştaki varsayılan vites endeksi (1. vites)
int activeMotorPWM = 60;              // Motorlara gönderilen güncel hız sinyal değeri

enum OperationMode { DRIVE_MODE, ARM_MODE }; // Sürüş ve Kol kontrolü olmak üzere iki çalışma modu tanımlar
OperationMode currentSystemMode = DRIVE_MODE; // Sistemin başlangıç modunu Sürüş olarak belirler
enum MoveDirection { STOPPED, FWD, BWD, LFT, RGT }; // Hareket yönlerini tanımlayan liste
MoveDirection lastMotorCmd = STOPPED;  // Motorlara gönderilen son komutu hafızada tutar

/* --- [GÜVENLİK VE FİLTRELEME] --- */
bool isGuardEnabled = true;             // Engel algılama sisteminin açık/kapalı durumunu tutar
bool isWayBlocked = false;              // Mesafe sınırı aşıldığında true olur ve ileri sürüşü keser
int bufferCounter = 0;                  // Engelden uzaklaşınca sürüşe izin vermek için gereken doğrulama sayacı
float distanceBuffer[3] = {0, 0, 0};    // Sensörden gelen parazitleri temizlemek için kullanılan dizi
int samplesCollected = 0;               // Toplanan veri örneği sayısını takip eder
float filteredDistance = 100.0;         // Üçlü filtrelemeden geçmiş temiz mesafe verisi
int dynamicStopThreshold = 40;          // Hıza (vitese) göre değişen dinamik durma mesafesi
int dynamicSafeThreshold = 50;          // Yeniden hareket etmek için gereken güvenli boşluk mesafesi

/* --- [ZAMANLAYICILAR VE BAYRAKLAR] --- */
unsigned long sensorUpdateMs = 0;       // Ultrasonik sensörün okuma periyodunu yöneten zamanlayıcı
unsigned long startHoldMs = 0;          // Start butonuna ne kadar süre basıldığını ölçen zamanlayıcı
unsigned long selectHoldMs = 0;         // Select butonuna ne kadar süre basıldığını ölçen zamanlayıcı
unsigned long servoUpdateMs = 0;        // Servoların tazeleme sıklığını yöneten zamanlayıcı
unsigned long audioAutoCutoffMs = 0;    // Buzzer'ın belirli bir süre sonra otomatik susmasını sağlayan zamanlayıcı
unsigned long lastResetStepMs = 0;      // Otomatik toplama (reset) işleminin adım zamanlayıcısı
unsigned long lastPacketMs = 0;         // Bluetooth bağlantı koptuğunda güvenli duruş sağlayan watchdog süresi

bool startTaskExecuted = false;         // Start uzun basış görevinin bir kez çalışmasını sağlayan bayrak
bool selectTaskExecuted = false;        // Select uzun basış görevinin bir kez çalışmasını sağlayan bayrak
bool isResetting = false;               // Kol toplama işleminin aktif olup olmadığını belirtir
bool isArmPowered = false;              // Servoların elektriksel olarak bağlı olup olmadığını takip eder
bool pTri = false, pCro = false, pSta = false, pSel = false, pCir = false; // Önceki tuş durumları

struct RobotArm {
    int pos1 = 90; int pos2 = 90; int pos3 = 90; int pos4 = 90; // 4 servonun güncel açı değerlerini tutar
    Servo m1, m2, m3, m4;               // 4 adet servo motor nesnesini tanımlar
} arm;

/* --- [YARDIMCI FONKSİYONLAR] --- */

// Mesafe ölçümü yapan fonksiyon
float measureDistance() {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2); // Sinyal hattını temizler
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10); // 10 mikrosaniye boyunca tetikleme sinyali gönderir
    digitalWrite(TRIG_PIN, LOW); // Tetiklemeyi sonlandırır
    long timeDelta = pulseIn(ECHO_PIN, HIGH, 25000); // Yankı sinyalinin geri dönme süresini ölçer
    return (timeDelta == 0) ? 250.0 : (timeDelta / 2.0) / 29.1; // Süreyi santimetreye çevirir
}

// Dinamik durma sınırlarını güncelleyen fonksiyon
void updateDynamicThresholds() {
    dynamicStopThreshold = map(activeGearIdx, 0, 6, 40, 60); // Vites arttıkça durma mesafesini artırır
    dynamicSafeThreshold = dynamicStopThreshold + 10; // Güvenli sürüş sınırını durma sınırının 10cm üzerine kurar
}

// Sensör verilerini filtreleyen fonksiyon
void processMovingAverage(float raw) {
    distanceBuffer[2] = distanceBuffer[1]; distanceBuffer[1] = distanceBuffer[0]; distanceBuffer[0] = raw; // Verileri kaydırır
    if (samplesCollected < 3) samplesCollected++; // İlk 3 veri toplanana kadar sayacı artırır
    if (samplesCollected == 1) filteredDistance = distanceBuffer[0]; // Tek veri varsa direkt kullanır
    else if (samplesCollected == 2) filteredDistance = (distanceBuffer[0] + distanceBuffer[1]) / 2.0; // İki verinin ortalamasını alır
    else filteredDistance = (distanceBuffer[0] + distanceBuffer[1] + distanceBuffer[2]) / 3.0; // Üç verinin hareketli ortalamasını alır
}

// Motor yön pinlerini topluca ayarlayan fonksiyon
void setMotorPins(int p1, int p2, int p3, int p4, int p5, int p6, int p7, int p8) {
    digitalWrite(MOTOR_A1_PIN, p1); digitalWrite(MOTOR_A2_PIN, p2); // Ön sol motor yönü
    digitalWrite(MOTOR_B1_PIN, p3); digitalWrite(MOTOR_B2_PIN, p4); // Ön sağ motor yönü
    digitalWrite(MOTOR_C1_PIN, p5); digitalWrite(MOTOR_C2_PIN, p6); // Arka sol motor yönü
    digitalWrite(MOTOR_D1_PIN, p7); digitalWrite(MOTOR_D2_PIN, p8); // Arka sağ motor yönü
}

// Motorlara güç ve yön veren fonksiyon
void applyDrivePower(MoveDirection dir) {
    if (dir == STOPPED) {
        ledcWrite(MOTOR_CHAN, 0); // Hız sinyalini sıfırlar
        setMotorPins(0, 0, 0, 0, 0, 0, 0, 0); // Tüm motor pinlerini lojik sıfıra çeker
    } else {
        ledcWrite(MOTOR_CHAN, activeMotorPWM); // Ayarlı vites gücünü uygular
        switch (dir) {
            case FWD: setMotorPins(1, 0, 1, 0, 1, 0, 1, 0); break; // İleri hareket lojiği
            case BWD: setMotorPins(0, 1, 0, 1, 0, 1, 0, 1); break; // Geri hareket lojiği
            case LFT: setMotorPins(0, 1, 0, 1, 1, 0, 1, 0); break; // Sola dönme (tank dönüşü)
            case RGT: setMotorPins(1, 0, 1, 0, 0, 1, 0, 1); break; // Sağa dönme (tank dönüşü)
        }
    }
}

// Servo motorların pin bağlantılarını yapan fonksiyon
void handleServoAttach() {
    if (!isArmPowered) { // Eğer enerji yoksa bağlantıyı başlatır
        arm.m1.attach(2, 600, 2500); arm.m2.attach(26, 600, 2500); // 1. ve 2. servo pinleri ve limitleri
        arm.m3.attach(18, 600, 2500); arm.m4.attach(19, 600, 2500); // 3. ve 4. servo pinleri ve limitleri
        isArmPowered = true; // Enerji verildi bayrağını işaretler
    }
}

// Servo motorların bağlantısını kesen (serbest bırakan) fonksiyon
void handleServoDetach() {
    if (isArmPowered) { // Eğer enerji varsa bağlantıyı keser
        arm.m1.detach(); arm.m2.detach(); arm.m3.detach(); arm.m4.detach(); // Tüm servoları boşa çıkarır
        isArmPowered = false; // Enerji kesildi bayrağını işaretler
    }
}

// Kısa süreli sesli uyarı veren fonksiyon
void playAlertTone(int duration) {
    ledcWrite(BUZZER_CHAN, 160); // Buzzer'ı 160 frekans gücünde çalıştırır
    audioAutoCutoffMs = millis() + duration; // Belirlenen süre sonunda durması için zamanı kurar
}

// Tüm hareket sistemini durduran acil durum fonksiyonu
void fullSafetyReset() {
    applyDrivePower(STOPPED); // DC motorları anında durdurur
    lastMotorCmd = STOPPED; // Son komut hafızasını temizler
    isWayBlocked = false; bufferCounter = 0; samplesCollected = 0; // Mesafe sistemini sıfırlar
}

// Robot kolunu başlangıç (güvenli) pozisyonuna getiren fonksiyon
void handleAsyncReset() {
    if (!isResetting) return; // Resetleme aktif değilse fonksiyondan çıkar
    if (millis() - lastResetStepMs >= SERVO_SOFT_DELAY_MS) { // Yumuşak hareket süresi dolmuşsa
        bool allAtHome = true; // Tüm servoların hedefe varıp varmadığını kontrol eder
        if (arm.pos1 < 90) { arm.pos1++; allAtHome = false; } else if (arm.pos1 > 90) { arm.pos1--; allAtHome = false; } // 1. Eklemi 90'a yaklaştırır
        if (arm.pos2 < 85) { arm.pos2++; allAtHome = false; } else if (arm.pos2 > 85) { arm.pos2--; allAtHome = false; } // 2. Eklemi 90'a yaklaştırır
        if (arm.pos4 < 90) { arm.pos4++; allAtHome = false; } else if (arm.pos4 > 90) { arm.pos4--; allAtHome = false; } // 4. Eklemi 90'a yaklaştırır
        if (arm.pos3 < 80) { arm.pos3 += 2; if (arm.pos3 > 80) arm.pos3 = 80; allAtHome = false; } // 3. Eklemi 80'e çeker
        else if (arm.pos3 > 80) { arm.pos3 -= 2; if (arm.pos3 < 80) arm.pos3 = 80; allAtHome = false; }

        if (isArmPowered) { // Eğer servolarda enerji varsa açıları motorlara yazar
            arm.m1.write(arm.pos1); arm.m2.write(arm.pos2); 
            arm.m3.write(arm.pos3); arm.m4.write(arm.pos4);
        }
        if (allAtHome) { isResetting = false; playAlertTone(100); } // Hedefe varıldıysa resetlemeyi bitirir
        lastResetStepMs = millis(); // Adım zamanlayıcıyı günceller
    }
}

/* --- [ANA KURULUM - SETUP] --- */
void setup() {
    Serial.begin(115200); // Seri haberleşmeyi başlatır (hata ayıklama için)
    ledcSetup(MOTOR_CHAN, 5000, 8); ledcAttachPin(MOTOR_PWM_PIN, MOTOR_CHAN); // Motor hız sinyalini ayarlar
    ledcSetup(BUZZER_CHAN, 800, 8); ledcAttachPin(BUZZER_PIN, BUZZER_CHAN); // Buzzer ton sinyalini ayarlar
    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1); // Servo motorlar için zamanlayıcı (timer) atar
    ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3); // Diğer 2 servo için zamanlayıcı atar
    handleServoAttach(); // Başlangıçta robot koluna güç verir
    pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT); // Mesafe sensörü pin yönlerini belirler
    int pSet[] = {15, 23, 32, 33, 5, 4, 27, 14}; // Tüm motor pinlerini diziye alır
    for(int p : pSet) pinMode(p, OUTPUT); // Dizideki tüm pinleri çıkış olarak ayarlar
    Dabble.begin(PROJECT_NAME); // Bluetooth yayını proje adıyla başlatır
    fullSafetyReset(); // Sistemi tam güvenli konumda başlatır
    isResetting = true; // Açılışta kolun kendini toplamasını sağlar
}

/* --- [ANA DÖNGÜ - LOOP] --- */
void loop() {
    Dabble.processInput(); // Gelen Bluetooth paketlerini çözer ve işler
 
    if (!Dabble.isAppConnected()) { fullSafetyReset(); return; } // Bağlantı koptuysa sistemi kilitler ve bekler

    if (Dabble.isAppConnected() && (GamePad.isUpPressed() || GamePad.isDownPressed() || GamePad.isLeftPressed() || GamePad.isRightPressed() || 
        GamePad.isSquarePressed() || GamePad.isCirclePressed() || GamePad.isTrianglePressed() || GamePad.isCrossPressed())) {
        lastPacketMs = millis(); // Herhangi bir tuşa basıldığında watchdog süresini yeniler
    }

    if (millis() - lastPacketMs > 500 && lastMotorCmd != STOPPED) {
        applyDrivePower(STOPPED); lastMotorCmd = STOPPED; // 500ms komut gelmezse DC motorları durdurur
    }

    /* --- [START BUTONU: KİLİT VE SESLİ UYARI YÖNETİMİ] --- */
    if (GamePad.isStartPressed()) { // Start butonuna basılıyorsa
        if (startHoldMs == 0) startHoldMs = millis(); // Basılma başlangıç anını kaydeder
        
        if (millis() - startHoldMs >= 1200) { // Butona 1.2 saniyeden fazla basıldıysa
            fullSafetyReset(); // DC motorları anında durdurur
            handleServoDetach(); // Servoların enerjisini keser (serbest bırakır)
            
            if (millis() % 200 < 100) ledcWrite(BUZZER_CHAN, 160); // Kesik kesik bip sesi için sesi açar
            else ledcWrite(BUZZER_CHAN, 0); // Sesi kapatır (stroboskobik ses efekti)

            startTaskExecuted = true; // Uzun basış işleminin başladığını işaretler
            return; // Elini çekene kadar döngünün geri kalanını çalıştırmaz (tam kilit)
        }
    } else { // Buton bırakıldıysa
        if (startHoldMs > 0 && !startTaskExecuted && (millis() - startHoldMs < 1200)) { // Kısa basılmışsa
            if (currentSystemMode == ARM_MODE) { // Sadece kol modundaysak sürüş moduna geçer
                currentSystemMode = DRIVE_MODE; // Sürüş modunu aktif eder
                playAlertTone(200); // Onay tonu çalar
            }
        }
        if (startTaskExecuted) { // Eğer uzun basıştan (kilitten) çıkılıyorsa
             ledcWrite(BUZZER_CHAN, 0); // Buzzer'ı susturur
             handleServoAttach(); // Servolara enerjiyi geri verir
        }
        startHoldMs = 0; startTaskExecuted = false; // Sayaçları ve işaretleri sıfırlar
    }

    handleAsyncReset(); // Arka planda kol toplama sürecini devam ettirir

    if (audioAutoCutoffMs > 0 && millis() >= audioAutoCutoffMs) { 
        ledcWrite(BUZZER_CHAN, 0); audioAutoCutoffMs = 0; // Süresi dolan uyarı sesini kapatır
    }

    if (currentSystemMode == DRIVE_MODE && isGuardEnabled && (lastMotorCmd == STOPPED || lastMotorCmd == FWD)) { // Sürüş ve koruma aktifse
        if (millis() - sensorUpdateMs >= 60) { // 60ms aralıklarla mesafe ölçer
            float raw = measureDistance(); // Ham mesafe verisini alır
            if (raw > 0 && raw <= 250) { // Geçerli bir veri aralığındaysa
                processMovingAverage(raw); // Veriyi filtreye sokar
                updateDynamicThresholds(); // Dinamik sınırları günceller
                if (!isWayBlocked) { // Yol açıkken engel kontrolü yapar
                    if (filteredDistance <= dynamicStopThreshold) { // Mesafe sınırı aşıldıysa
                        if (lastMotorCmd == FWD) { applyDrivePower(STOPPED); lastMotorCmd = STOPPED; } // İlerlemeyi durdurur
                        isWayBlocked = true; bufferCounter = 0; // Yolun kapalı olduğunu işaretler
                    }
                } else { // Yol kapalıyken açılma kontrolü yapar
                    if (filteredDistance > dynamicSafeThreshold) { // Güvenli boşluk oluştuysa
                        bufferCounter++; if (bufferCounter >= 5) { isWayBlocked = false; bufferCounter = 0; } // 5 doğrulama sonrası yolu açar
                    } else { bufferCounter = 0; } // Mesafe tekrar düşerse sayacı sıfırlar
                }
            }
            sensorUpdateMs = millis(); // Sensör zamanlayıcısını günceller
        }
        if (isWayBlocked && GamePad.isUpPressed() && audioAutoCutoffMs == 0) playAlertTone(80); // Engel varken ileri basılırsa biler
    }

    if (currentSystemMode == DRIVE_MODE) processDriveMode(); else processArmMode(); // Aktif moda göre kontrol fonksiyonunu çağırır

    pSta = GamePad.isStartPressed(); pSel = GamePad.isSelectPressed(); // Bir sonraki döngü için tuş durumlarını kaydeder
    pCir = GamePad.isCirclePressed(); pTri = GamePad.isTrianglePressed(); pCro = GamePad.isCrossPressed(); // Tuş durumlarını günceller
}

/* --- [SÜRÜŞ MODU YÖNETİMİ] --- */
void processDriveMode() {
    handleServoAttach(); // Sürüşte servoların dik durması için gücü korur

    if (GamePad.isSelectPressed()) { // Select butonuna basılıyorsa
        if (selectHoldMs == 0) selectHoldMs = millis(); // Zamanlamayı başlatır
        if (millis() - selectHoldMs >= 1200 && !selectTaskExecuted) { // 1.2 saniye uzun basıldıysa
            isGuardEnabled = !isGuardEnabled; // Engel korumasını açar veya kapatır
            if (!isGuardEnabled) { isWayBlocked = false; bufferCounter = 0; } // Kapatıldıysa tüm engelleri temizler
            playAlertTone(isGuardEnabled ? 500 : 200); // Duruma göre bildirim sesi verir
            selectTaskExecuted = true; // Uzun görevi işaretler
        }
    } else { // Select bırakıldıysa
        if (selectHoldMs > 0 && !selectTaskExecuted && (millis() - selectHoldMs < 1200)) { // Kısa basılmışsa
            fullSafetyReset(); // Sürüşü durdurur
            currentSystemMode = ARM_MODE; // Robot kol moduna geçer
            playAlertTone(200); // Onay tonu verir
        }
        selectHoldMs = 0; selectTaskExecuted = false; // Sayaçları sıfırlar
    }

    if (GamePad.isSquarePressed()) { // Kare butonu korna görevi görür
        ledcWrite(BUZZER_CHAN, 160); // Yüksek frekansta buzzer çalıştırır
        audioAutoCutoffMs = millis() + 100; // 100ms sonra susmasını sağlar
    }

    if (GamePad.isCirclePressed() && !pCir) { // Yuvarlak butonuna basıldığı an (tek tetik)
        isResetting = true; lastResetStepMs = millis(); playAlertTone(300); // Kol toplama işlemini başlatır
    }

    if (GamePad.isTrianglePressed() && !pTri) { // Üçgen butonu (Vites artırma)
        if (activeGearIdx < 6) { activeGearIdx++; activeMotorPWM = GEAR_PWM_TABLE[activeGearIdx]; playAlertTone(activeGearIdx==6?600:100); } // Hızı artırır
        else playAlertTone(600); updateDynamicThresholds(); // Sınırda ise farklı ton verir ve dinamik mesafeyi günceller
    }
    if (GamePad.isCrossPressed() && !pCro) { // Çarpı butonu (Vites düşürme)
        if (activeGearIdx > 0) { activeGearIdx--; activeMotorPWM = GEAR_PWM_TABLE[activeGearIdx]; playAlertTone(activeGearIdx==0?600:100); } // Hızı düşürür
        else playAlertTone(600); updateDynamicThresholds(); // Sınırda ise farklı ton verir ve dinamik mesafeyi günceller
    }

    bool u = GamePad.isUpPressed(), d = GamePad.isDownPressed(), l = GamePad.isLeftPressed(), r = GamePad.isRightPressed(); // Hareket tuşlarını okur
    if (d || l || r) { isWayBlocked = false; bufferCounter = 0; } // Geri ve yanlara giderken mesafe engelini geçersiz kılar
    MoveDirection nextDir = STOPPED; // Yeni yönü başlangıçta duruyor olarak belirler
    if (u) { if (isWayBlocked && isGuardEnabled) nextDir = STOPPED; else nextDir = FWD; } // Engel varsa durdurur yoksa ileri sürer
    else if (d) nextDir = BWD; else if (l) nextDir = LFT; else if (r) nextDir = RGT; // Diğer yönleri belirler
    if (nextDir != lastMotorCmd) { applyDrivePower(nextDir); lastMotorCmd = nextDir; } // Yön değişmişse motorlara yeni gücü gönderir
}

/* --- [KOL KONTROL MODU YÖNETİMİ] --- */
void processArmMode() {
    handleServoAttach(); // Servolara sürekli enerji vererek pozisyonlarını korur

    if (GamePad.isSelectPressed()) { // Select butonuna basılıyorsa
        if (selectHoldMs == 0) selectHoldMs = millis(); // Zamanlamayı başlatır
        if (millis() - selectHoldMs >= 1200 && !selectTaskExecuted) { // Uzun basıldıysa
            isResetting = true; // Kolu otomatik toplama moduna sokar
            lastResetStepMs = millis(); // Zamanlayıcıyı günceller
            selectTaskExecuted = true; // Uzun görevi işaretler
            playAlertTone(400); // Onay tonu verir
        }
    } else { // Select bırakıldıysa
        selectHoldMs = 0; selectTaskExecuted = false; // Sayaçları sıfırlar
    }

    if (!isResetting && (millis() - servoUpdateMs >= 20)) { // Manuel kontrol modu ve 20ms periyot kontrolü
        bool isMoving = false; // Herhangi bir servonun hareket edip etmediğini tutar
        if (GamePad.isRightPressed()) { if (arm.pos1 < 155) { arm.pos1++; isMoving = true; } } // 1. Eklemi sağa çevirir
        else if (GamePad.isLeftPressed()) { if (arm.pos1 > 25) { arm.pos1--; isMoving = true; } } // 1. Eklemi sola çevirir
        if (GamePad.isUpPressed()) { if (arm.pos2 > 85) { arm.pos2--; isMoving = true; } } // 2. Eklemi yukarı kaldırır
        else if (GamePad.isDownPressed()) { if (arm.pos2 < 160) { arm.pos2++; isMoving = true; } } // 2. Eklemi aşağı indirir
        if (GamePad.isCrossPressed()) { if (arm.pos3 > 40) { arm.pos3 -= 2; isMoving = true; } } // 3. Eklemi kapatır
        else if (GamePad.isTrianglePressed()) { if (arm.pos3 < 80 ) { arm.pos3 += 2; isMoving = true; } } // 3. Eklemi açar
        if (GamePad.isSquarePressed()) { if (arm.pos4 < 120 ) { arm.pos4++; isMoving = true; } } // 4. Eklemi (kıskaç) kapatır
        else if (GamePad.isCirclePressed()) { if (arm.pos4 > 70 ) { arm.pos4--; isMoving = true; } } // 4. Eklemi (kıskaç) açar
        
        if (isMoving) { // Hareket varsa servolara yeni açıları yazar
            arm.m1.write(arm.pos1); arm.m2.write(arm.pos2); 
            arm.m3.write(arm.pos3); arm.m4.write(arm.pos4);
        }
        servoUpdateMs = millis(); // Servo zamanlayıcısını günceller
    }
}