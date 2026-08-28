# Sahte GPS — STM32 üzerinden NMEA / UART

Uçuş kontrolcüsüne (ArduPilot) gerçek bir GPS modülü olmadan, **seri porttan elle girilen sabit bir konum** besleyen STM32 firmware'i.

Kapalı alanda masa üstü testler için tasarlandı: gimbal ROI ("Point Camera Here") denemeleri, EKF origin kurulumu, konuma bağlı davranışların doğrulanması gibi işler için gerçek GPS fix'i beklemeden çalışabilirsin.

> [!WARNING]
> Bu firmware uçuş kontrolcüsüne **sahte konum** besler. Yalnızca yer testleri içindir; gerçek uçuşta kesinlikle kullanma.

---

## Nasıl çalışır

```
PC (Serial Monitor)  --USART2 (PA2/PA3)-->  STM32  --USART1 (PA9)-->  FC RX
      "41.008200,28.978400,150,90"                    $GPGGA / $GPRMC / $GPHDT
```

Açılışta kodda tanımlı sabit konumu 5 Hz hızında NMEA olarak yayınlar. Serial Monitor'den yeni bir koordinat girdiğinde, en fazla 200 ms içinde yeni konum FC'ye gitmeye başlar.

---

## Donanım

| İşlev | Port | Pin | Baud |
|---|---|---|---|
| Debug / komut girişi | USART2 | RX=`PA3`, TX=`PA2` | 115200 |
| NMEA çıkışı (FC'ye) | USART1 | RX=`PA10`, TX=`PA9` | 38400 (ayarlanabilir) |

**Kablolama:**

```
STM32 PA9  ──────────►  FC  RX   (GPS portunun RX pini)
STM32 GND  ──────────►  FC  GND  (ORTAK GND ŞART)
```

Ortak GND olmadan sinyalin referansı olmaz ve FC hiçbir veri okuyamaz. STM32'yi PC USB'sinden, FC'yi bataryadan besliyorsan bu bağlantı özellikle kritiktir.

> Nucleo kartlarda `PA2`/`PA3` genellikle ST-Link'in USB sanal COM portuna bağlıdır, ek adaptör gerekmez. Blackpill gibi ST-Link VCP'si olmayan kartlarda Serial Monitor'e erişmek için `PA2`/`PA3`'e ayrı bir USB-TTL çevirici bağlaman gerekir.

---

## Kurulum

1. Arduino IDE'ye [STM32duino](https://github.com/stm32duino/Arduino_Core_STM32) core'unu kur.
2. Kartını ve programlayıcını seç (**Tools → Board / Upload method**).
3. `NMEA_GPS.ino` dosyasını aç ve flashla.
4. Serial Monitor'ü **115200** baud ile aç, satır sonunu **Newline** olarak ayarla.

> [!IMPORTANT]
> Serial Monitor'de satır sonu ayarı **Newline** değilse komutlar hiçbir zaman işlenmez.

---

## Seri komutlar

| Komut | Açıklama |
|---|---|
| `lat,lon` | Yeni konum |
| `lat,lon,yükseklik_m` | Konum + irtifa |
| `lat,lon,yükseklik_m,yön_derece` | Konum + irtifa + gerçek yön (true heading) |
| `?` | Mevcut durumu yazdır |
| `m` | NMEA aynası — FC'ye giden cümleleri ekranda göster |
| `t` | Loopback testi — `PA9`'un fiziksel olarak veri bastığını kanıtlar |
| `b<baud>` | NMEA çıkış baud'unu değiştir (örn. `b115200`) |
| `h` | Komut listesi |

**Örnekler:**

```
41.008200,28.978400              # İstanbul, irtifa ve yön korunur
39.925000,32.836900,900,0        # Anıtkabir, 900 m, kuzeye bakıyor
39.920800,32.854100,890,45       # Kızılay, 890 m, kuzeydoğuya bakıyor
```

Yön alanı `0`=Kuzey, `90`=Doğu, `180`=Güney, `270`=Batı.

Girdi katı şekilde doğrulanır: alanlardan biri bile geçerli bir sayı değilse **komut tümden reddedilir** ve mevcut konum korunur. Bu, seri hatta binen gürültünün konumu sessizce bozmasını engeller.

---

## Üretilen NMEA cümleleri

Her turda üçü birlikte, 5 Hz hızında gönderilir:

| Cümle | Taşıdığı bilgi |
|---|---|
| `$GPGGA` | Konum, irtifa (MSL), fix kalitesi, uydu sayısı, HDOP |
| `$GPRMC` | Konum, UTC zaman, tarih, hız, rota, geçerlilik bayrağı |
| `$GPHDT` | Gerçek yön (true heading) |

`GGA` ve `RMC` aynı döngüde arka arkaya yazılır; ArduPilot'un NMEA sürücüsü bu ikisini 150 ms içinde birlikte görmek ister.

Örnek çıktı:

```
$GPGGA,120227.00,4100.4920,N,02858.7040,E,1,12,0.8,65.0,M,0.0,M,,*6E
$GPRMC,120227.00,A,4100.4920,N,02858.7040,E,0.0,0.0,010126,,,A*50
$GPHDT,0.00,T*05
```

---

## ArduPilot ayarları

| Parametre | Değer | Not |
|---|---|---|
| `GPS_TYPE` | `5` | NMEA. ArduPilot 4.6+ sürümlerde adı `GPS1_TYPE` |
| `SERIALx_PROTOCOL` | `5` | GPS. `x` = **fiziksel olarak bağlandığın port** |
| `SERIALx_BAUD` | `38` | 38400 anlamına gelir |
| `SERIALx_OPTIONS` | `0` | Sıfır değilse hatta invert/swap uygulanıyor olabilir |

Değişiklikten sonra **FC'yi reboot et** — `SERIALx_PROTOCOL` reboot olmadan uygulanmaz.

### Doğru `SERIALx`'i bulmak

Kart üzerindeki `RX1`, `RX4` gibi silkscreen etiketleri **donanım UART numarasını** gösterir; ArduPilot'un `SERIAL1`, `SERIAL4` numaralandırmasıyla birebir aynı olmak zorunda değildir. Eşlemeyi kartının ArduPilot dokümantasyon sayfasındaki "UART Mapping" tablosundan doğrula.

Doğru portu bulduğunun kesin kanıtı, reboot sonrası Mission Planner'ın **Messages** sekmesinde beliren şu satırdır:

```
GPS 1: detected as NMEA
```

### GPS örneği (instance) tuzağı

ArduPilot, protokolü GPS (`5`) olan **ilk** portu GPS1, **ikinci** portu GPS2 sayar. Çoğu kartta `SERIAL3` fabrika ayarı olarak zaten GPS protokolündedir. Cihazını başka bir porta taktıysan GPS2 slotuna düşebilir; bu durumda `GPS_TYPE` değil **`GPS_TYPE2`** parametresini ayarlaman gerekir.

Bu senaryoda HUD'daki "GPS:" göstergesi GPS1'in durumunu gösterdiği için, cihazın düzgün çalışsa bile ekranda "No GPS" yazmaya devam edebilir. En temiz yol, kabloyu kartın **GPS1** konnektörüne takmaktır.

---

## Doğrulama

Sorunun hangi katmanda olduğunu ayırmak için iki yerleşik test var:

### `m` — NMEA aynası

FC'ye giden cümlelerin birebir aynısını ekrana basar. Cümlelerin **doğru kurulduğunu** (alanlar, checksum) kanıtlar.

### `t` — Loopback testi

`PA9` ile `PA10` arasına kısa bir jumper tak, sonra `t` yaz. `PA9`'dan fiziksel olarak çıkan byte'lar USART1 RX'ten geri okunup ekrana basılır.

| Sonuç | Anlamı |
|---|---|
| `$GP...` akıyor | `PA9` gerçekten veri basıyor → sorun kabloda veya FC parametresinde |
| Hiçbir şey gelmiyor | USART1 açılmamış, byte'lar çöpe gidiyor → kart/pin tanımı sorunu |

> Aynanın çıktı vermesi tek başına `PA9`'un sinyal ürettiğini kanıtlamaz — USART1 hiç açılmamışsa `write()` sessizce çöpe yazar ve ekranda yine aynı çıktıyı görürsün. İkisini ayıran test budur.

---

## Sorun giderme

| Belirti | Olası sebep | Çözüm |
|---|---|---|
| FC'de "No GPS" | Yanlış `SERIALx` ayarlandı | Silkscreen etiketi ≠ ArduPilot SERIAL numarası; kart dokümanındaki UART eşlemesine bak |
| FC'de "No GPS" | Cihaz GPS2 slotunda | `GPS_TYPE2 = 5` ayarla, ya da kabloyu GPS1 portuna taşı |
| FC'de "No GPS" | Reboot atılmadı | `SERIALx_PROTOCOL` reboot olmadan uygulanmaz |
| FC'de "No GPS" | Ortak GND yok | STM32 GND ile FC GND'yi birleştir |
| FC'de "No GPS" | TX/RX ters | `PA9` mutlaka FC'nin **RX** pinine gitmeli |
| FC'de "No GPS" | `SERIALx_OPTIONS` sıfır değil | Invert/swap bitleri NMEA'yı okunamaz hale getirir, `0` yap |
| Komutlar işlenmiyor | Satır sonu ayarı yanlış | Serial Monitor'de **Newline** seç |
| Loopback'te çıktı yok | USART1 açılmamış | Kartının `PA9`/`PA10` pinlerinin USART1'e ait olduğunu doğrula |
| "EKF variance" / GPS glitch | Konum çok uzağa atlatıldı | 100–200 m'lik adımlarla ilerle, fix'in oturmasını bekle |

---

## Yapılandırma

Kodun başındaki `#define`'lardan ayarlanır:

| Tanım | Varsayılan | Açıklama |
|---|---|---|
| `BASLANGIC_LAT_1E7` | `410082000` | Açılış enlemi (derece × 10⁷) |
| `BASLANGIC_LON_1E7` | `289784000` | Açılış boylamı (derece × 10⁷) |
| `BASLANGIC_ALT_M` | `65` | Açılış irtifası (m, MSL) |
| `BASLANGIC_HEADING_DEG` | `0` | Açılış yönü (derece) |
| `NMEA_PERIOD_MS` | `200` | Yayın periyodu (5 Hz) |
| `GPS_BAUD_VARSAYILAN` | `38400` | NMEA çıkış baud'u |
| `NMEA_FIX_KALITE` | `1` | GGA fix: 1=GPS, 2=DGPS, 4=RTK-fixed |
| `NMEA_UYDU_SAYISI` | `12` | GGA uydu sayısı |
| `NMEA_HDOP` | `"0.8"` | HDOP değeri |
| `NMEA_TARIH` | `"010126"` | RMC tarihi (ggaayy) |

---

## Notlar

- Tüm koordinat matematiği **tamsayı** aritmetiğiyle yapılır (derece × 10⁷); float kullanılmaz.
- Seri okuma bloklamayan (non-blocking) şekilde yapılır, NMEA yayınının periyodu komut girişinden etkilenmez.
- `$GPHDT` ile gönderilen yön bilgisinin EKF tarafından kullanılması için FC'de `EK3_SRC1_YAW = 2` (GPS) ayarlanmalıdır.
