/* ===========================================================================
   SAHTE (SIMULE) GPS  ->  UCUS KONTROLCUSU (FC)  /  NMEA - UART

   AKIS:
     PC (Serial Monitor) --DBG(PA2/PA3)--> STM32 --GPS_OUT(PA9)--> FC RX
             duz metin komut                        NMEA cumleleri

   KABLO:  PA9 (STM TX) -> FC'nin GPS portundaki RX
           GND          -> GND        (ORTAK GND SART)

   FC (ArduPilot) PARAMETRELERI:
           GPS_TYPE          = 5   (NMEA)
           SERIALx_PROTOCOL  = 5   (GPS)     x = fiziksel bagli oldugun port
           SERIALx_BAUD      = 38  (38400)
           -> degisiklikten sonra FC'yi REBOOT et.
   NOT: ArduPilot GPS portunda baud'u kendisi tarar (4800..230400), bu yuzden
        SERIALx_BAUD tam eslesmese de bulabilir. Yine de esitlemek en saglami.
   =========================================================================== */

#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* ---------------- BASLANGIC KONUMU (acilista yayinlanan sabit nokta) ------ */
#define BASLANGIC_LAT_1E7      410082000L  // 41.0082 N
#define BASLANGIC_LON_1E7      289784000L  // 28.9784 E
#define BASLANGIC_ALT_M        65L         // deniz seviyesi uzeri 65 m
#define BASLANGIC_HEADING_DEG  0           // 0=Kuzey, 90=Dogu, 180=Guney, 270=Bati

/* ---------------- NMEA YAYIN AYARLARI ------------------------------------- */
#define NMEA_PERIOD_MS         200         // 5 Hz  (GGA+RMC+HDT birlikte)
#define GPS_BAUD_VARSAYILAN    38400UL     // FC'deki SERIALx_BAUD ile ayni olmali
#define NMEA_FIX_KALITE        1           // GGA fix: 1=GPS, 2=DGPS, 4=RTK-fixed
#define NMEA_UYDU_SAYISI       12          // GGA uydu sayisi (FC'nin gordugu deger)
#define NMEA_HDOP              "0.8"       // GGA/RMC HDOP
#define NMEA_TARIH             "010126"    // RMC tarihi: ggaayy (01/01/2026)
#define NMEA_SAAT_OFSET_SN     43200UL     // saat 12:00:00'dan baslasin

/* ---------------- SERI PORTLAR -------------------------------------------- */
// DBG: PC'ye giden port. Komutlari BURADAN yazarsin, durumu BURADAN okursun.
//      (Nucleo kartlarda PA2/PA3 genelde ST-Link USB sanal COM portudur.)
HardwareSerial DBG(PA3, PA2);       // USART2  RX=PA3, TX=PA2

// GPS_OUT: NMEA cumlelerinin CIKTIGI port. PA9 -> FC RX.
HardwareSerial GPS_OUT(PA10, PA9);  // USART1  RX=PA10, TX=PA9

/* ---------------- DURUM DEGISKENLERI -------------------------------------- */
static int32_t  g_lat_1e7     = BASLANGIC_LAT_1E7;
static int32_t  g_lon_1e7     = BASLANGIC_LON_1E7;
static int32_t  g_alt_m       = BASLANGIC_ALT_M;
static int32_t  g_heading_deg = BASLANGIC_HEADING_DEG;

static uint32_t g_gps_baud    = GPS_BAUD_VARSAYILAN;
static bool     g_nmea_ayna   = false;   // true: uretilen NMEA'yi DBG'ye de bas
static bool     g_loopback    = false;   // true: GPS_OUT'un RX'ine geleni DBG'ye bas


/* ===========================================================================
   METIN -> SAYI  (float kullanmadan)
   =========================================================================== */

// ONEMLI: Bu iki fonksiyon metni SIKI dogrular. Tek bir gecersiz karakter bile
// olsa false doner ve komut tumden reddedilir. Boylece DBG hattina binen
// gurultu (jumper takarken olusan parazit gibi) konumu sessizce BOZAMAZ.

// "-123" -> -123.  Bastan sona sayi degilse false.
static bool metinInt(const char *s, int32_t *cikis) {
    if (!s || !*s) return false;
    bool neg = false;
    if (*s == '-') { neg = true; s++; } else if (*s == '+') s++;
    if (!(*s >= '0' && *s <= '9')) return false;

    int32_t v = 0;
    uint8_t hane = 0;
    while (*s >= '0' && *s <= '9') {
        if (++hane > 9) return false;              // tasma korumasi
        v = v * 10 + (*s++ - '0');
    }
    if (*s != 0) return false;                     // artik karakter -> gecersiz
    *cikis = neg ? -v : v;
    return true;
}

// "41.008200" -> 410082000  (derece * 1e7, float KULLANMADAN)
static bool metinDerece1e7(const char *s, int32_t *cikis) {
    if (!s || !*s) return false;
    bool neg = false;
    if (*s == '-') { neg = true; s++; } else if (*s == '+') s++;
    if (!(*s >= '0' && *s <= '9')) return false;

    int32_t tam = 0;
    uint8_t hane = 0;
    while (*s >= '0' && *s <= '9') {
        if (++hane > 3) return false;              // en fazla 180 -> 3 hane
        tam = tam * 10 + (*s++ - '0');
    }
    if (tam > 180) return false;                   // tam * 1e7 tasmasin

    int32_t kesir = 0;
    if (*s == '.') {
        s++;
        if (!(*s >= '0' && *s <= '9')) return false;   // "41." gibi yarim sayi
        for (uint8_t i = 0; i < 7; i++) {
            kesir *= 10;
            if (*s >= '0' && *s <= '9') kesir += (*s++ - '0');
        }
        while (*s >= '0' && *s <= '9') s++;         // 7 haneden fazlasini yut
    }
    if (*s != 0) return false;                     // artik karakter -> gecersiz

    int32_t v = tam * 10000000L + kesir;
    *cikis = neg ? -v : v;
    return true;
}


/* ===========================================================================
   NMEA URETIMI
   =========================================================================== */

// Govdeyi ($ ve *CRC olmadan) alir, checksum'i hesaplar, tam cumleyi yollar.
// Ayna acikken ayni cumle DBG portuna da yazilir (dogrulama icin).
static void nmeaGonder(const char *govde) {
    static const char ONALTILIK[] = "0123456789ABCDEF";

    uint8_t crc = 0;
    for (const char *p = govde; *p; p++) crc ^= (uint8_t)(*p);

    char cumle[128];
    int n = snprintf(cumle, sizeof(cumle), "$%s*%c%c\r\n",
                     govde, ONALTILIK[crc >> 4], ONALTILIK[crc & 0x0F]);
    if (n <= 0) return;
    if (n > (int)sizeof(cumle) - 1) n = (int)sizeof(cumle) - 1;

    GPS_OUT.write((const uint8_t *)cumle, (size_t)n);        // <-- FC'ye giden
    if (g_nmea_ayna) DBG.write((const uint8_t *)cumle, (size_t)n);
}

// derece*1e7 -> NMEA'nin "ddmm.mmmm" / "dddmm.mmmm" formati + yon harfi
static void dereceyiNmeaYaz(char *cikis, uint8_t derece_hane,
                             int32_t v_1e7, char pozitifHarf, char negatifHarf,
                             char &harf) {
    harf = (v_1e7 < 0) ? negatifHarf : pozitifHarf;
    int32_t mutlak = (v_1e7 < 0) ? -v_1e7 : v_1e7;

    int32_t derece     = mutlak / 10000000L;
    int32_t kalan      = mutlak % 10000000L;
    int32_t dakika_1e4 = (kalan * 6L) / 100L;      // 0 .. 599999

    if (derece_hane == 3) {
        sprintf(cikis, "%03ld%02ld.%04ld",
                (long)derece, (long)(dakika_1e4 / 10000L), (long)(dakika_1e4 % 10000L));
    } else {
        sprintf(cikis, "%02ld%02ld.%04ld",
                (long)derece, (long)(dakika_1e4 / 10000L), (long)(dakika_1e4 % 10000L));
    }
}

// Bir turda GGA + RMC + HDT birlikte gonderilir.
// ArduPilot'un NMEA suruculugu GGA ile RMC'yi 150 ms icinde birlikte gormek
// ister; bu yuzden ucu de ayni dongude, arka arkaya yaziliyor.
static void nmeaHepsiniGonder() {
    uint32_t toplam_sn = (millis() / 1000UL) + NMEA_SAAT_OFSET_SN;
    int hh = (int)((toplam_sn / 3600UL) % 24UL);
    int mm = (int)((toplam_sn / 60UL) % 60UL);
    int ss = (int)(toplam_sn % 60UL);

    char lat_s[16], lon_s[16];
    char lat_h, lon_h;
    dereceyiNmeaYaz(lat_s, 2, g_lat_1e7, 'N', 'S', lat_h);
    dereceyiNmeaYaz(lon_s, 3, g_lon_1e7, 'E', 'W', lon_h);

    char govde[120];

    // --- $GPGGA : konum + yukseklik + fix kalitesi + uydu + HDOP ---
    //     alanlar: zaman, enlem, N/S, boylam, E/W, fix, uydu, HDOP,
    //              yukseklik, M, geoid ayrimi, M, DGPS yasi, istasyon
    snprintf(govde, sizeof(govde),
             "GPGGA,%02d%02d%02d.00,%s,%c,%s,%c,%d,%02d," NMEA_HDOP ",%ld.0,M,0.0,M,,",
             hh, mm, ss, lat_s, lat_h, lon_s, lon_h,
             NMEA_FIX_KALITE, NMEA_UYDU_SAYISI, (long)g_alt_m);
    nmeaGonder(govde);

    // --- $GPRMC : konum + zaman + tarih + hiz/rota ---
    //     A = veri gecerli, hiz 0.0 kt, rota 0.0 (sabit duruyoruz),
    //     manyetik sapma bos, sondaki A = otonom mod
    snprintf(govde, sizeof(govde),
             "GPRMC,%02d%02d%02d.00,A,%s,%c,%s,%c,0.0,0.0," NMEA_TARIH ",,,A",
             hh, mm, ss, lat_s, lat_h, lon_s, lon_h);
    nmeaGonder(govde);

    // --- $GPHDT : GERCEK YON (true heading) ---
    //     Yon bilgisini FC'ye tasiyan TEK cumle bu. T = true (gercek kuzeye
    //     gore, manyetik degil). EKF'in bunu kullanmasi icin FC'de
    //     EK3_SRC1_YAW = 2 (GPS) olmali.
    int32_t h = ((g_heading_deg % 360) + 360) % 360;
    snprintf(govde, sizeof(govde), "GPHDT,%ld.00,T", (long)h);
    nmeaGonder(govde);
}


/* ===========================================================================
   SERI KOMUTLAR (DBG portundan)
   =========================================================================== */

static void yardimYaz() {
    DBG.println(F("--- KOMUTLAR ---"));
    DBG.println(F("  lat,lon[,yukseklik_m[,yon_derece]]   yeni sabit konum"));
    DBG.println(F("      ornek: 41.008200,28.978400,150,90"));
    DBG.println(F("  ?      mevcut durumu yazdir"));
    DBG.println(F("  m      NMEA aynasini ac/kapat (uretilen cumleleri burada gor)"));
    DBG.println(F("  t      LOOPBACK testi: PA9'un GERCEKTEN sinyal urettigini kanitlar"));
    DBG.println(F("         (once PA9 ile PA10'u kisa bir jumper ile birlestir)"));
    DBG.println(F("  b<baud> GPS portunun baud'unu degistir, ornek: b115200"));
    DBG.println(F("  h      bu yardim"));
    DBG.println(F("NOT: Serial Monitor'de satir sonu 'Newline' secili olmali."));
}

static void durumYaz() {
    DBG.print(F("[DURUM] lat="));   DBG.print(g_lat_1e7 / 1e7, 7);
    DBG.print(F(" lon="));          DBG.print(g_lon_1e7 / 1e7, 7);
    DBG.print(F(" yukseklik="));    DBG.print(g_alt_m);
    DBG.print(F("m yon="));         DBG.print(g_heading_deg);
    DBG.print(F("derece  gps_baud=")); DBG.print(g_gps_baud);
    DBG.print(F(" ayna="));         DBG.println(g_nmea_ayna ? F("ACIK") : F("KAPALI"));
}

static void baudDegistir(uint32_t yeni) {
    static const uint32_t GECERLI[] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400 };
    for (uint8_t i = 0; i < sizeof(GECERLI) / sizeof(GECERLI[0]); i++) {
        if (GECERLI[i] == yeni) {
            GPS_OUT.end();
            GPS_OUT.begin(yeni);
            g_gps_baud = yeni;
            DBG.print(F("[BAUD] GPS portu -> ")); DBG.println(yeni);
            DBG.println(F("       FC'deki SERIALx_BAUD ile eslestirmeyi unutma."));
            return;
        }
    }
    DBG.println(F("[HATA] gecerli baud: 4800 9600 19200 38400 57600 115200 230400"));
}

//  Satir bicimleri:
//    41.008200,28.978400
//    41.008200,28.978400,150
//    41.008200,28.978400,150,90
//    ?  /  m  /  h  /  b115200
static void komutIsle(char *satir) {
    for (char *p = satir; *p; p++) {
        if (*p == '\r' || *p == '\n') { *p = 0; break; }
    }
    if (satir[0] == 0) return;   // bos satir, yoksay

    // --- tek harfli komutlar ---
    if (satir[0] == '?' && satir[1] == 0) { durumYaz();  return; }
    if ((satir[0] == 'h' || satir[0] == 'H') && satir[1] == 0) { yardimYaz(); return; }

    if ((satir[0] == 'm' || satir[0] == 'M') && satir[1] == 0) {
        g_nmea_ayna = !g_nmea_ayna;
        DBG.print(F("[AYNA] "));
        DBG.println(g_nmea_ayna ? F("ACIK  - asagidaki cumleler FC'ye giden verinin AYNISI")
                                : F("KAPALI"));
        return;
    }

    if ((satir[0] == 't' || satir[0] == 'T') && satir[1] == 0) {
        g_loopback = !g_loopback;
        if (g_loopback) {
            DBG.println(F("[LOOPBACK] ACIK. PA9 <-> PA10 arasina jumper tak."));
            DBG.println(F("  Asagida $GP... goruyorsan: PA9 GERCEKTEN veri BASIYOR,"));
            DBG.println(F("  sorun kabloda veya FC parametresindedir."));
            DBG.println(F("  Hicbir sey gelmiyorsa: USART1 acilmamis, sorun KODDA/KARTTA."));
        } else {
            DBG.println(F("[LOOPBACK] KAPALI"));
        }
        return;
    }

    if (satir[0] == 'b' || satir[0] == 'B') {
        int32_t b = 0;
        if (!metinInt(satir + 1, &b) || b <= 0) {
            DBG.println(F("[HATA] ornek: b115200"));
            return;
        }
        baudDegistir((uint32_t)b);
        return;
    }

    // --- konum komutu: virgulle ayrilmis alanlar ---
    char *alan[4] = { nullptr, nullptr, nullptr, nullptr };
    uint8_t n = 0;
    alan[n++] = satir;
    for (char *p = satir; *p && n < 4; p++) {
        if (*p == ',') { *p = 0; alan[n++] = p + 1; }
    }

    if (n < 2 || alan[0][0] == 0 || alan[1][0] == 0) {
        DBG.println(F("[HATA] format: lat,lon[,yukseklik_m[,yon_derece]]  ornek: 41.008200,28.978400"));
        DBG.println(F("       komut listesi icin h yaz."));
        return;
    }

    // TUM alanlari once dogrula, hicbirini yazma. Tek bir alan bile bozuksa
    // komut tumden reddedilir; mevcut konum oldugu gibi kalir.
    int32_t yeni_lat = 0, yeni_lon = 0;
    int32_t yeni_alt = g_alt_m, yeni_hdg = g_heading_deg;

    if (!metinDerece1e7(alan[0], &yeni_lat) ||
        !metinDerece1e7(alan[1], &yeni_lon)) {
        DBG.println(F("[HATA] enlem/boylam sayi degil. ornek: 41.008200,28.978400"));
        return;
    }
    if (yeni_lat < -900000000L || yeni_lat > 900000000L ||
        yeni_lon < -1800000000L || yeni_lon > 1800000000L) {
        DBG.println(F("[HATA] enlem [-90,90], boylam [-180,180] araliginda olmali"));
        return;
    }
    if (n >= 3 && alan[2][0] != 0 && !metinInt(alan[2], &yeni_alt)) {
        DBG.println(F("[HATA] yukseklik sayi degil (metre, tamsayi)"));
        return;
    }
    if (n >= 4 && alan[3][0] != 0 && !metinInt(alan[3], &yeni_hdg)) {
        DBG.println(F("[HATA] yon sayi degil (derece, tamsayi)"));
        return;
    }

    // Hepsi gecerli -> simdi yaz.
    g_lat_1e7     = yeni_lat;
    g_lon_1e7     = yeni_lon;
    g_alt_m       = yeni_alt;
    g_heading_deg = yeni_hdg;

    DBG.print(F("[YENI KONUM] lat=")); DBG.print(g_lat_1e7 / 1e7, 7);
    DBG.print(F(" lon="));             DBG.print(g_lon_1e7 / 1e7, 7);
    DBG.print(F(" alt="));             DBG.print(g_alt_m);
    DBG.print(F("m yon="));            DBG.print(g_heading_deg);
    DBG.println(F("  -> bir sonraki NMEA ile UART'a gidiyor"));
}

// Non-blocking seri okuma: NMEA yayinini BLOKE ETMEDEN karakter biriktirir.
static void seriKomutOku() {
    static char tampon[48];
    static uint8_t idx = 0;

    while (DBG.available()) {
        char c = (char)DBG.read();
        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                tampon[idx] = 0;
                komutIsle(tampon);
                idx = 0;
            }
        } else if (idx < sizeof(tampon) - 1) {
            tampon[idx++] = c;
        } else {
            idx = 0;   // asiri uzun satir -> cop, bastan basla
        }
    }
}


/* ===========================================================================
   SETUP / LOOP
   =========================================================================== */

void setup() {
    DBG.begin(115200);
    GPS_OUT.begin(g_gps_baud);   // NMEA CIKISI - FC'ye giden port

    delay(200);
    DBG.println();
    DBG.println(F("=== SERI KOMUTLA SAHTE GPS -> FC / NMEA over UART ==="));
    DBG.print  (F("NMEA cikisi: PA9 (TX) -> FC RX,  GND -> GND,  baud="));
    DBG.println(g_gps_baud);
    DBG.println(F("FC: GPS_TYPE=5, SERIALx_PROTOCOL=5, SERIALx_BAUD=38, sonra REBOOT"));
    yardimYaz();
    durumYaz();
}

void loop() {
    static uint32_t son_nmea  = 0;
    static uint32_t son_debug = 0;
    static uint32_t gonderim  = 0;

    uint32_t simdi = millis();

    seriKomutOku();     // her donguide kontrol et, hicbir seyi bloke etmez

    // 1. Guncel konum + yonu NMEA olarak yayinla (5 Hz)
    if (simdi - son_nmea >= NMEA_PERIOD_MS) {
        son_nmea = simdi;
        nmeaHepsiniGonder();
        gonderim++;
    }

    // 2. Loopback testi: PA9 -> PA10 jumper takiliysa, PA9'dan FIZIKSEL olarak
    //    cikan byte'lar USART1 RX'ten geri okunur. Ekranda $GP... goruluyorsa
    //    pin gercekten suruyor demektir; ayna ciktisindan farki budur.
    if (g_loopback) {
        while (GPS_OUT.available()) DBG.write((uint8_t)GPS_OUT.read());
    }

    // 3. 1 Hz durum ozeti. Ayna/loopback acikken bastirilir, yoksa akisi kirletir.
    if (!g_nmea_ayna && !g_loopback && simdi - son_debug >= 1000) {
        son_debug = simdi;
        DBG.print(F("[NMEA] paket="));  DBG.print(gonderim);
        DBG.print(F(" lat="));          DBG.print(g_lat_1e7 / 1e7, 6);
        DBG.print(F(" lon="));          DBG.print(g_lon_1e7 / 1e7, 6);
        DBG.print(F(" alt="));          DBG.print(g_alt_m);
        DBG.print(F("m yon="));         DBG.print(g_heading_deg);
        DBG.println();
    }
}
