# Hexnet IO Module v0-50 — Platform Geliştirme Prompt'u

Bu belge, **Hexnet IO Module** (aktif geliştirme hattı **IOModule-v0-50**, firmware **v0.50**, ESP32-S3) için platform/backend/mobil ekip veya AI geliştiricisine verilecek **tek referans prompt**'tur. Cihaz firmware'i ile platform arasındaki sözleşmeyi, mevcut cihaz yeteneklerini ve implement edilmesi gereken platform işlerini tanımlar.

---

## 1. Görev tanımı

Hexnet IO Module sahadaki endüstriyel I/O cihazıdır (16 röle, 4 dimmer, RGB matris, ADC, DHT, motor, CAN, RS485, BLE). Cihaz tarafında **MQTT telemetri**, **komut kontrolü** ve **OTA (uzaktan firmware güncelleme)** altyapısı kurulmuştur.

**Platform ekibinden beklenen:**

1. Cihaz kaydı ve MQTT topic çözümleme (`resolve-mqtt` API)
2. Standart telemetry JSON'unu almak, saklamak ve göstermek
3. Komut gönderimi (röle, dimmer, RGB, motor, OTA)
4. Firmware artefakt yönetimi ve `ota_start` ile uzaktan güncelleme
5. OTA ilerlemesini telemetry `ota` bloğu ile izleme

Cihazda otomatik sürüm kontrolü, SHA256 doğrulama ve BLE OTA **yoktur** — bunlar platform veya ileri faz firmware işidir.

---

## 2. Cihaz kimliği ve sürüm

| Alan | Değer |
|------|--------|
| Geliştirme hattı (repo) | `IOModule-v0-50` (`HEXNET_IO_DEV_LINE`) |
| Firmware / katalog | `v0.50` (`HEXNET_IO_RELEASE`) |
| CMake proje sürümü | `0.50.0` → telemetri `device.fw` via `hexnet_firmware_version_string()` |
| Donanım tipi (resolve) | `Vango-Medium` |
| MCU | ESP32-S3 |
| Flash | 8 MB — dual OTA (`ota_0`, `ota_1` ~2,9 MB), `factory` partition |
| MAC kimliği | **WiFi STA MAC** (`AA:BB:CC:DD:EE:FF`) — BLE adresi kullanılmaz |

---

## 3. Firmware mimarisi (cihaz — referans)

Platformun bilmesi gereken modül yapısı:

```
app_main.c          → GPIO, OTA init, NVS, WiFi/MQTT init, bringup task
hexnet_wifi.c       → STA, internet probe, SNTP, captive portal tetikleme
hexnet_resolve_mqtt → POST resolve → topic routing (NVS cache)
hexnet_mqtt.c       → Telemetry publish, command subscribe, ota_start
hexnet_app.c        → CAN, shift register, RGB, ADC, DHT, motor, RS485, BLE (gecikmeli)
UserCodes/Ota/      → OtaManager — HTTPS/HTTP OTA indirme ve flash yazma
```

**Başlatma sırası:** board GPIO → OTA init → NVS → WiFi/MQTT init → (3–20 sn) → I/O task'ları → MQTT telemetry → BLE (MQTT telemetri stabil olduktan ~45 sn sonra).

---

## 4. Bağlantı altyapısı

### 4.1 MQTT broker (varsayılan)

- URI: `mqtt://185.33.234.10:1883`
- Fallback topic öneki: `hexnet/v1/{company_id}/{device_id}/`
  - Telemetry: `.../telemetry`
  - Command: `.../command`

### 4.2 Resolve API (kayıt ve topic atama)

Cihaz internete çıkınca periyodik çağırır (başarısızsa NVS cache ile devam edebilir).

| Özellik | Değer |
|---------|--------|
| Method | `POST` |
| URL | `{BASE}/api/device/resolve-mqtt.php` |
| Varsayılan BASE | `http://185.33.234.10/myhexnet` |

**İstek gövdesi:**

```json
{
  "mac": "AA:BB:CC:DD:EE:FF",
  "fw": "0.40.0",
  "hw": "Vango-Medium"
}
```

**Yanıtta zorunlu alanlar (routing tam sayılması için):**

- `ok` (truthy)
- `company_id`, `device_pk_id` veya `mqtt_segment`
- `telemetry_topic` — cihazın publish ettiği topic
- `command_topic` — platformun publish ettiği topic (cihaz subscribe)

İsteğe bağlı: `device_uid`, `status_topic`, `command_reply_topic`, `state` (`registered` vb.)

Resolve tamamlanmadan cihaz MQTT'ye tam bağlanmayabilir. Platform her cihazı MAC + hw ile kayıt edip **kalıcı topic** döndürmelidir.

### 4.3 NTP ve timestamp

- Cihaz WiFi sonrası SNTP başlatır (`pool.ntp.org`, `time.google.com`)
- Telemetry `ts`: Unix saniye (int)
- NTP senkron değilse: `ts: 0` (telemetry yine gönderilebilir)
- Platform `ts: 0` paketlerini “saat ayarsız” olarak işaretleyebilir

---

## 5. Telemetry sözleşmesi (cihaz → platform)

**Tek standart format.** Eski alanlar kullanılmamalı.

### 5.1 Tam şema

```json
{
  "protocol": "hexnet-io",
  "version": "1.0",
  "ts": 1715788800,

  "device": {
    "id": 1,
    "mac": "AA:BB:CC:DD:EE:FF",
    "company_id": 1,
    "customer_id": 0,
    "fw": "0.40.0"
  },

  "status": {
    "online": true,
    "wifi_rssi": -62,
    "mqtt": "connected"
  },

  "power": {
    "vin_v": 12.4,
    "battery_v": 12.1,
    "battery_pct": 85
  },

  "sensors": {
    "temperature_c": 24.0,
    "humidity_pct": 55.0,
    "water_pct": [0, 0, 0, 0]
  },

  "outputs": {
    "relays": [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    "relays_mask": 2,
    "dimmers": [0, 50, 0, 0],
    "rgb": { "r": 255, "g": 0, "b": 0 }
  },

  "ota": {
    "state": "idle",
    "progress": 0,
    "target_version": "",
    "last_error": ""
  }
}
```

### 5.2 Alan kuralları

| Kural | Açıklama |
|-------|----------|
| Voltajlar | `*_v` (örn. `vin_v`, `battery_v`) |
| Yüzdeler | `*_pct` |
| Sıcaklık | `temperature_c` |
| Nem | `humidity_pct` |
| Röleler | `relays` (16 eleman, 0/1) + `relays_mask` (uint16) |
| Dimmer | 0–100, 4 kanal |
| RGB | object `{r,g,b}` — array değil |
| `ts` | int, Unix saniye |

### 5.3 Kaldırılmış / kullanılmayan alanlar (MQTT telemetry)

Bunları parse etmeyin veya üretmeyin:

- `volt`, `battery`, `batteryVolt`, `battery_voltage`
- Kök seviye `outputs_mask` (yerine `outputs.relays` + `outputs.relays_mask`)
- RGB array formatı

### 5.4 Yayın sıklığı

- Heartbeat: ~5 saniye
- Durum değişiminde: min ~250 ms aralıkla ek publish
- Offline queue: bağlantı kopunca birikir, reconnect sonrası flush

---

## 6. Komut sözleşmesi (platform → cihaz)

**Topic:** resolve sonrası `command_topic`  
**QoS:** 1 (cihaz subscribe)

Genel format:

```json
{
  "cmd": "<komut_adı>",
  "args": { }
}
```

### 6.1 Röle

**Tek röle:**

```json
{
  "cmd": "relay_set",
  "relay": 3,
  "state": 1
}
```

**Tüm maske:**

```json
{
  "cmd": "relay_set",
  "args": { "mask": 65535 }
}
```

**Bit:**

```json
{
  "cmd": "relay_bit",
  "args": { "index": 3, "on": true }
}
```

### 6.2 Dimmer

```json
{
  "cmd": "dim_set",
  "args": { "index": 0, "value": 75 }
}
```

(`dimmer_set` alias kabul edilir; `index` / `channel` 0–3, `value` 0–100)

### 6.3 RGB

```json
{
  "cmd": "rgb_set",
  "args": { "r": 255, "g": 128, "b": 0, "enable": 1 }
}
```

### 6.4 Motor

```json
{
  "cmd": "motor_set",
  "args": { "value": 1 }
}
```

`value`: `0` = stop, `1` = forward, `2` = backward

### 6.5 OTA (uzaktan firmware güncelleme)

```json
{
  "cmd": "ota_start",
  "args": {
    "url": "https://cdn.example.com/firmware/io-module-0.50.0.bin",
    "version": "0.50.0",
    "force": false
  }
}
```

| Alan | Zorunlu | Açıklama |
|------|---------|----------|
| `url` | Evet | `.bin` firmware URL. Cihazda HTTP de açık (`CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP`); üretimde HTTPS kullanın. |
| `version` | Hayır | Hedef sürüm. Mevcut `device.fw` ile aynı ve `force: false` → red. |
| `force` | Hayır | `true` → aynı sürümde bile güncelle. |
| `sha256` | Hayır | Cihaz **henüz doğrulamıyor** (ileri faz). |

**Red nedenleri:** boş URL, OTA zaten devam ediyor, aynı sürüm (`force` false).

### 6.6 Eski BLE formatı (geriye dönük)

Kökte `writeDataType` varsa payload BLE mobil protokolü olarak işlenir; platform yeni entegrasyonlarda `cmd` formatını kullanmalıdır.

---

## 7. OTA — cihaz tarafı durumu

### 7.1 Tamamlanan (firmware)

- Dual OTA partition + `otadata`
- Bootloader app rollback (`PENDING_VERIFY` → boot sonrası doğrulama)
- `OtaManager`: URL'den indir → flash yaz → reboot
- MQTT `ota_start` komutu
- Telemetry'de `ota` bloğu (state, progress, target_version, last_error)
- `app_main` içinde erken `ota_manager_init()`

### 7.2 OTA state değerleri

| state | Anlam |
|--------|--------|
| `idle` | Güncelleme yok |
| `requested` | Komut kabul edildi |
| `downloading` | İndiriliyor |
| `writing` | Flash yazılıyor |
| `verifying` | Doğrulama |
| `set_boot` | Boot partition |
| `success` | Başarılı (reboot öncesi) |
| `rebooting` | Yeniden başlatılıyor |
| `failed` | Hata — `last_error` okuyun |

`progress`: 0–100

### 7.3 Cihazda OLMAYAN (platform sağlamalı)

- Otomatik “yeni sürüm var mı?” API çağrısı
- Resolve yanıtında firmware URL
- SHA256 / imza doğrulama
- WiFi portal veya BLE üzerinden OTA
- `ota_start` için ayrı ACK topic (izleme = telemetry)

### 7.4 Önerilen platform OTA iş akışı

```
1. Admin panel: cihaz seç (MAC / device_uid)
2. Firmware sürümü seç (URL + version kayıtlı)
3. Ön kontrol: online, mqtt connected, ota.state == idle
4. MQTT publish → ota_start
5. Telemetry izle: progress, state, last_error
6. Reboot sonrası device.fw == target_version → başarı
7. Timeout (örn. 15 dk) ve failed alarm
```

### 7.5 Önerilen platform veri modeli

**firmware_releases:** id, version, hw (`Vango-Medium`), release_line (`v0-50`), file_url, sha256, size, is_active, created_at

**ota_jobs:** id, device_id, firmware_id, state (pending|sent|downloading|success|failed|timeout), progress, last_error, started_at, finished_at, initiated_by

**API örnekleri:**

- `POST /api/admin/ota/start` → `{ "device_id": 42, "firmware_version": "0.50.0", "force": false }`
- `GET /api/admin/ota/jobs/{id}`
- `GET /api/device/{id}/telemetry/latest`

### 7.6 Firmware artefakt

- Build çıktısı: ESP-IDF `*.bin` (proje: `mqtt_tcp_custom_outbox`)
- Public erişilebilir URL (HTTPS önerilir)
- `version` string = cihaz `PROJECT_VER` ile uyumlu (örn. `0.50.0`)

---

## 8. Uçtan uca OTA örneği

1. Firmware `0.50.0` yüklendi: `https://185.33.234.10/myhexnet/firmware/io-v0.50.0.bin`
2. Cihaz MAC `A4:CF:12:34:56:78` → command topic: `hexnet/v1/1/15/command`
3. Platform publish:

```json
{
  "cmd": "ota_start",
  "args": {
    "url": "https://185.33.234.10/myhexnet/firmware/io-v0.50.0.bin",
    "version": "0.50.0",
    "force": false
  }
}
```

4. Telemetry: `ota.state` = `downloading`, `progress` artar
5. Cihaz reboot
6. Telemetry: `ota.state` = `idle`, `device.fw` = `0.50.0`

---

## 9. Platform QA checklist

- [ ] Resolve: yeni MAC → doğru telemetry/command topic
- [ ] Telemetry: standart şema parse; eski alan yok
- [ ] `ts` NTP sonrası > 0; öncesi 0 kabul
- [ ] `relay_set`, `dim_set`, `rgb_set`, `motor_set` çalışıyor
- [ ] `ota_start` → progress 0→100 → reboot → yeni `fw`
- [ ] Aynı sürüm + `force: false` → red (`same version`)
- [ ] Paralel ikinci OTA → red (`ota already in progress`)
- [ ] Hatalı URL → `failed` + `last_error`
- [ ] Offline cihaz → job timeout politikası

---

## 10. Güvenlik ve operasyon

- OTA ve kritik komutlar sadece yetkili admin
- Aynı cihaza eşzamanlı tek OTA
- Canary rollout (önce 1 cihaz)
- Başarısız OTA: dual partition sayesinde önceki slot kalabilir — panelde `last_error` ve son `fw` göster
- Self-signed HTTPS cihazda indirme hatasına yol açabilir; geçerli sertifika veya geçici HTTP (sadece test)

---

## 11. Beklenen platform teslimatları

1. **resolve-mqtt.php** (veya eşdeğeri) — MAC/hw kayıt, topic dönüş
2. **Telemetry ingestion** — standart JSON, `ota` + `device.fw` DB
3. **Command publisher** — MQTT `cmd` formatı
4. **Firmware CDN/storage** — sürüm, URL, metadata
5. **OTA panel + API** — tetikleme, ilerleme, geçmiş, timeout
6. **Admin UI** — canlı durum, OTA progress bar, hata mesajları

---

## 12. AI / geliştirici talimatı (kopyala-yapıştır)

> Sen Hexnet platform backend ve admin panel geliştiricisisin. Yukarıdaki sözleşmeye göre IO Module (IOModule-v0-50 / firmware v0.50) cihazlarıyla entegre olacak servisleri yaz. Cihaz firmware'i hazır; sen resolve API, telemetry parser, MQTT command publisher, firmware release yönetimi ve OTA job takibini implement et. Telemetry'de yalnızca belirtilen standart JSON alanlarını kullan; OTA için `ota_start` komutunu gönder ve `ota` bloğunu izle. SHA256 ve otomatik sürüm kontrolü cihazda yok — platform katmanında planla. Tüm endpoint'ler, DB şeması ve admin akışlarını bu belgedeki örneklerle uyumlu üret.

---

*Belge kaynağı: IO Module IOModule-v0-50 firmware — `hexnet_mqtt.c`, `hexnet_version.c`, `hexnet_resolve_mqtt.*`, `UserCodes/Ota/OtaManager.*`, `MQTT JSON.txt`, `partitions.csv`.*
