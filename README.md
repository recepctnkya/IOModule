# Hexnet IO Module — IOModule-v0-50 (aktif geliştirme)

Bu klasör **tek aktif firmware geliştirme hattıdır**. Tüm yeni özellikler, OTA ve telemetri değişiklikleri burada yapılır.

| Kavram | Değer |
|--------|--------|
| **Geliştirme hattı** | `IOModule-v0-50` (bu repo klasörü) |
| **Firmware sürümü (şu an)** | `v0.50` / CMake `0.50.0` |
| **Donanım** | ESP32-S3, `Vango-Medium` |
| **Dondurulmuş hatlar** | `IOModule-v0-30`, `IOModule-v0-40` (yalnızca referans) |

## ESP-IDF CMD (Windows)

Proje klasöründe çift tık veya **ESP-IDF CMD** içinden:

| Komut | İşlev |
|--------|--------|
| **`idf.cmd`** | IDF ortamı + proje klasörü (shell açık kalır) |
| **`build.cmd`** | `idf.py build` |
| **`flash.cmd COM40`** | `idf.py -p COM40 flash` |
| **`monitor.cmd COM40`** | `idf.py -p COM40 monitor` |
| **`bfm.cmd COM40`** | build + flash + monitor |
| **`YUKLE.cmd`** | Port sorar, flash |
| **`clean.cmd`** | `idf.py fullclean` |

IDF yolu farklıysa: `idf_path.local.cmd.example` → `idf_path.local.cmd` kopyalayıp `IDF_PATH` düzenleyin.

```bat
cd IOModule-v0-50
idf.cmd
build.cmd
flash.cmd COM40
```

Linux/macOS: `cd IOModule-v0-50 && . $IDF_PATH/export.sh && idf.py build`

## Derleme çıktısı (platform OTA)

`build\vango_medium_v0_50.bin` — donanım + sürüm adı (`vango_medium` + `v0_50` ← CMake `0.50.0`).

Sürüm değişince dosya adı otomatik güncellenir (ör. `0.51.0` → `vango_medium_v0_51.bin`).

## Telemetri `device.fw`

Her yayında dolu sürüm gönderilir: `hexnet_firmware_version_string()` → çalışan partition `0.50.0` veya yedek `v0.50`.

## Workspace

VS Code / Cursor: `IOModule-v0-50.code-workspace`

## Platform

Panel ve API: `HexnetWorkspace/myhexnet` — OTA katalog sürümü ile `device.fw` eşleşmeli.
