# MiniRobo2025-WS
部内ロボコンのプログラムです。
STM32のプログラムは、主に`./main/`フォルダ内にあります。
`./main/main_app.c`には、`void setup()`関数と`void loop()`関数があります。

## `./main/`以外の変更点
### ./Core/Src/main.c
```c
/* USER CODE BEGIN Includes */
#include "BoardAPI.h"
/* USER CODE END Includes */
```
```c
  /* USER CODE BEGIN 2 */
  setup();
  /* USER CODE END 2 */
```
```c
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  loop();
  }
  /* USER CODE END 3 */
```

### ./USB_DEVICE/App/usbd_cdc_if.c
```c
/* USER CODE BEGIN INCLUDE */
#include "BoardAPI.h"
/* USER CODE END INCLUDE */
```
```c
  /* USER CODE BEGIN 11 */
  void (*receivedCallback)(uint8_t *, uint32_t) = cdc_getReceivedCallback();
  if (receivedCallback != NULL) {
	  receivedCallback(Buf, *Len);
  }

  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);
  return (USBD_OK);
  /* USER CODE END 11 */
```

### ./USB_DEVICE/App/usbd_desc.c
```c
/* USER CODE BEGIN INCLUDE */
#include "BoardAPI/cdc.h"
/* USER CODE END INCLUDE */
```
```c
/* USER CODE BEGIN PRIVATE_DEFINES */
#undef USBD_MANUFACTURER_STRING
#define USBD_MANUFACTURER_STRING (cdc_getVendorName())
#undef USBD_PRODUCT_STRING_HS
#define USBD_PRODUCT_STRING_HS (cdc_getProductName())
/* USER CODE END PRIVATE_DEFINES */
```

## ライセンス
このプロジェクト全体は[MITライセンス](LICENSE)の下でライセンスされています。

- `./Drivers/STM32F4xx_HAL_Driver/`：BSD-3-Clause またはパッケージ内ライセンス
- `./Drivers/CMSIS/`：Apache License 2.0
- `./Drivers/CMSIS/Device/ST/STM32F4xx/`：Apache License 2.0 またはパッケージ内ライセンス
- `./Middlewares/ST/STM32_USB_Device_Library/`：SLA0044（ST Ultimate Liberty Software License Agreement）またはパッケージ内ライセンス

各ディレクトリ内の`LICENSE.txt`やパッケージライセンスも必ずご確認ください。