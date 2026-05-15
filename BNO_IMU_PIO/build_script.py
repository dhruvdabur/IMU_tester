Import("env")
import os

# List of source files to compile
source_files = [
    "Core/Src/main.c",
    "Core/Src/bno055.c",
    "Core/Src/madgwick.c",
    "Core/Src/stm32f4xx_hal_msp.c",
    "Core/Src/stm32f4xx_it.c",
    "Core/Src/syscalls.c",
    "Core/Src/sysmem.c",
    "Core/Src/system_stm32f4xx.c",
    "USB_DEVICE/App/usb_device.c",
    "USB_DEVICE/App/usbd_cdc_if.c",
    "USB_DEVICE/App/usbd_desc.c",
    "USB_DEVICE/Target/usbd_conf.c",
    "Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c",
    "Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c",
    "Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c",
    "Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c",
]

# Add all the source files to the build
for file in source_files:
    env.Append(CPPPATH=[os.path.dirname(file)])
    # Convert to absolute path for the builder
    abs_file = os.path.join("$PROJECT_DIR", file)
    env.Append(SRC=[abs_file])


