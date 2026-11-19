# SAVVYCANFD Windows x64

Double-click `run_SAVVYCANFD.bat` to launch the GUI, or run
`bin\SAVVYCANFD.exe` directly. All required runtime libraries are
bundled inside `bin\`.

## Layout

```
SAVVYCANFD-Windows-x64\
├── bin\                Executables + Qt runtime + libusb DLL
├── lib\usb_can.lib     Import library (MSVC)
├── include\can\        Public C API headers
├── examples\basic_can.c Example source
├── run_SAVVYCANFD.bat  GUI launcher
└── README.md           this file
```

## Requirements

- Windows 10 or newer, x64
- Supported USB-CAN adapter connected
