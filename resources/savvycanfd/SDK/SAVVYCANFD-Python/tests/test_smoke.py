"""Smoke tests — verifies the native library loads, status_str works,
and discover() returns sanely with no devices attached. Designed to run
in CI without hardware."""
import usb_can


def test_module_imports():
    assert hasattr(usb_can, "can_sdk_init")
    assert hasattr(usb_can, "discover")
    assert hasattr(usb_can, "Device")


def test_status_str():
    assert usb_can.status_str(0) == "OK"
    assert "Invalid" in usb_can.status_str(-1)


def test_init_shutdown_clean():
    assert usb_can.can_sdk_init() == usb_can.CAN_OK
    assert usb_can.can_sdk_shutdown() == usb_can.CAN_OK


def test_discover_with_no_devices():
    usb_can.can_sdk_init()
    try:
        devs = usb_can.discover()
        assert isinstance(devs, list)
        # length depends on what's plugged in; just verify no exception
    finally:
        usb_can.can_sdk_shutdown()
