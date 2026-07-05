// Strong overrides of TinyUSB's weak device callbacks for the WebUSB GC-adapter
// mode.
//
// The XInput library (Adafruit_TinyUSB_XInput) also defines these three
// callbacks, but as *weak* symbols (TinyUSB declares them weak, and that
// declaration propagates weakness to any definition compiled in a translation
// unit that includes the TinyUSB headers). To reliably override the XInput
// versions we need *strong* definitions, so this file deliberately does NOT
// include any TinyUSB headers — it uses primitive, ABI-compatible parameter
// and return types and forwards to the real implementations in
// WebUSBBackend.cpp, which dispatch between the WebUSB and XInput personalities.

// Implemented in WebUSBBackend.cpp.
extern const void *webusb_get_app_drivers(unsigned char *driver_count);
extern const unsigned char *webusb_get_bos(void);
extern unsigned char webusb_vendor_control(
    unsigned char rhport,
    unsigned char stage,
    const void *request
);

// const usbd_class_driver_t *usbd_app_driver_get_cb(uint8_t *driver_count)
const void *usbd_app_driver_get_cb(unsigned char *driver_count) {
    return webusb_get_app_drivers(driver_count);
}

// uint8_t const *tud_descriptor_bos_cb(void)
const unsigned char *tud_descriptor_bos_cb(void) {
    return webusb_get_bos();
}

// bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
//                                 tusb_control_request_t const *request)
unsigned char tud_vendor_control_xfer_cb(
    unsigned char rhport,
    unsigned char stage,
    const void *request
) {
    return webusb_vendor_control(rhport, stage, request);
}
