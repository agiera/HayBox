#ifndef _COMMS_WEBUSBBACKEND_HPP
#define _COMMS_WEBUSBBACKEND_HPP

#include "core/CommunicationBackend.hpp"

// Set true once the WebUSB GC-adapter boot mode has taken over the USB stack.
// Read by the USB descriptor/vendor-control overrides to decide between the
// WebUSB and XInput personalities. Never set back to false at runtime (the mode
// is chosen once at boot).
extern bool g_webusb_mode;

// Communication backend for the "single-port GameCube adapter" boot mode
// (entered by holding Y at plugin). It reconfigures the composite USB device to
// advertise WebUSB (VID 0x057E / PID 0x0337) so the browser calibration/metadata
// host can read and write this controller's GameCube metadata over USB control
// transfers. It does not emit any controller reports.
class WebUSBBackend : public CommunicationBackend {
  public:
    WebUSBBackend(InputSource **input_sources, size_t input_source_count);
    ~WebUSBBackend();
    void SendReport();
};

#endif
