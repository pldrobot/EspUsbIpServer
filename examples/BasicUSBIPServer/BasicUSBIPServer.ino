// BasicUSBIPServer - minimal example using the EspUsbIpServer library.
//
// Hardware: ESP32-S3 built with ARDUINO_USB_MODE=0 (USB Host mode).
// Plug any full/low-speed USB device into the ESP32-S3 USB port, then attach it
// from a Linux/Windows host with:
//   usbip attach -r <ESP32_IP> -b 1-1

#include <WiFi.h>
#include <EspUsbIpServer.h>

const char* WIFI_SSID = "YourSSID";
const char* WIFI_PASS = "YourPassword";

UsbDevice   usbDevice;
UsbIpServer usbipServer;

void setup() {
    Serial.begin(115200);

    // Starts the USB host tasks; no polling needed afterwards.
    usbDevice.begin();

    Serial.printf("Connecting to %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nConnected, IP: %s\n", WiFi.localIP().toString().c_str());
    WiFi.setSleep(false);   // modem sleep adds latency to every transfer

    // Starts the USB/IP tasks, listening on port 3240.
    usbipServer.begin(&usbDevice);

    usbipServer.onClientConnect([](const char* ip) {
        Serial.printf("USB/IP client attached: %s\n", ip);
    });
    usbipServer.onClientDisconnect([](const char* ip, uint32_t duration) {
        Serial.printf("USB/IP client detached: %s (was attached %us)\n", ip, duration);
    });
}

void loop() {
    delay(1000);   // everything runs in its own tasks
}
