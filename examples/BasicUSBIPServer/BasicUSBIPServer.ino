// BasicUSBIPServer — minimal example using the esp_usbip_server library.
//
// Hardware: ESP32-S3 with ARDUINO_USB_MODE=0 (USB Host mode).
// After flashing, connect a USB printer to the ESP32-S3 USB port, then
// attach from a Linux/Windows host with:
//   usbip attach -r <ESP32_IP> -b 1-1

#include <WiFi.h>
#include <EspUsbIpServer.h>

// ── Configuration ─────────────────────────────────────────────────────────────
const char* WIFI_SSID = "YourSSID";
const char* WIFI_PASS = "YourPassword";

// ── Globals ───────────────────────────────────────────────────────────────────
UsbDevice   usbDevice;
UsbIpServer usbipServer;

// ── USB Host task (runs on Core 1) ────────────────────────────────────────────
void usbHostTask(void*) {
    usbDevice.begin();
    for (;;) {
        usbDevice.task();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ── Arduino setup ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // USB Host must run on Core 1 to avoid conflicts with the WiFi stack
    xTaskCreatePinnedToCore(usbHostTask, "usb_host", 4096, nullptr, 5, nullptr, 1);

    // Connect to WiFi
    Serial.printf("Connecting to %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nConnected — IP: %s\n", WiFi.localIP().toString().c_str());

    // begin() spawns the server task pinned to this core (Core 1)
    usbipServer.begin(&usbDevice);

    usbipServer.onClientConnect([](const char* ip) {
        Serial.printf("USB/IP client attached: %s\n", ip);
    });
    usbipServer.onClientDisconnect([](const char* ip, uint32_t duration) {
        Serial.printf("USB/IP client detached: %s (was attached %us)\n", ip, duration);
    });

    Serial.printf("USB/IP server listening on %s:3240\n",
                  WiFi.localIP().toString().c_str());
}

// ── Arduino loop ──────────────────────────────────────────────────────────────
void loop() {
    // Server runs in its own task — nothing needed here.
    delay(1000);
}
