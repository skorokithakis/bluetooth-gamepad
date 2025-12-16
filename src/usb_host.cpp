// This translation unit is always compiled, but USB Host support is optional.
// When `DISABLE_USB_HOST` is defined (e.g. dev USB-CDC builds), provide stubs
// and avoid pulling in host-only headers/libs.

#include "usb_host.h"

#ifdef DISABLE_USB_HOST

bool usb_host_init() {
    return false;
}

bool usb_host_gamepad_connected() {
    return false;
}

#else

#include "gamepad_state.h"
#include "hid_parser.h"

#include <Arduino.h>
#include <atomic>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <usb/usb_host.h>
#include <esp_log.h>

#include "hid_host.h"

static const char* TAG = "USB_HOST";

static std::atomic<bool> g_gamepad_connected{false};

typedef struct {
    hid_host_device_handle_t hid_device_handle;
    hid_host_driver_event_t event;
    void* arg;
} hid_host_event_queue_t;

static QueueHandle_t hid_host_event_queue = nullptr;

static void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                        const hid_host_interface_event_t event,
                                        void* arg) {
    uint8_t data[64] = {0};
    size_t data_length = 0;
    hid_host_dev_params_t dev_params;
    hid_host_device_get_params(hid_device_handle, &dev_params);

    switch (event) {
        case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
            if (hid_host_device_get_raw_input_report_data(hid_device_handle, data, sizeof(data), &data_length) == ESP_OK) {
                // Non-keyboard/mouse devices are gamepads.
                if (dev_params.proto != HID_PROTOCOL_KEYBOARD && dev_params.proto != HID_PROTOCOL_MOUSE) {
                    parse_hid_report(data, data_length);
                }
            }
            break;

        case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HID Device disconnected");
            g_gamepad_connected = false;

            // Clear gamepad state to release all buttons/axes.
            if (xSemaphoreTake(g_gamepad_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                memset(&g_gamepad_state, 0, sizeof(g_gamepad_state));
                g_gamepad_state.changed = true;
                xSemaphoreGive(g_gamepad_mutex);
            }

            hid_host_device_close(hid_device_handle);
            break;

        case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
            ESP_LOGW(TAG, "HID transfer error");
            break;
    }
}

static void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_driver_event_t event,
                                  void* arg) {
    hid_host_dev_params_t dev_params;
    hid_host_device_get_params(hid_device_handle, &dev_params);

    const hid_host_device_config_t dev_config = {
        .callback = hid_host_interface_callback,
        .callback_arg = nullptr,
    };

    switch (event) {
        case HID_HOST_DRIVER_EVENT_CONNECTED:
            ESP_LOGI(TAG, "HID Device connected, protocol: %d", dev_params.proto);

            if (hid_host_device_open(hid_device_handle, &dev_config) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to open HID device");
                return;
            }

            // Skip keyboards and mice - we only want gamepads.
            if (dev_params.proto == HID_PROTOCOL_KEYBOARD || dev_params.proto == HID_PROTOCOL_MOUSE) {
                ESP_LOGI(TAG, "Ignoring keyboard/mouse device");
                hid_host_device_close(hid_device_handle);
                return;
            }

            if (hid_host_device_start(hid_device_handle) == ESP_OK) {
                g_gamepad_connected = true;
                ESP_LOGI(TAG, "Gamepad started");
            } else {
                ESP_LOGE(TAG, "Failed to start HID device");
                hid_host_device_close(hid_device_handle);
            }
            break;

        default:
            break;
    }
}

static void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                                     const hid_host_driver_event_t event,
                                     void* arg) {
    const hid_host_event_queue_t evt_queue = {
        .hid_device_handle = hid_device_handle,
        .event = event,
        .arg = arg,
    };
    xQueueSend(hid_host_event_queue, &evt_queue, 0);
}

static void hid_host_task(void* pvParameters) {
    hid_host_event_queue_t evt_queue;

    while (true) {
        if (xQueueReceive(hid_host_event_queue, &evt_queue, pdMS_TO_TICKS(50))) {
            hid_host_device_event(evt_queue.hid_device_handle, evt_queue.event, evt_queue.arg);
        }
    }
}

static void usb_lib_task(void* arg) {
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };

    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB Host install failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }

    xTaskNotifyGive((TaskHandle_t)arg);

    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

bool usb_host_init() {
    hid_host_event_queue = xQueueCreate(10, sizeof(hid_host_event_queue_t));
    if (hid_host_event_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return false;
    }

    BaseType_t task_created = xTaskCreatePinnedToCore(
        usb_lib_task,
        "usb_events",
        4096,
        xTaskGetCurrentTaskHandle(),
        2,
        nullptr,
        0
    );

    if (task_created != pdTRUE) {
        ESP_LOGE(TAG, "Failed to create USB lib task");
        return false;
    }

    // Wait for USB Host library to be ready.
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0) {
        ESP_LOGE(TAG, "USB Host library initialization timeout");
        return false;
    }

    const hid_host_driver_config_t hid_host_driver_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_device_callback,
        .callback_arg = nullptr,
    };

    esp_err_t err = hid_host_install(&hid_host_driver_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HID Host install failed: %s", esp_err_to_name(err));
        return false;
    }

    task_created = xTaskCreate(hid_host_task, "hid_task", 4096, nullptr, 2, nullptr);
    if (task_created != pdTRUE) {
        ESP_LOGE(TAG, "Failed to create HID task");
        return false;
    }

    ESP_LOGI(TAG, "USB Host initialized");
    return true;
}

bool usb_host_gamepad_connected() {
    return g_gamepad_connected.load();
}

#endif  // !DISABLE_USB_HOST
