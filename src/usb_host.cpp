// This translation unit is always compiled, but USB Host support is optional.
// When `DISABLE_USB_HOST` is defined (e.g. dev USB-CDC builds), provide stubs
// and avoid pulling in host-only headers/libs.

#include "usb_host.h"

#ifdef DISABLE_USB_HOST

bool usb_host_init() {
    return false;
}

void usb_host_poll() {
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
#include <freertos/semphr.h>
#include <usb/usb_host.h>
#include <esp_log.h>

#include "hid_host.h"

static const char* TAG = "USB_HOST";

static std::atomic<bool> g_gamepad_connected{false};

// Automatic recovery if the controller fails to enumerate after boot.
// Set to 0 to disable.
#ifndef USB_HOST_AUTO_RECOVER_AFTER_MS
#define USB_HOST_AUTO_RECOVER_AFTER_MS 8000
#endif
#ifndef USB_HOST_AUTO_RECOVER_MAX_ATTEMPTS
#define USB_HOST_AUTO_RECOVER_MAX_ATTEMPTS 3
#endif
#ifndef USB_HOST_AUTO_RECOVER_COOLDOWN_MS
#define USB_HOST_AUTO_RECOVER_COOLDOWN_MS 2000
#endif

typedef struct {
    hid_host_device_handle_t hid_device_handle;
    hid_host_driver_event_t event;
    void* arg;
} hid_host_event_queue_t;

static QueueHandle_t hid_host_event_queue = nullptr;
static std::atomic<bool> g_hid_task_shutdown{false};
static TaskHandle_t g_hid_task_handle = nullptr;

static std::atomic<bool> g_usb_shutdown{false};
static TaskHandle_t g_usb_task_handle = nullptr;
static TaskHandle_t g_control_task_handle = nullptr;

static SemaphoreHandle_t g_usb_lifecycle_mutex = nullptr;
static std::atomic<bool> g_hid_installed{false};

static unsigned long g_usb_last_init_ms = 0;
static unsigned long g_usb_last_recover_ms = 0;
static uint8_t g_usb_recover_attempts = 0;

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
            hid_parser_reset();

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
    QueueHandle_t q = hid_host_event_queue;
    if (q != nullptr) {
        xQueueSend(q, &evt_queue, 0);
    }
}

static void hid_host_task(void* pvParameters) {
    hid_host_event_queue_t evt_queue;

    while (!g_hid_task_shutdown.load()) {
        if (xQueueReceive(hid_host_event_queue, &evt_queue, pdMS_TO_TICKS(50))) {
            hid_host_device_event(evt_queue.hid_device_handle, evt_queue.event, evt_queue.arg);
        }
    }

    QueueHandle_t q = hid_host_event_queue;
    hid_host_event_queue = nullptr;
    if (q != nullptr) {
        xQueueReset(q);
        vQueueDelete(q);
    }
    g_hid_task_handle = nullptr;
    vTaskDelete(nullptr);
}

static void usb_lib_task(void* arg) {
    g_control_task_handle = (TaskHandle_t)arg;
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

    xTaskNotifyGive(g_control_task_handle);

    while (!g_usb_shutdown.load()) {
        uint32_t event_flags;
        usb_host_lib_handle_events(pdMS_TO_TICKS(250), &event_flags);

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }

    // Short delay to allow clients to clean up.
    vTaskDelay(pdMS_TO_TICKS(10));
    for (int attempt = 0; attempt < 3; attempt++) {
        err = usb_host_uninstall();
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "USB Host uninstall failed (%s), retrying...", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB Host uninstall failed: %s", esp_err_to_name(err));
    }

    if (g_control_task_handle != nullptr) {
        xTaskNotifyGive(g_control_task_handle);
    }
    g_usb_task_handle = nullptr;
    vTaskDelete(nullptr);
}

static void usb_host_stop_locked();

static bool usb_host_start_locked() {
    g_usb_shutdown = false;
    g_hid_task_shutdown = false;
    g_gamepad_connected = false;
    g_control_task_handle = xTaskGetCurrentTaskHandle();

    hid_host_event_queue = xQueueCreate(10, sizeof(hid_host_event_queue_t));
    if (hid_host_event_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return false;
    }

    // Clear any pending notifications before waiting.
    (void)ulTaskNotifyTake(pdTRUE, 0);

    BaseType_t task_created = xTaskCreatePinnedToCore(
        usb_lib_task,
        "usb_events",
        4096,
        (void*)g_control_task_handle,
        2,
        &g_usb_task_handle,
        0
    );

    if (task_created != pdTRUE) {
        ESP_LOGE(TAG, "Failed to create USB lib task");
        QueueHandle_t q = hid_host_event_queue;
        hid_host_event_queue = nullptr;
        if (q != nullptr) {
            vQueueDelete(q);
        }
        return false;
    }

    // Wait for USB Host library to be ready.
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0) {
        ESP_LOGE(TAG, "USB Host library initialization timeout");
        usb_host_stop_locked();
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
        usb_host_stop_locked();
        return false;
    }
    g_hid_installed = true;

    task_created = xTaskCreate(hid_host_task, "hid_task", 4096, nullptr, 2, &g_hid_task_handle);
    if (task_created != pdTRUE) {
        ESP_LOGE(TAG, "Failed to create HID task");
        usb_host_stop_locked();
        return false;
    }

    g_usb_last_init_ms = millis();
    ESP_LOGI(TAG, "USB Host initialized");
    return true;
}

static void usb_host_stop_locked() {
    g_gamepad_connected = false;

    if (g_hid_installed.load()) {
        esp_err_t err = hid_host_uninstall();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "HID Host uninstall failed: %s", esp_err_to_name(err));
        }
        g_hid_installed = false;
    }

    g_hid_task_shutdown = true;
    // Allow HID task to observe shutdown and delete its queue.
    vTaskDelay(pdMS_TO_TICKS(75));

    if (g_usb_task_handle != nullptr) {
        g_usb_shutdown = true;
        // Clear any pending notifications before waiting for shutdown ack.
        (void)ulTaskNotifyTake(pdTRUE, 0);
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000)) == 0) {
            ESP_LOGE(TAG, "USB Host shutdown timeout");
        }
    }

    // Ensure parser + state are reset on stop.
    hid_parser_reset();
    if (xSemaphoreTake(g_gamepad_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(&g_gamepad_state, 0, sizeof(g_gamepad_state));
        g_gamepad_state.changed = true;
        xSemaphoreGive(g_gamepad_mutex);
    }
}

bool usb_host_init() {
    if (g_usb_lifecycle_mutex == nullptr) {
        g_usb_lifecycle_mutex = xSemaphoreCreateMutex();
        if (g_usb_lifecycle_mutex == nullptr) {
            ESP_LOGE(TAG, "Failed to create lifecycle mutex");
            return false;
        }
    }

    if (xSemaphoreTake(g_usb_lifecycle_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ESP_LOGE(TAG, "USB Host init mutex timeout");
        return false;
    }

    // Already running.
    if (g_usb_task_handle != nullptr && g_hid_installed.load()) {
        xSemaphoreGive(g_usb_lifecycle_mutex);
        return true;
    }

    // If partially running, stop first.
    if (g_usb_task_handle != nullptr || g_hid_installed.load() || hid_host_event_queue != nullptr) {
        usb_host_stop_locked();
    }

    bool ok = usb_host_start_locked();
    xSemaphoreGive(g_usb_lifecycle_mutex);
    return ok;
}

void usb_host_poll() {
    if (USB_HOST_AUTO_RECOVER_AFTER_MS == 0) {
        return;
    }

    if (g_usb_lifecycle_mutex == nullptr) {
        return;
    }

    if (g_gamepad_connected.load()) {
        g_usb_recover_attempts = 0;
        return;
    }

    const unsigned long now = millis();
    if (now - g_usb_last_init_ms < (unsigned long)USB_HOST_AUTO_RECOVER_AFTER_MS) {
        return;
    }
    if (g_usb_recover_attempts >= USB_HOST_AUTO_RECOVER_MAX_ATTEMPTS) {
        return;
    }
    if (now - g_usb_last_recover_ms < (unsigned long)USB_HOST_AUTO_RECOVER_COOLDOWN_MS) {
        return;
    }

    if (xSemaphoreTake(g_usb_lifecycle_mutex, 0) != pdTRUE) {
        return;
    }

    // Double-check under lock.
    if (g_gamepad_connected.load()) {
        xSemaphoreGive(g_usb_lifecycle_mutex);
        return;
    }

    g_usb_recover_attempts++;
    g_usb_last_recover_ms = now;
    ESP_LOGW(
        TAG,
        "No gamepad after %lu ms; restarting USB host (attempt %u/%u)",
        (unsigned long)(now - g_usb_last_init_ms),
        (unsigned int)g_usb_recover_attempts,
        (unsigned int)USB_HOST_AUTO_RECOVER_MAX_ATTEMPTS
    );

    usb_host_stop_locked();
    (void)usb_host_start_locked();
    xSemaphoreGive(g_usb_lifecycle_mutex);
}

bool usb_host_gamepad_connected() {
    return g_gamepad_connected.load();
}

#endif  // !DISABLE_USB_HOST
