#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "rom/ets_sys.h"

#define WIFI_STA_SSID "uwb"
#define WIFI_STA_PASS "8706012345"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_CONNECT_TIMEOUT_MS 30000

#define DMX_UART_NUM UART_NUM_1
#define DMX_TX_PIN GPIO_NUM_17
#define DMX_DIR_PIN GPIO_NUM_18

#define DMX_CHANNELS 512
#define DMX_FRAME_SIZE (DMX_CHANNELS + 1)  // + start code

#define DMX_BREAK_US 100
#define DMX_MAB_US 12
#define DMX_FRAME_PERIOD_MS 25
#define DMX_UART_RX_BUF_SIZE 256

static const char *TAG = "dmx";

static uint8_t dmx_frame[DMX_FRAME_SIZE];
static SemaphoreHandle_t dmx_mutex;
static EventGroupHandle_t wifi_event_group;

static esp_err_t root_handler(httpd_req_t *req);
static esp_err_t set_handler(httpd_req_t *req);
static esp_err_t set_all_handler(httpd_req_t *req);

static void dmx_uart_init(void) {
  const uart_config_t uart_cfg = {
      .baud_rate = 250000,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_2,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
      .source_clk = UART_SCLK_DEFAULT,
#endif
  };

  // ESP-IDF 4.4 requires a non-zero RX buffer when installing the UART driver.
  ESP_ERROR_CHECK(
      uart_driver_install(DMX_UART_NUM, DMX_UART_RX_BUF_SIZE, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(DMX_UART_NUM, &uart_cfg));
  ESP_ERROR_CHECK(uart_set_pin(DMX_UART_NUM, DMX_TX_PIN, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void dmx_direction_init(void) {
  const gpio_config_t io_cfg = {
      .pin_bit_mask = (1ULL << DMX_DIR_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&io_cfg));

  // TX mode for MAX485/HW-519: DE=1, /RE=1 (tie both to this pin).
  ESP_ERROR_CHECK(gpio_set_level(DMX_DIR_PIN, 1));
}

static void dmx_set_channel(uint16_t channel, uint8_t value) {
  if (channel == 0 || channel > DMX_CHANNELS) {
    return;
  }

  xSemaphoreTake(dmx_mutex, portMAX_DELAY);
  dmx_frame[channel] = value;
  xSemaphoreGive(dmx_mutex);
}

static void dmx_set_all(uint8_t value) {
  xSemaphoreTake(dmx_mutex, portMAX_DELAY);
  for (int i = 1; i <= DMX_CHANNELS; i++) {
    dmx_frame[i] = value;
  }
  xSemaphoreGive(dmx_mutex);
}

static void dmx_send_frame(const uint8_t *frame) {
  // BREAK: line held low for at least 88 us (DMX512 minimum).
  ESP_ERROR_CHECK(uart_set_line_inverse(DMX_UART_NUM, UART_SIGNAL_TXD_INV));
  ets_delay_us(DMX_BREAK_US);

  // Mark-after-break (line high for at least 8 us).
  ESP_ERROR_CHECK(uart_set_line_inverse(DMX_UART_NUM, UART_SIGNAL_INV_DISABLE));
  ets_delay_us(DMX_MAB_US);

  const int written = uart_write_bytes(DMX_UART_NUM, (const char *)frame,
                                       DMX_FRAME_SIZE);
  if (written != DMX_FRAME_SIZE) {
    ESP_LOGW(TAG, "Short DMX frame write: %d of %d bytes", written,
             DMX_FRAME_SIZE);
  }
  ESP_ERROR_CHECK(uart_wait_tx_done(DMX_UART_NUM, pdMS_TO_TICKS(50)));
}

static void dmx_task(void *arg) {
  (void)arg;
  uint8_t tx_frame[DMX_FRAME_SIZE];

  while (1) {
    xSemaphoreTake(dmx_mutex, portMAX_DELAY);
    memcpy(tx_frame, dmx_frame, sizeof(tx_frame));
    xSemaphoreGive(dmx_mutex);

    dmx_send_frame(tx_frame);
    vTaskDelay(pdMS_TO_TICKS(DMX_FRAME_PERIOD_MS));
  }
}

static void spiffs_init(void) {
  const esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true,
  };

  ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

  size_t total = 0;
  size_t used = 0;
  ESP_ERROR_CHECK(esp_spiffs_info(NULL, &total, &used));
  ESP_LOGI(TAG, "SPIFFS mounted: %u / %u bytes used", (unsigned int)used,
           (unsigned int)total);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  (void)arg;

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
    return;
  }

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    ESP_LOGW(TAG, "WiFi disconnected, retrying...");
    esp_wifi_connect();
    return;
  }

  if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "WiFi connected, got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(TAG, "Open http://" IPSTR " in your browser",
             IP2STR(&event->ip_info.ip));
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

static void wifi_station_init(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  ESP_ERROR_CHECK(sta_netif == NULL ? ESP_FAIL : ESP_OK);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &wifi_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &wifi_event_handler, NULL));

  wifi_config_t sta_cfg = {0};
  memcpy(sta_cfg.sta.ssid, WIFI_STA_SSID, strlen(WIFI_STA_SSID));
  memcpy(sta_cfg.sta.password, WIFI_STA_PASS, strlen(WIFI_STA_PASS));
  sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "Connecting to WLAN SSID=%s ...", WIFI_STA_SSID);

  const EventBits_t bits =
      xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE,
                          pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
  ESP_ERROR_CHECK((bits & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_FAIL);
}

static esp_err_t root_handler(httpd_req_t *req) {
  FILE *f = fopen("/spiffs/index.html", "rb");
  if (f == NULL) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "index.html missing");
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "text/html");

  char chunk[1024];
  size_t read_len = 0;
  do {
    read_len = fread(chunk, 1, sizeof(chunk), f);
    if (read_len > 0) {
      if (httpd_resp_send_chunk(req, chunk, read_len) != ESP_OK) {
        fclose(f);
        httpd_resp_sendstr_chunk(req, NULL);
        return ESP_FAIL;
      }
    }
  } while (read_len > 0);

  fclose(f);
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

static bool read_query_uint(httpd_req_t *req, const char *key, unsigned int *out) {
  char query[64];
  char value[16];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    return false;
  }
  if (httpd_query_key_value(query, key, value, sizeof(value)) != ESP_OK) {
    return false;
  }

  char *end = NULL;
  unsigned long parsed = strtoul(value, &end, 10);
  if (end == value || *end != '\0') {
    return false;
  }
  *out = (unsigned int)parsed;
  return true;
}

static esp_err_t set_handler(httpd_req_t *req) {
  unsigned int channel = 0;
  unsigned int value = 0;
  if (!read_query_uint(req, "ch", &channel) || !read_query_uint(req, "v", &value) ||
      channel < 1 || channel > DMX_CHANNELS || value > 255) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Use /set?ch=1..512&v=0..255");
    return ESP_FAIL;
  }

  dmx_set_channel((uint16_t)channel, (uint8_t)value);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, "OK");
  return ESP_OK;
}

static esp_err_t set_all_handler(httpd_req_t *req) {
  unsigned int value = 0;
  if (!read_query_uint(req, "v", &value) || value > 255) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Use /all?v=0..255");
    return ESP_FAIL;
  }

  dmx_set_all((uint8_t)value);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, "OK");
  return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 8;

  httpd_handle_t server = NULL;
  if (httpd_start(&server, &config) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start HTTP server");
    return NULL;
  }

  const httpd_uri_t root_uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = root_handler,
      .user_ctx = NULL,
  };

  const httpd_uri_t set_uri = {
      .uri = "/set",
      .method = HTTP_GET,
      .handler = set_handler,
      .user_ctx = NULL,
  };

  const httpd_uri_t set_all_uri = {
      .uri = "/all",
      .method = HTTP_GET,
      .handler = set_all_handler,
      .user_ctx = NULL,
  };

  httpd_register_uri_handler(server, &root_uri);
  httpd_register_uri_handler(server, &set_uri);
  httpd_register_uri_handler(server, &set_all_uri);

  return server;
}

void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  memset(dmx_frame, 0, sizeof(dmx_frame));
  dmx_mutex = xSemaphoreCreateMutex();
  ESP_ERROR_CHECK(dmx_mutex == NULL ? ESP_FAIL : ESP_OK);
  wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(wifi_event_group == NULL ? ESP_FAIL : ESP_OK);

  dmx_uart_init();
  dmx_direction_init();
  spiffs_init();
  wifi_station_init();
  start_webserver();

  ESP_LOGI(TAG, "DMX transmitter started on UART1 TX=%d, DIR=%d", DMX_TX_PIN,
           DMX_DIR_PIN);

  xTaskCreatePinnedToCore(dmx_task, "dmx_task", 4096, NULL, 10, NULL, 1);
}
