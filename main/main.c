#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "board_define.h"
#include "protocol.h"

#define ESP_MAXIMUM_RETRY  3

#define RING_BUFFER_SIZE 1024

#define WIFI_SSID_SIZE 64
#define WIFI_PW_SIZE 64

#define UART_BUFFER_SIZE 1024

#define WIFI_HOST_NAME_SIZE 128
#define SOCKET_IP_ADDRESS_SIZE 20

#define SOCKET_WRITE_DATA_SIZE 255
#define SOCKET_READ_DATA_SIZE 255

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

static const char *TAG = "AXDEN ESP32";

static QueueHandle_t uart0_queue;
static RingbufHandle_t uart_write_ring_buf;
static RingbufHandle_t socket_write_data_ring_buf;
static SemaphoreHandle_t uartDataSendxSemaphore = NULL;

static volatile uint8_t esp32_uart_step = 0x00;

static uint8_t wifi_connection_state = 0x00;
static uint32_t wifi_retry_connection_count = 0;

static uint16_t uart_buffer_size = 0;
static uint8_t uart_buffer[UART_BUFFER_SIZE] = { 0x00 };

static uint16_t uart_tx_buffer_length = 0;
static uint8_t uart_tx_buffer[UART_BUFFER_SIZE] = { 0x00 };

static uint16_t esp32_wifi_ssid_size = 0;
static uint8_t esp32_wifi_ssid[WIFI_SSID_SIZE];

static uint16_t esp32_wifi_pw_size = 0;
static uint8_t esp32_wifi_pw[WIFI_PW_SIZE];

static uint8_t esp32_socket_address_setting_type = 0x00;

static uint8_t esp32_socket_host_name[WIFI_HOST_NAME_SIZE];

static uint8_t esp32_socket_ip_address_size = 0;
static uint8_t esp32_socket_ip_address[SOCKET_IP_ADDRESS_SIZE];

static uint16_t esp32_socket_write_data_size = 0;
static uint8_t esp32_socket_write_data_buffer[SOCKET_WRITE_DATA_SIZE];

static uint32_t socket_recv_data_size = 0;
static uint8_t socket_recv_data[SOCKET_READ_DATA_SIZE];

static uint32_t esp32_socket_port_number = 0;

static RingbufHandle_t socket_write_data_ring_buf;

static void app_timer_callback(void *arg) {
	static uint8_t led_state = 0x00;

	if (led_state == 0x00) {

		gpio_set_level(GPIO_LED_2, 1);
		led_state = 0x01;

	} else {

		gpio_set_level(GPIO_LED_2, 0);
		led_state = 0x00;

	}

}

static void eps32_uart_write(uint8_t *data, uint16_t size) {

	xSemaphoreTake(uartDataSendxSemaphore, portMAX_DELAY);

	uart_write_bytes(UART_NUM_0, (const char*) data, size);

	uart_wait_tx_done(UART_NUM_0, 100 * portTICK_PERIOD_MS);

	xSemaphoreGive(uartDataSendxSemaphore);

}

void set_uart_usb_in_out_write_string(char *data) {

	uart_tx_buffer_length = 0;
	memset(uart_tx_buffer, 0x00, sizeof(uart_tx_buffer));

	uart_tx_buffer_length = sprintf((char*) uart_tx_buffer, "%s\r\n", data);

	eps32_uart_write(uart_tx_buffer, uart_tx_buffer_length);

}

static void connect_wifi() {

	set_uart_usb_in_out_write_string("Start Connect WiFi");

	wifi_config_t wifi_config;
	memset(&wifi_config, 0, sizeof(wifi_config_t));

	memcpy(wifi_config.sta.ssid, esp32_wifi_ssid, esp32_wifi_ssid_size);

	memcpy(wifi_config.sta.password, esp32_wifi_pw, esp32_wifi_pw_size);

	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

	ESP_LOGI(TAG, "connect to ap SSID:%s password:%s", esp32_wifi_ssid,
			esp32_wifi_pw);

	ESP_ERROR_CHECK(esp_wifi_start());

}

static void uart_event_task(void *pvParameters) {

	uart_event_t event;

	uint8_t *buffer = (uint8_t*) malloc(RD_BUF_SIZE);

	for (;;) {

		if (xQueueReceive(uart0_queue, (void*) &event,
				(portTickType) portMAX_DELAY)) {

			bzero(buffer, RD_BUF_SIZE);

			switch (event.type) {

			case UART_DATA: {

				uart_read_bytes(UART_NUM_0, buffer, event.size, portMAX_DELAY);

				for (uint16_t i = 0; i < event.size; i++) {

					uart_buffer[uart_buffer_size++] = buffer[i];

					if (uart_buffer_size > 2) {

						if (uart_buffer[uart_buffer_size - 2] == '\r'
								&& uart_buffer[uart_buffer_size - 1] == '\n') {

							if (esp32_uart_step == 0x00) {
								//WiFi Name

								esp32_wifi_ssid_size = 0;
								memset(esp32_wifi_ssid, 0x00,
										sizeof(esp32_wifi_ssid));

								esp32_wifi_ssid_size = uart_buffer_size - 2;
								memcpy(esp32_wifi_ssid, uart_buffer,
										esp32_wifi_ssid_size);

								set_uart_usb_in_out_write_string(
										"WIFI PASSWROD : ");

								esp32_uart_step = 0x01;

							} else if (esp32_uart_step == 0x01) {
								//WiFi Password

								esp32_wifi_pw_size = 0;
								memset(esp32_wifi_pw, 0x00,
										sizeof(esp32_wifi_pw));

								esp32_wifi_pw_size = uart_buffer_size - 2;
								memcpy(esp32_wifi_pw, uart_buffer,
										esp32_wifi_pw_size);

								connect_wifi();

							} else if (esp32_uart_step == 0x02) {
								//Socket Write Data

								esp32_socket_write_data_size = 0;
								memset(esp32_socket_write_data_buffer, 0x00,
										sizeof(esp32_socket_write_data_buffer));

								esp32_socket_write_data_size = uart_buffer_size
										- 2;
								memcpy(esp32_socket_write_data_buffer,
										uart_buffer,
										esp32_socket_write_data_size);

								xRingbufferSend(socket_write_data_ring_buf,
										esp32_socket_write_data_buffer,
										esp32_socket_write_data_size,
										pdMS_TO_TICKS(10000));

							}

							uart_buffer_size = 0;
							break;

						}

					}

					if (uart_buffer_size >= sizeof(uart_buffer)) {

						uart_buffer_size = 0;

					}

				}

				break;

			}

			case UART_FIFO_OVF: {

				ESP_LOGI(TAG, "hw fifo overflow");
				uart_flush_input(UART_NUM_0);
				xQueueReset(uart0_queue);

				break;

			}

			case UART_BUFFER_FULL: {

				ESP_LOGI(TAG, "ring buffer full");
				uart_flush_input(UART_NUM_0);
				xQueueReset(uart0_queue);

				break;

			}

			case UART_BREAK: {
				ESP_LOGI(TAG, "uart rx break");
				break;

			}

			case UART_PARITY_ERR: {
				ESP_LOGI(TAG, "uart parity error");
				break;

			}

			case UART_FRAME_ERR: {
				ESP_LOGI(TAG, "uart frame error");
				break;

			}

			default: {

				ESP_LOGI(TAG, "uart event type: %d", event.type);
				break;

			}

			}

		}

	}

	free(buffer);
	buffer = NULL;

	vTaskDelete(NULL);

}

static void tcp_client_task(void *pvParameters) {

	uint8_t socket_address_setup_state = 0x00;

	char addr_str[128];

	int16_t addr_family;
	int32_t ip_protocol;
	uint32_t recv_ring_buf_len = 0;

	while (1) {

		uint8_t *recv_data = (uint8_t*) xRingbufferReceive(
				socket_write_data_ring_buf, &recv_ring_buf_len,
				(portTickType) portMAX_DELAY);

		if (recv_data != NULL && recv_ring_buf_len > 0) {

			socket_address_setup_state = 0x00;

			struct sockaddr_in dest_addr;

			if (esp32_socket_address_setting_type == SOCKET_HOSTNAME_TYPE) {

				struct hostent *host;
				struct ip4_addr *ip4_addr;

				host = gethostbyname((char* ) esp32_socket_host_name);

				if (!host) {

				} else {

					ip4_addr = (struct ip4_addr*) host->h_addr;
					dest_addr.sin_addr.s_addr = ip4_addr->addr;

					socket_address_setup_state = 0x01;

				}

			} else if (esp32_socket_address_setting_type == SOCKET_IP_TYPE) {

				dest_addr.sin_addr.s_addr = inet_addr(
						(char* ) esp32_socket_ip_address);

				socket_address_setup_state = 0x01;

			}

			if (socket_address_setup_state == 0x01) {

				dest_addr.sin_family = AF_INET;
				dest_addr.sin_port = htons(esp32_socket_port_number);
				addr_family = AF_INET;
				ip_protocol = IPPROTO_IP;
				inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

				int32_t socket_number = socket(addr_family, SOCK_STREAM,
						ip_protocol);

				if (socket_number < 0) {

					set_uart_usb_in_out_write_string("Socket create error");

				} else {

					int32_t socket_error = connect(socket_number,
							(struct sockaddr*) &dest_addr, sizeof(dest_addr));

					if (socket_error != 0) {

						set_uart_usb_in_out_write_string(
								"Socket connect error");

					} else {

						socket_error = send(socket_number, recv_data,
								recv_ring_buf_len, 0);

						if (socket_error < 0) {

							set_uart_usb_in_out_write_string(
									"Socket write error");

						} else {

							socket_recv_data_size = 0;
							memset(socket_recv_data, 0x00,
									sizeof(socket_recv_data));

							socket_recv_data_size = recv(socket_number,
									socket_recv_data,
									sizeof(socket_recv_data) - 1, 0);

							if (socket_recv_data_size <= 0) {

								set_uart_usb_in_out_write_string(
										"Socket read error");

							} else {

								set_uart_usb_in_out_write_string(
										"Socket Read data");

								set_uart_usb_in_out_write_string(
										(char*) socket_recv_data);

								if (socket_number != -1) {

									shutdown(socket_number, 0);

									close(socket_number);

								}

							}

						}

					}

				}

			} else {

				set_uart_usb_in_out_write_string("Socket address set error");

			}

			vRingbufferReturnItem(socket_write_data_ring_buf,
					(void*) recv_data);

		}

	}

	vTaskDelete(NULL);

}

static esp_err_t event_handler(void *ctx, system_event_t *event) {

	switch (event->event_id) {

	case SYSTEM_EVENT_AP_STACONNECTED: {

		ESP_LOGI(TAG, "station:" MACSTR " join, AID=%d",
				MAC2STR(event->event_info.sta_connected.mac),
				event->event_info.sta_connected.aid);

		break;

	}

	case SYSTEM_EVENT_AP_STADISCONNECTED: {

		ESP_LOGI(TAG, "station:" MACSTR "leave, AID=%d",
				MAC2STR(event->event_info.sta_disconnected.mac),
				event->event_info.sta_disconnected.aid);

		break;

	}

	case SYSTEM_EVENT_STA_START: {

		esp_wifi_connect();
		break;

	}

	case SYSTEM_EVENT_STA_GOT_IP: {

		wifi_connection_state = 0x01;

		wifi_retry_connection_count = 0;

		set_uart_usb_in_out_write_string("Connect WiFi Success");

		set_uart_usb_in_out_write_string("Write Data : ");

		esp32_uart_step = 0x02;

		break;

	}

	case SYSTEM_EVENT_STA_DISCONNECTED: {

		wifi_connection_state = 0x00;

		esp32_uart_step = 0x03;

		set_uart_usb_in_out_write_string("WiFi Disconnect, Start Retry");

		if (wifi_retry_connection_count < ESP_MAXIMUM_RETRY) {

			esp_wifi_connect();
			wifi_retry_connection_count++;
			ESP_LOGI(TAG, "retry to connect to the AP");

		} else {

			esp_restart();

		}

		ESP_LOGI(TAG, "connect to the AP fail");
		break;

	}

	default:
		break;
	}

	return ESP_OK;

}

void app_main(void) {

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT()
	;

	wifi_mode_t mode = WIFI_MODE_STA;

	nvs_flash_init();

	tcpip_adapter_init();

	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_wifi_set_mode(mode));

	///////////////////////Create Uart Semaphore//////////////

	vSemaphoreCreateBinary(uartDataSendxSemaphore);

	///////////////////////Create Ring Buffer/////////////////

	uart_write_ring_buf = xRingbufferCreate(RING_BUFFER_SIZE,
			RINGBUF_TYPE_NOSPLIT);

	socket_write_data_ring_buf = xRingbufferCreate(RING_BUFFER_SIZE,
			RINGBUF_TYPE_NOSPLIT);

	///////////////////////Init Uart/////////////////////////
	uart_config_t uart_config = { .baud_rate = 9600, .data_bits =
			UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits =
			UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_APB, };

	uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 20,
			&uart0_queue, 0);

	uart_param_config(UART_NUM_0, &uart_config);

	esp_log_level_set(TAG, ESP_LOG_INFO);

	uart_set_pin(UART_NUM_0, UART_1_TX_GPIO, UART_1_RX_GPIO, UART_PIN_NO_CHANGE,
	UART_PIN_NO_CHANGE);

	xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

	xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);

	gpio_pad_select_gpio(GPIO_LED_0);
	gpio_set_direction(GPIO_LED_0, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_LED_0, 0);

	gpio_pad_select_gpio(GPIO_LED_1);
	gpio_set_direction(GPIO_LED_1, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_LED_1, 0);

	gpio_pad_select_gpio(GPIO_LED_2);
	gpio_set_direction(GPIO_LED_2, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_LED_2, 0);

	const esp_timer_create_args_t periodic_timer_args = { .callback =
			&app_timer_callback, .name = "app_timer" };
	esp_timer_handle_t app_timer;
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &app_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(app_timer, 100000));

	//Setup Default server ip, port
	esp32_socket_ip_address_size = strlen(SERVER_IP);
	memcpy(esp32_socket_ip_address, SERVER_IP, strlen(SERVER_IP));

	esp32_socket_port_number = SERVER_PORT;

	esp32_socket_address_setting_type = SOCKET_IP_TYPE;

	set_uart_usb_in_out_write_string("AXDEN ESP32 WiFi Example");

	set_uart_usb_in_out_write_string("WiFi Name : ");

}
