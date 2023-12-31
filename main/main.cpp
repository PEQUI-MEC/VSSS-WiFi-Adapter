#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_crc.h"


#include <algorithm>

#include "esp_now_msg.h"

#define BUF_SIZE (1024)


void flash_init() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void espnow_to_serial(void * args) {
    while (true) {
        MessagePacket packet;
        read_msg_queue(packet);
        // log bytes received
        add_peer(packet.mac_addr);
        // to string
        // Set last byte to 0 to be sure it is a null terminated string
        packet.data[packet.data_len] = '\0';
        // Create C++ string from byte array
        std::string text(packet.data.begin(), packet.data.begin() + packet.data_len);
        std::string msg = "U";
        for (auto& [id, mac] : ROBOT_MACS) {
            // compare mac address
            if (mac == packet.mac_addr) {
                msg = id;
                break;
            }
        }
        msg += "@" + text + '#';
        const char* send_data = msg.c_str();
        uart_write_bytes(UART_NUM_0, send_data, strlen(send_data));
        vTaskDelay(1); // Wait for 1 tick
    }
}

void uart_task(void *arg) {
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    std::string buffer;
    // reserve 500 bytes
    buffer.reserve(500);
    while (1) {
        // Wait for data to be received
        int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 0);
        // data[len] = 0;

        for (int i = 0; i < len; i++) {
            if (data[i] == '#') {
                if (buffer[0] >= 'A' && buffer[0] <= 'Z' && buffer[1] == '@') {
                    // char id = buffer[0];
                    // std::string msg = buffer.substr(2);
                    // send_string_msg(ROBOT_MACS[id], msg);
                    send_string_msg(BROADCAST_MAC, buffer);
                }
                buffer.clear();
            } else {
                buffer += data[i];
            }
        }

        vTaskDelay(1); // Wait for 1 tick
    }
}

extern "C" void app_main() {
    flash_init();
    setup_wifi();
    setup_espnow();

    // Configure UART port
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    //Add peer to all robots
    for(auto& robot : ROBOT_MACS){
        add_peer(robot.second);
    }

    // Start UART task
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);
    xTaskCreate(espnow_to_serial, "espnow_to_serial", 4096, NULL, 10, NULL);

    printf("Setup done!\n");
}

