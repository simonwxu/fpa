/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <stdio.h>
#include <esp_log.h>
#include <esp_now.h>
#include <esp_pm.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <FreeRTOSConfig.h>
#include "esp_sleep.h"
#include "esp_mac.h"

//typedef esp_pm_config_t esp_pm_config_esp32s3_t;

#define NUM_BOARDS 5
#define MAC_ADDR_BYTES 6
#define MAX_CPU_FREQ 80
#define MIN_CPU_FREQ 10
#define HALF_SECOND 500000

static const char *TAG = "esp-now";

// all MAC addresses
uint8_t broadcastAddrs[NUM_BOARDS][MAC_ADDR_BYTES] = {{0x84, 0xfc, 0xe6, 0x7b, 0xaa, 0x68}, //0
                                                    {0x30, 0x30, 0xf9, 0x5a, 0x7c, 0x54}, //1
                                                    {0x3c, 0x84, 0x27, 0x04, 0xfb, 0xcc}, //2
                                                    {0x30, 0x30, 0xf9, 0x5a, 0x88, 0x48}, //3
                                                    {0x3c, 0x84, 0x27, 0x04, 0xfd, 0x18}}; //4

//index in array for current board
//change this when switching to new board
uint8_t myIndex = 0;

// Structure example to send data
// Must match the receiver structure
// max payload is 250 bytes
typedef struct struct_message {
    uint8_t id;
    char msg[128];
} struct_message;

struct_message sendData;
struct_message recvData;

//all the peers + self 
esp_now_peer_info_t peerInfo[NUM_BOARDS];

esp_pm_config_t pm_cfg;

void configure_pm(){
    pm_cfg.max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    pm_cfg.min_freq_mhz = MIN_CPU_FREQ;
    pm_cfg.light_sleep_enable = true;

    if (esp_pm_configure(&pm_cfg) != ESP_OK){
        ESP_LOGI(TAG, "POWER MANAGEMENT CONFIG ERROR");
    }
    else{
        ESP_LOGI(TAG, "POWER MANAGEMENT CONFIG SUCCESS");
    }
}

void init_wifi(){
    // Set device as a Wi-Fi Station
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_inactive_time(WIFI_IF_STA, 6));
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
}

void readMacAddr(){
    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x",
                    baseMac[0], baseMac[1], baseMac[2],
                    baseMac[3], baseMac[4], baseMac[5]);
    } else {
        ESP_LOGI(TAG, "Failed to read MAC address");
    }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    ESP_LOGI(TAG, "%s", status == ESP_NOW_SEND_SUCCESS ? "Packet Sent Successfully" : "Packet Failed to Send");
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&recvData, incomingData, sizeof(recvData));
    //ESP_LOGI(TAG, "From: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Bytes received: %d", len);
    ESP_LOGI(TAG, "From: %d", recvData.id);
    ESP_LOGI(TAG, "%s", recvData.msg);
}

void add_peers(){
    // Register all boards as peers except self
    for (int i = 0; i < NUM_BOARDS; i++){
        if (i == myIndex){
            continue;
        }
        memcpy(peerInfo[i].peer_addr, broadcastAddrs[i], MAC_ADDR_BYTES);
        peerInfo[i].channel = 0;  
        peerInfo[i].encrypt = false;
        // Add peer        
        if (esp_now_add_peer(&peerInfo[i]) != ESP_OK){
            ESP_LOGI(TAG, "Failed to add peer");
            return;
        }
    }
}

void send_to_peers(){
    //set send data
    sendData.id = myIndex;
    strcpy(sendData.msg, "TEST");
    esp_sleep_enable_timer_wakeup(2 * HALF_SECOND);
    while(1){
        // Send message via ESP-NOW to all peers
        esp_err_t result = esp_now_send(NULL, (uint8_t *) &sendData, sizeof(sendData));
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "Sent to all");
        }
        else {
            ESP_LOGI(TAG, "Error sending the data");
        }


        //vTaskDelay(pdMS_TO_TICKS(10000));
        //esp_sleep_enable_wifi_wakeup();
        //esp_light_sleep_start();
        
        recvData.id = 0xff;
        esp_light_sleep_start();
        //stay awake for 50 ms, this should be the absolute most needed
        vTaskDelay(pdMS_TO_TICKS(50));
         
    }
}
 
void app_main(void) {
    esp_err_t ret;
    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    //configure_pm();
 
    init_wifi();
    esp_wifi_disconnect();

    //readMacAddr();

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        ESP_LOGI(TAG, "Error initializing ESP-NOW");
        return;
    }
    else{
        ESP_LOGI(TAG, "Successfully initialized ESP-NOW");
    }

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);

    add_peers();

    xTaskCreate(&send_to_peers, "send", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);

    //vTaskDelay(pdMS_TO_TICKS(1000));

    //esp_light_sleep_start();

}

