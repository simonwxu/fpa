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
#define LOW_POWER_MANAGEMENT_BASE_ADDR 0x60008000
#define RTC_CNTL_WAKEUP_STATE_REG_OFFSET 0x003c
#define WAKEUP_CNTL_ADDR (LOW_POWER_MANAGEMENT_BASE_ADDR | RTC_CNTL_WAKEUP_STATE_REG_OFFSET)
#define WIFI_CNTL_BIT 0x20
//wait time for RX in MS
//100 occasionally fails when 5 packets need to be sent
#define WAIT_TIME 150
#define MESSAGE_LENGTH 240

static const char *TAG = "esp-now";

// all MAC addresses
uint8_t broadcastAddrs[NUM_BOARDS][MAC_ADDR_BYTES] = {{0x84, 0xfc, 0xe6, 0x7b, 0xaa, 0x68}, //0
                                                    {0x30, 0x30, 0xf9, 0x5a, 0x7c, 0x54}, //1
                                                    {0x3c, 0x84, 0x27, 0x04, 0xfb, 0xcc}, //2
                                                    {0x30, 0x30, 0xf9, 0x5a, 0x88, 0x48}, //3
                                                    {0x3c, 0x84, 0x27, 0x04, 0xfd, 0x18}}; //4
//M = 1, S = 0
uint8_t master = 1;

//index in array for current board
//change this when switching to new board
uint8_t myIndex = 1;

//for checking when data is received
uint8_t data_recv = 0;

// Structure example to send data
// Must match the receiver structure
// max payload is 250 bytes
typedef struct struct_message {
    uint8_t id;
    char msg[MESSAGE_LENGTH];
} struct_message;

struct_message sendData;
struct_message recvData;

//testing how much data can be sent
const char *test_str = "My tea's gone cold, I'm wondering why I"
                        "Got out of bed at all"
                        "The morning rain clouds up my window (Window)"
                        "And I can't see at all"
                        "And even if I could, it'd all be grey"
                        "But your picture on my wall"
                        "It reminds me that it's not so bad, it's not so bad (Bad)"
                        "Dear Slim, I wrote you, but you still ain't callin'"
                        "I left my cell, my pager and my home phone at the bottom"
                        "I sent two letters back in autumn, you must not've got 'em"
                        "There prob'ly was a problem at the post office or somethin'"
                        "Sometimes I scribble addresses too sloppy when I jot 'em"
                        "But anyways, fuck it, what's been up, man? How's your daughter?"
                        "My girlfriend's pregnant too, I'm 'bout to be a father"
                        "If I have a daughter, guess what I'ma call her? I'ma name her Bonnie"
                        "I read about your Uncle Ronnie too, I'm sorry"
                        "I had a friend kill himself over some bitch who didn't want him"
                        "I know you prob'ly hear this every day, but I'm your biggest fan"
                        "I even got the underground shit that you did with Skam"
                        "I got a room full of your posters and your pictures, man"
                        "I like the shit you did with Rawkus too, that shit was phat"
                        "Anyways, I hope you get this, man, hit me back"
                        "Just to chat, truly yours, your biggest fan, this is Stan";
                        //string literals are automatically null terminated

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
    esp_wifi_disconnect();
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
    sendData.id = myIndex;
}

void send_to_peers(){
    //set send data
    strcpy(sendData.msg, "TEST");
    esp_sleep_enable_timer_wakeup(4 * HALF_SECOND);
    
    while(1){
        /*
        if (esp_now_send(NULL, (uint8_t *) &sendData, sizeof(sendData)) == ESP_OK){
            ESP_LOGI(TAG, "sent");
        }
        //*/
        init_wifi();
        //send to all peers, we don't know which is master (at least in current design)
        esp_now_send(NULL, (uint8_t *) &sendData, sizeof(sendData));
        /*
        for (uint8_t i = 0; i < WAIT_TIME; i++){
            if (data_recv == 1){
                data_recv = 0;
                ESP_LOGI(TAG, "MS WAITED: %d", i);
                //break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        */
        vTaskDelay(pdMS_TO_TICKS(WAIT_TIME));
        esp_wifi_stop();


        //vTaskDelay(pdMS_TO_TICKS(1000));
        esp_light_sleep_start();
        
        //recvData.id = 0xff;
        //esp_light_sleep_start();
        //stay awake for 50 ms, this should be the absolute most needed
        //vTaskDelay(pdMS_TO_TICKS(50));
         
    }
}

void respond(){
    //strcpy(sendData.msg, test_str);
        //master waits for message from slave
        //wait for message
        //what if received at the same time?
        //does it matter? we're sending to all by default anyways so doesn't matter if 2 received at same time (for now)
        //TODO: violating watchdog, will fix later?
        //while(!data_recv);
    
    uint8_t destination = recvData.id;
    uint8_t string_terminated = 0;
    uint8_t packet_counter = 0;
    while(!string_terminated){
        for (int i = 0; i < MESSAGE_LENGTH; i++){
            char c = *(test_str + i + (240 * packet_counter));
            //currently NOT sending the null terminator, could this be an issue?
            if (c == '\0'){
                string_terminated = 1;
                break;
            }
            sendData.msg[i] = c;
        }
        esp_now_send(peerInfo[destination].peer_addr, (uint8_t *) &sendData, sizeof(sendData));
        /*esp_err_t result = esp_now_send(peerInfo[destination].peer_addr, (uint8_t *) &sendData, sizeof(sendData));
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "Sent to %d", destination);
        }
        else {
            ESP_LOGI(TAG, "Error sending the data");
        }
        */
        packet_counter++;
    }

    
    
    
            //*/
            //TODO: it might make sense to set a max wait time before losing master status, but for testing we just want to define 1 as always master for now

}

// callback when data is sent
//status is the ACK
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  //does not logging improve performance?
    if (master){
        if (ESP_NOW_SEND_SUCCESS){
            ESP_LOGI(TAG, "%d: Delivery Success", (int)recvData.id);
        }
        else{
            ESP_LOGI(TAG, "%d: Delivery Fail", (int)recvData.id);
        }
    }
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&recvData, incomingData, sizeof(recvData));
    
    //ESP_LOGI(TAG, "From: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    if (master){
        ESP_LOGI(TAG, "Bytes received: %d", len);
        ESP_LOGI(TAG, "From: %d", recvData.id);
        ESP_LOGI(TAG, "%s", recvData.msg);
        respond();
    }
    else{
        ESP_LOGI(TAG, "%s", recvData.msg);
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

    unsigned int *wakeup_cntl_reg = (unsigned int *) WAKEUP_CNTL_ADDR;
    ESP_LOGI(TAG, "%x", (int)wakeup_cntl_reg);
    ESP_LOGI(TAG, "WAKEUP REG: %08x", *wakeup_cntl_reg);

    //configure_pm();

    if (esp_sleep_enable_wifi_wakeup() == ESP_OK){
        ESP_LOGI(TAG, "enabled wifi wakeup");
    }

    ESP_ERROR_CHECK(esp_sleep_enable_uart_wakeup(0));

    ESP_LOGI(TAG, "WAKEUP REG: %08x", *wakeup_cntl_reg);

    *wakeup_cntl_reg |= (WIFI_CNTL_BIT << 15);

    ESP_LOGI(TAG, "WAKEUP REG: %08x", *wakeup_cntl_reg);
 
    init_wifi();
    

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

    if (!master){
        xTaskCreate(&send_to_peers, "send", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    }

    while(1){
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    /*
    else{
        xTaskCreate(&respond, "respond", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    }
    */
    //vTaskDelay(pdMS_TO_TICKS(1000));
    
    //esp_light_sleep_start();

}

