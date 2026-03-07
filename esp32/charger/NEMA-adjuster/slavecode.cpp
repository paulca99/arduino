#include <WiFi.h>
#include <esp_now.h>
#include <AccelStepper.h>
#include <SolarPosition.h>
#include <time.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <Preferences.h>

// --- PIN CONFIGURATION ---
#define STEP_PIN 26
#define DIR_PIN  25
#define EN_PIN   27 
#define LIMIT_TOP 32    // Triggered when panel is at max elevation
#define LIMIT_BOTTOM 33 // Triggered when panel is at min elevation

#define STEPS_PER_MM 1828.57 
AccelStepper stepper(1, STEP_PIN, DIR_PIN); 
SolarPosition localSun(53.71, -2.09);
Preferences prefs;

typedef struct { uint32_t unixTime; int staggerMinutes; float manualAngle; float calibOffset; } struct_sync;
typedef struct { float angle; bool moving; uint8_t mac[6]; } struct_status;

bool timeSynced = false;
float angleOffset = 0.0;
unsigned long moveTrigger = 0;
int currentChannel = 1;

void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
    struct_sync msg; memcpy(&msg, data, sizeof(msg));
    if (msg.calibOffset != 0.0) {
        angleOffset += msg.calibOffset;
        prefs.putFloat("offset", angleOffset);
    } else if (msg.manualAngle >= 10.0) {
        float target = msg.manualAngle + angleOffset;
        float steps = (827.24 - (840.0 * cos(radians(target)))) * STEPS_PER_MM;
        digitalWrite(EN_PIN, LOW); 
        stepper.moveTo(steps);
    } else if (msg.unixTime > 1000000) {
        struct timeval tv = {(time_t)msg.unixTime, 0};
        settimeofday(&tv, NULL);
        timeSynced = true;
        moveTrigger = millis() + (msg.staggerMinutes * 60000);
    }
}

void setup() {
    pinMode(EN_PIN, OUTPUT); 
    digitalWrite(EN_PIN, HIGH); // Driver Disabled

    // Setup Limit Switches with Internal Pullups
    pinMode(LIMIT_TOP, INPUT_PULLUP);
    pinMode(LIMIT_BOTTOM, INPUT_PULLUP);

    prefs.begin("solar", false);
    angleOffset = prefs.getFloat("offset", 0.0);
    
    stepper.setMaxSpeed(800); 
    stepper.setAcceleration(300);
    
    WiFi.mode(WIFI_STA); 
    esp_now_init();
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    
    uint8_t bcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peer = {}; 
    memcpy(peer.peer_addr, bcast, 6); 
    esp_now_add_peer(&peer);
}

void loop() {
    // 1. Safety Check: Stop motor if limit hit
    bool topHit = (digitalRead(LIMIT_TOP) == LOW);
    bool bottomHit = (digitalRead(LIMIT_BOTTOM) == LOW);

    if (topHit || bottomHit) {
        // Stop movement immediately if hitting a limit
        if ((topHit && stepper.speed() > 0) || (bottomHit && stepper.speed() < 0)) {
            stepper.stop();
            stepper.setCurrentPosition(stepper.currentPosition()); // Hard stop
        }
    }

    // 2. Time Sync / Channel Hopping
    if (!timeSynced && (millis() % 5000 < 50)) { 
        currentChannel = (currentChannel % 13) + 1;
        esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
    }
    
    // 3. Solar Calculation & Move Trigger
    if (timeSynced && millis() > moveTrigger) {
        moveTrigger = millis() + 1800000;
        time_t now; time(&now);
        float elev = localSun.getSolarElevation(now + 900); 
        float target = constrain(90.0 - elev + angleOffset, 10.0, 65.0);
        float steps = (827.24 - (840.0 * cos(radians(target)))) * STEPS_PER_MM;
        
        // Only move if not blocked by a limit switch
        if (!(topHit && steps > stepper.currentPosition()) && 
            !(bottomHit && steps < stepper.currentPosition())) {
            digitalWrite(EN_PIN, LOW); 
            stepper.moveTo(steps);
        }
        
        struct_status r; r.angle = target; r.moving = true; WiFi.macAddress(r.mac);
        uint8_t bcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        esp_now_send(bcast, (uint8_t *) &r, sizeof(r));
    }
    
    // 4. Run Stepper
    stepper.run();
    
    // 5. Sleep & Power Management
    if (stepper.distanceToGo() == 0 && digitalRead(EN_PIN) == LOW) {
        delay(2000); 
        digitalWrite(EN_PIN, HIGH); // Power down motor
        
        struct_status r; r.angle = 0; r.moving = false; WiFi.macAddress(r.mac);
        uint8_t bcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        esp_now_send(bcast, (uint8_t *) &r, sizeof(r));
        
        esp_sleep_enable_timer_wakeup(1800ULL * 1000000ULL); 
        esp_light_sleep_start();
    }
}
