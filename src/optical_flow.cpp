#include <Arduino.h>
#include "optical_flow.h"
#include "config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- MSPv2 Function IDs ---
#define MSP2_SENSOR_OPTIC_FLOW  0x1F02
#define MSP2_SENSOR_RANGEFINDER 0x1F01

// Les valeurs initiales (tu pourras les modifier en direct via ton interface web)
#define FLOW_GYRO_MULT_X 1.6f  // Ajustement pour le Roll
#define FLOW_GYRO_MULT_Y 1.6f  // Ajustement pour le Pitch

// --- MSPv2 Parser States ---
enum MspState {
    MSP_IDLE,
    MSP_HEADER_X,
    MSP_HEADER_DIR,
    MSP_FLAG,
    MSP_FUNC_LO,
    MSP_FUNC_HI,
    MSP_SIZE_LO,
    MSP_SIZE_HI,
    MSP_PAYLOAD,
    MSP_CRC
};

// --- Données partagées (protégées par mutex) ---
static portMUX_TYPE flow_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile float flow_vel_x     = 0.0f;
static volatile float flow_vel_y     = 0.0f;
static volatile int   flow_qual      = 0;
static volatile float flow_raw_dx    = 0.0f;
static volatile float flow_raw_dy    = 0.0f;
static volatile bool  flow_new_data  = false;
static volatile float flow_pos_x     = 0.0f;  // Position intégrée X (cm)
static volatile float flow_pos_y     = 0.0f;  // Position intégrée Y (cm)

// Flag de reset position (set depuis main loop, lu par flow_task)
static volatile bool flow_reset_pos  = false;

static TaskHandle_t flow_task_handle = nullptr;

// --- Scale factor runtime-modifiable ---
static volatile float flow_scale_runtime = FLOW_SCALE_FACTOR;
static volatile float flow_gyro_mult_x_rt = FLOW_GYRO_MULT_X;
static volatile float flow_gyro_mult_y_rt = FLOW_GYRO_MULT_Y;

// --- Variables volatiles pour recevoir les angles depuis la boucle principale ---
volatile float _flow_angle_yaw    = 0.0f;
volatile float _flow_lidar_dist   = 0.0f;

// --- Variables volatiles pour compensation gyroscopique ---
volatile float _flow_gyro_pitch   = 0.0f;  // deg/s (instantané pour le gyro-gating)
volatile float _flow_gyro_roll    = 0.0f;  // deg/s (instantané pour le gyro-gating)

// --- Accumulateurs gyro intégrés à 250 Hz dans imu.cpp ---
extern volatile float flow_accum_pitch;  // degrés accumulés
extern volatile float flow_accum_roll;   // degrés accumulés

// --- Variables volatiles pour compensation inclinaison ---
volatile float _flow_angle_roll   = 0.0f;  // deg
volatile float _flow_angle_pitch  = 0.0f;  // deg

// --- CRC8 DVB-S2 ---
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t byte) {
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
        else crc = crc << 1;
    }
    return crc;
}

// --- Tâche FreeRTOS — tourne sur Core 0 à ~100 Hz ---
static void flow_task(void *parameter) {
    (void)parameter;

    // Parser state
    MspState state = MSP_IDLE;
    uint8_t  msp_flag = 0;
    uint16_t msp_func = 0;
    uint16_t msp_size = 0;
    uint16_t msp_idx  = 0;
    uint8_t  msp_crc  = 0;
    uint8_t  msp_payload[16];  // Optic flow payload = 9 bytes max

    // Filtered velocities
    float filt_vel_x = 0.0f;
    float filt_vel_y = 0.0f;

    // Position intégrée (accumulateurs locaux)
    float pos_x_int = 0.0f;
    float pos_y_int = 0.0f;

    // Timing
    unsigned long last_frame_us = micros();

    const float ALPHA = 0.5f;  // Filtre passe-bas coefficient

    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        // Vérifier demande de reset position
        if (flow_reset_pos) {
            pos_x_int = 0.0f;
            pos_y_int = 0.0f;
            filt_vel_x = 0.0f;
            filt_vel_y = 0.0f;
            flow_reset_pos = false;
        }

        // Lire tous les bytes disponibles sur Serial1
        while (Serial1.available()) {
            uint8_t c = Serial1.read();

            switch (state) {
                case MSP_IDLE:
                    if (c == '$') state = MSP_HEADER_X;
                    break;

                case MSP_HEADER_X:
                    state = (c == 'X') ? MSP_HEADER_DIR : MSP_IDLE;
                    break;

                case MSP_HEADER_DIR:
                    if (c == '<' || c == '>') {
                        state = MSP_FLAG;
                    } else {
                        state = MSP_IDLE;
                    }
                    break;

                case MSP_FLAG:
                    msp_flag = c;
                    msp_crc = 0;
                    msp_crc = crc8_dvb_s2(msp_crc, c);
                    state = MSP_FUNC_LO;
                    break;

                case MSP_FUNC_LO:
                    msp_func = c;
                    msp_crc = crc8_dvb_s2(msp_crc, c);
                    state = MSP_FUNC_HI;
                    break;

                case MSP_FUNC_HI:
                    msp_func |= ((uint16_t)c << 8);
                    msp_crc = crc8_dvb_s2(msp_crc, c);
                    state = MSP_SIZE_LO;
                    break;

                case MSP_SIZE_LO:
                    msp_size = c;
                    msp_crc = crc8_dvb_s2(msp_crc, c);
                    state = MSP_SIZE_HI;
                    break;

                case MSP_SIZE_HI:
                    msp_size |= ((uint16_t)c << 8);
                    msp_crc = crc8_dvb_s2(msp_crc, c);
                    msp_idx = 0;
                    if (msp_size > 0 && msp_size <= sizeof(msp_payload)) {
                        state = MSP_PAYLOAD;
                    } else if (msp_size == 0) {
                        state = MSP_CRC;
                    } else {
                        // Payload trop grand, on ignore
                        state = MSP_IDLE;
                    }
                    break;

                case MSP_PAYLOAD:
                    msp_payload[msp_idx++] = c;
                    msp_crc = crc8_dvb_s2(msp_crc, c);
                    if (msp_idx >= msp_size) {
                        state = MSP_CRC;
                    }
                    break;

                case MSP_CRC:
                    if (c == msp_crc && msp_func == MSP2_SENSOR_OPTIC_FLOW && msp_size == 9) {
                        // --- Trame OPTIC_FLOW valide ---
                        uint8_t quality = msp_payload[0];

                        int32_t motion_x = (int32_t)(
                            (uint32_t)msp_payload[1]       |
                            ((uint32_t)msp_payload[2] << 8)  |
                            ((uint32_t)msp_payload[3] << 16) |
                            ((uint32_t)msp_payload[4] << 24)
                        );

                        int32_t motion_y = (int32_t)(
                            (uint32_t)msp_payload[5]       |
                            ((uint32_t)msp_payload[6] << 8)  |
                            ((uint32_t)msp_payload[7] << 16) |
                            ((uint32_t)msp_payload[8] << 24)
                        );

                        // motion_x/y sont déjà des deltas instantanés (Matek 3901-L0X)
                        {
                            int32_t delta_x = motion_x;
                            int32_t delta_y = motion_y;

                            // Calculer dt avec filtre passe-bas lourd (anti-jitter UART)
                            unsigned long now_us = micros();
                            float raw_dt_s = (float)(now_us - last_frame_us) * 1e-6f;
                            last_frame_us = now_us;
                            if (raw_dt_s < 0.001f) raw_dt_s = 0.001f;  // Sécurité
                            if (raw_dt_s > 0.5f) raw_dt_s = 0.5f;

                            static float filtered_dt_s = 0.01f;  // Init ~100 Hz
                            const float DT_ALPHA = 0.05f;        // Filtre très lourd
                            filtered_dt_s += DT_ALPHA * (raw_dt_s - filtered_dt_s);
                            float dt_s = filtered_dt_s;

                            // Lire les valeurs partagées
                            float lidar_dist_raw = _flow_lidar_dist;
                            float yaw_deg        = _flow_angle_yaw;

                            // Filtre passe-bas sur le LiDAR pour rejeter les spikes d'obstacles
                            static float filtered_lidar_for_flow = 0.0f;
                            const float LIDAR_ALPHA = 0.2f;
                            if (filtered_lidar_for_flow < 1.0f) {
                                // Premier appel : initialiser directement
                                filtered_lidar_for_flow = lidar_dist_raw;
                            } else {
                                filtered_lidar_for_flow += LIDAR_ALPHA * (lidar_dist_raw - filtered_lidar_for_flow);
                            }
                            float lidar_dist = filtered_lidar_for_flow;

                            // === COMPENSATION TILT (accumulateurs gyro intégrés à 250 Hz) ===
                            portENTER_CRITICAL(&flow_mux);
                            float accum_roll_deg  = flow_accum_roll;
                            float accum_pitch_deg = flow_accum_pitch;
                            flow_accum_roll  = 0.0f;
                            flow_accum_pitch = 0.0f;
                            portEXIT_CRITICAL(&flow_mux);

                            // Conversion degrés accumulés → radians, avec le multiplicateur de tuning
                            float gyro_comp_x = accum_roll_deg  * 0.01745329f * flow_scale_runtime * flow_gyro_mult_x_rt;
                            float gyro_comp_y = accum_pitch_deg * 0.01745329f * flow_scale_runtime * flow_gyro_mult_y_rt;

                            // Compensation : delta capteur + rotation propre du drone
                            float corrected_delta_x = (float)delta_x + gyro_comp_x;
                            float corrected_delta_y = (float)delta_y + gyro_comp_y;

                            // Convertir en vitesse body frame (cm/s)
                            float vel_x_body = FLOW_SIGN_X * (corrected_delta_y * lidar_dist) / (dt_s * flow_scale_runtime);
                            float vel_y_body = FLOW_SIGN_Y * (corrected_delta_x * lidar_dist) / (dt_s * flow_scale_runtime);
                            
                            // Rejet de spike : vitesse > 200 cm/s physiquement impossible
                            if (fabsf(vel_x_body) > 200.0f) vel_x_body = 0.0f;
                            if (fabsf(vel_y_body) > 200.0f) vel_y_body = 0.0f;

                            // Filtre passe-bas basique
                            filt_vel_x = filt_vel_x + ALPHA * (vel_x_body - filt_vel_x);
                            filt_vel_y = filt_vel_y + ALPHA * (vel_y_body - filt_vel_y);

                            // === NOUVELLE LOGIQUE : GYRO-GATING (Masquage dynamique) ===
                            // _flow_gyro_roll et pitch sont les vitesses de rotation instantanées (deg/s)
                            float abs_gyro_roll = fabsf(_flow_gyro_roll);
                            float abs_gyro_pitch = fabsf(_flow_gyro_pitch);
                            
                            // Seuil : Si le drone tourne à plus de 15 degrés par seconde
                            const float GYRO_MASK_THRESHOLD = 15.0f; 

                            if (abs_gyro_roll > GYRO_MASK_THRESHOLD || abs_gyro_pitch > GYRO_MASK_THRESHOLD) {
                                // 1. Le drone bascule : on tue la vélocité pour empêcher le D du PID de s'affoler
                                filt_vel_x = 0.0f;
                                filt_vel_y = 0.0f;
                                
                                // 2. On n'intègre pas la position. pos_x_int et pos_y_int restent figées.
                            } else {
                                // Le drone est stable (surplace ou translation douce) : Intégration normale
                                if (quality >= FLOW_QUALITY_MIN) {
                                    // Deadband vélocité : ne pas intégrer les petites vitesses résiduelles
                                    float int_vx = (fabsf(filt_vel_x) > 2.0f) ? filt_vel_x : 0.0f;
                                    float int_vy = (fabsf(filt_vel_y) > 2.0f) ? filt_vel_y : 0.0f;
                                    pos_x_int += int_vx * dt_s;
                                    pos_y_int += int_vy * dt_s;
                                }
                            }

                            // Publication thread-safe
                            portENTER_CRITICAL(&flow_mux);
                            flow_vel_x    = filt_vel_x;
                            flow_vel_y    = filt_vel_y;
                            flow_qual     = quality;
                            flow_raw_dx   = (float)delta_x;
                            flow_raw_dy   = (float)delta_y;
                            flow_new_data = true;
                            flow_pos_x    = pos_x_int;
                            flow_pos_y    = pos_y_int;
                            portEXIT_CRITICAL(&flow_mux);
                        }
                    }
                    // Ignorer MSP2_SENSOR_RANGEFINDER (0x1F01) — on utilise le TF-Luna
                    state = MSP_IDLE;
                    break;
            }
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(10));  // ~100 Hz
    }
}

// --- API publiques ---
void optical_flow_set_scale(float scale) {
    if (scale > 0.1f) flow_scale_runtime = scale;
}

void optical_flow_set_gyro_mult(float mx, float my) {
    // MODIFICATION ICI: Suppression du blocage "if > 0", on accepte les valeurs 0 et négatives pour le tuning
    flow_gyro_mult_x_rt = mx;
    flow_gyro_mult_y_rt = my;
}

void optical_flow_init() {
    Serial1.begin(FLOW_BAUD, SERIAL_8N1, PIN_FLOW_RX, PIN_FLOW_TX);
    Serial.println(F("[FLOW] Serial1 initialisé (MSPv2, 115200)"));
}

void optical_flow_start_task() {
    if (flow_task_handle != nullptr) return;

    xTaskCreatePinnedToCore(
        flow_task,
        "opt_flow",
        4096,
        nullptr,
        2,                 // Priorité moyenne
        &flow_task_handle,
        0                  // Core 0 (comme radio / IMU secondaire / lidar)
    );

    Serial.println(F("[FLOW] Tâche démarrée (Core 0, ~100 Hz)"));
}

void optical_flow_reset_position() {
    flow_reset_pos = true;
    // Le reset effectif se fait dans flow_task au prochain cycle
}

void optical_flow_update(DroneState *drone) {
    // Transmettre les valeurs actuelles pour la conversion body→world
    _flow_angle_yaw  = drone->angle_yaw;        // Yaw fusionné (gyro+mag, plus stable)
    _flow_lidar_dist = drone->lidar_distance;   // Altitude compensée (cm)

    _flow_gyro_roll  = drone->gyro_roll_input;   // deg/s
    _flow_gyro_pitch = drone->gyro_pitch_input;   // deg/s
    _flow_angle_roll  = drone->angle_roll;         // deg
    _flow_angle_pitch = drone->angle_pitch;        // deg

    // Mise à jour multiplicateurs gyro depuis DroneState
    optical_flow_set_gyro_mult(drone->flow_gyro_mult_x, drone->flow_gyro_mult_y);

    // Copie thread-safe des données optical flow
    portENTER_CRITICAL(&flow_mux);
    drone->flow_velocity_x = flow_vel_x;
    drone->flow_velocity_y = flow_vel_y;
    drone->flow_quality    = flow_qual;
    drone->flow_raw_x      = flow_raw_dx;
    drone->flow_raw_y      = flow_raw_dy;
    drone->new_flow_data   = flow_new_data;
    flow_new_data          = false;
    // Position intégrée directement depuis l'OF (sans EKF)
    drone->pos_est_x       = flow_pos_x;
    drone->pos_est_y       = flow_pos_y;
    portEXIT_CRITICAL(&flow_mux);
}