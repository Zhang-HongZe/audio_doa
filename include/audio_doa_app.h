#pragma once

#include "audio_doa.h"
#include "audio_doa_tracker.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#define AUDIO_DOA_APP_BUFFER_MAX_SIZE_EACH_CHANNEL (1024)

typedef struct {
    audio_doa_tracker_result_callback_t doa_tracker_result_callback;
    void *doa_tracker_result_callback_ctx;
} audio_doa_app_config_t;

typedef struct {
    audio_doa_handle_t doa_handle;
    audio_doa_tracker_handle_t doa_tracker_handle;
    int32_t last_callback_time;
    struct {
        bool vad_detect : 1;
    }flags;
} audio_doa_app_t;

/**
 * @brief  Create a new audio DOA app instance
 * 
 * @param app 
 * @param config 
 * @return esp_err_t 
 */
esp_err_t audio_doa_app_create(audio_doa_app_t *app, audio_doa_app_config_t *config);

/**
 * @brief  Start the audio DOA app
 * 
 * @param app 
 * @return esp_err_t 
 */
esp_err_t audio_doa_app_start(audio_doa_app_t *app);

/**
 * @brief  Stop the audio DOA app
 * 
 * @param app 
 * @return esp_err_t 
 */
esp_err_t audio_doa_app_stop(audio_doa_app_t *app);

/**
 * @brief  Destroy the audio DOA app instance
 * 
 * @param app 
 * @return esp_err_t 
 */
esp_err_t audio_doa_app_destroy(audio_doa_app_t *app);

/**
 * @brief  Write audio data to the audio DOA app
 * 
 * @param app 
 * @param data 
 * @param bytes_size 
 * @return esp_err_t
 */
esp_err_t audio_doa_app_data_write(audio_doa_app_t *app, uint8_t *data, int bytes_size);

/**
 * @brief  Set the VAD detect flag
 * 
 * @param app 
 * @param vad_detect 
 * @return esp_err_t
 */
esp_err_t audio_doa_app_set_vad_detect(audio_doa_app_t *app, bool vad_detect);

#ifdef __cplusplus
}
#endif  /* __cplusplus */