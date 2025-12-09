#include <stdio.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_err.h"
#include "audio_doa_app.h"
#include "audio_doa.h"
#include "audio_doa_tracker.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

static const char *TAG = "audio_doa_app";

static void audio_doa_callback(float angle, void *ctx)
{
    audio_doa_app_t *app = (audio_doa_app_t *)ctx;
    if (app == NULL) {
        ESP_LOGE(TAG, "audio_doa_callback: app is NULL");
        return;
    }
    if (app->doa_tracker_handle == NULL) {
        ESP_LOGE(TAG, "audio_doa_callback: doa_tracker_handle is NULL");
        return;
    }
    audio_doa_tracker_feed(app->doa_tracker_handle, angle);

    ESP_LOGI(TAG, "audio_doa_callback: angle %.2f", angle);

    if (app->audio_doa_monitor_callback != NULL) {
        app->audio_doa_monitor_callback(angle, app->audio_doa_monitor_callback_ctx);
    }
}

esp_err_t audio_doa_app_create(audio_doa_app_handle_t *app, audio_doa_app_config_t *config)
{
    esp_err_t ret = ESP_OK;

    if (app == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *app = (audio_doa_app_handle_t)calloc(1, sizeof(audio_doa_app_t));
    if (*app == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    (*app)->doa_handle = NULL;
    (*app)->doa_tracker_handle = NULL;

    ret = audio_doa_new(&(*app)->doa_handle, NULL);
    if (ret != ESP_OK) {
        return ret;
    }
    (*app)->audio_doa_monitor_callback = config->audio_doa_monitor_callback;
    (*app)->audio_doa_monitor_callback_ctx = (void *)*app;
    audio_doa_set_doa_result_callback((*app)->doa_handle, audio_doa_callback, (void *)*app);

    audio_doa_tracker_cfg_t doa_tracker_cfg = {
        .result_callback = config->audio_doa_result_callback,
        .ctx = *app,
        .output_interval_ms = 1000,
    };
    ret = audio_doa_tracker_init(&doa_tracker_cfg, &(*app)->doa_tracker_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = audio_doa_app_start(*app);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "audio_doa_app_create success");

    return ESP_OK;
}

esp_err_t audio_doa_app_destroy(audio_doa_app_handle_t app)
{
    esp_err_t ret = ESP_OK;
    ret = audio_doa_delete(app->doa_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = audio_doa_tracker_deinit(app->doa_tracker_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    free(app);
    return ESP_OK;
}

esp_err_t audio_doa_app_start(audio_doa_app_handle_t app)
{
    esp_err_t ret = ESP_OK;
    if (app == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ret = audio_doa_start(app->doa_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = audio_doa_tracker_enable(app->doa_tracker_handle, true);
    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

esp_err_t audio_doa_app_stop(audio_doa_app_handle_t app)
{
    esp_err_t ret = ESP_OK;
    if (app == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ret = audio_doa_stop(app->doa_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = audio_doa_tracker_enable(app->doa_tracker_handle, false);
    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

esp_err_t audio_doa_app_data_write(audio_doa_app_handle_t app, uint8_t *data, int bytes_size)
{
    esp_err_t ret = ESP_OK;
    if (app == NULL || data == NULL || bytes_size <= 0) {
        ESP_LOGE(TAG, "audio_doa_app_data_write: invalid args");
        return ESP_ERR_INVALID_ARG;
    }
    if (app->flags.vad_detect == false) {
        return ESP_OK;
    }
    ret = audio_doa_data_write(app->doa_handle, data, bytes_size);

    return ret;
}

esp_err_t audio_doa_app_set_vad_detect(audio_doa_app_handle_t app, bool vad_detect)
{
    if (app == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    app->flags.vad_detect = vad_detect;
    return ESP_OK;
}

#ifdef __cplusplus
}
#endif  /* __cplusplus */