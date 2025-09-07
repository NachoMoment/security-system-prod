idf_component_register(
  SRCS
    "main.cc"
    "main_functions.cc"
    "detection_responder.cc"
    "app_camera_esp.c"
    "recorder.c"
    "record_gate.c"
    "image_provider.cc"
    "app_now_rx.c"
    "person_detect_model_data.cc"
  INCLUDE_DIRS "."
  REQUIRES
    esp_wifi
    esp_event
    esp_netif
    nvs_flash
    esp_timer
    esp32-camera
    fatfs
    sdmmc
    driver
)

# Build time
target_compile_definitions(${COMPONENT_TARGET} PRIVATE
  TFLM_SKIP_CAMERA_INIT=1
  TENSOR_ARENA_BYTES=1048576
  # PERSON_TRIG_PCT=55
  # PERSON_CONSEC_FRAMES=4
  # PERSON_STOP_PCT=35
  # PERSON_STOP_CONSEC_FRAMES=8
)

# For ESP32-S3-EYE module:
target_compile_definitions(${COMPONENT_TARGET} PRIVATE CAMERA_MODEL_ESP32S3_EYE=1)

target_compile_definitions(${COMPONENT_TARGET} PRIVATE XCLK_FREQ_HZ=20000000)

target_compile_definitions(${COMPONENT_TARGET} PRIVATE ESPNOW_CHANNEL=1)


