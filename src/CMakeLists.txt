set(crawler_sources
    "main.cpp"
    "crawler.cpp"
    "policy_pipeline.cpp"
    "robot_io.cpp"
    "imu_sensor.cpp"
    "ble_control.cpp"
    "safety.cpp"
    "student_policy.c"
)

set(crawler_requires framework-arduinoespressif32)
if(CONFIG_HTTPD_WS_SUPPORT)
    list(APPEND crawler_sources
        "wifi_telemetry_httpd.cpp"
        "websocket_lifecycle.cpp"
    )
    list(APPEND crawler_requires esp_http_server)
else()
    list(APPEND crawler_sources "wifi_telemetry.cpp")
endif()

idf_component_register(
    SRCS ${crawler_sources}
    INCLUDE_DIRS
        "."
        "../include"
    REQUIRES ${crawler_requires}
)

# ESP-IDF's release setting is -O2. Keep the generated neural-network policy
# and its application call path at the original Arduino build's -O3 level.
# This target option is appended after ESP-IDF's global optimization flag.
target_compile_options(${COMPONENT_LIB} PRIVATE -O3)
