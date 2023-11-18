cmake_minimum_required(VERSION 3.12.4)

idf_component_register(
    SRCS "rotary_encoder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES 
        driver 
        esp_event
)
