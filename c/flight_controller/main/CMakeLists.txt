idf_component_register(
    SRCS 
        "main.c"
        "i2c_manager.c"
        "mpu6050.c"
        "qmc5883l.c"
        "sensor_fusion.c"
        "json_creator.c"
    INCLUDE_DIRS "."
    REQUIRES 
        driver
        esp_timer
        freertos
)