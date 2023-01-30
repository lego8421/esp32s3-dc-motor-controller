idf_component_register(
    SRCS
        ${PROJECT_DIR}/main/app_main.cpp
        ${PROJECT_DIR}/main/at_handlers.cpp
        # motor
        ${PROJECT_DIR}/main/motor/motor_control.c
        ${PROJECT_DIR}/main/motor/pid_control.c
        ${PROJECT_DIR}/main/motor/timer_interrupt.c
        # test
        ${PROJECT_DIR}/main/app_loadcell.cpp
    INCLUDE_DIRS
        .
        ./motor
)
