set(COMPONENT_SRCS
    cubic_spline_planner.cpp
    golfcar_control.cpp
    lqr_steer_control.cpp
    model_matrix.cpp
    pid_controller.cpp
)

set(COMPONENT_ADD_INCLUDEDIRS
    .
)
register_component()
