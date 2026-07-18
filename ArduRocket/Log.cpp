#include "ArduRocket.h"

#if HAL_LOGGING_ENABLED

/*
  A rocket flight is about two seconds long and costs a motor, so the log is the
  entire tuning surface. RKT carries what is needed to reconstruct why the fins
  did what they did: the stage, the tilt the controller saw, the dynamic pressure
  it scheduled its gains on, and the fin commands that came out.
 */
struct PACKED log_Rocket {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  stage;
    float    tilt_deg;
    float    up_accel_g;
    float    q_pa;
    float    spin_rate_dps;
    float    fin1;
    float    fin2;
    float    fin3;
    float    fin4;
};

void ArduRocket::Log_Write_Rocket()
{
    if (motors == nullptr || ahrs_view == nullptr) {
        return;
    }

    // Tilt away from vertical. In the view frame that is the angle between the
    // view's "up" and true up, which is what the roll and pitch loops drive to zero.
    const float tilt_deg = degrees(fabsf(ahrs_view->roll)) + degrees(fabsf(ahrs_view->pitch));

    const float *fins = motors->fin_out();

    const struct log_Rocket pkt{
        LOG_PACKET_HEADER_INIT(LOG_ROCKET_MSG),
        time_us       : AP_HAL::micros64(),
        stage         : (uint8_t)stage,
        tilt_deg      : tilt_deg,
        up_accel_g    : g2.rocket.up_accel_g(),
        q_pa          : dynamic_pressure_pa,
        spin_rate_dps : degrees(ahrs_view->get_gyro().z),
        fin1          : fins[0],
        fin2          : fins[1],
        fin3          : fins[2],
        fin4          : fins[3],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void ArduRocket::Log_Write_Attitude()
{
    if (attitude_control == nullptr) {
        return;
    }
    attitude_control->Write_ANG();
}

void ArduRocket::Log_Write_Vehicle_Startup_Messages()
{
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

void ArduRocket::full_rate_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Rocket();
    }
}

void ArduRocket::ten_hz_logging_loop()
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Rocket();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }
}

const struct LogStructure ArduRocket::log_structure[] = {
    LOG_COMMON_STRUCTURES,

    // @LoggerMessage: RKT
    // @Description: Rocket flight stage and fin commands
    // @Field: TimeUS: Time since system startup
    // @Field: Stage: Flight stage
    // @FieldValueEnum: Stage: FlightStage
    // @Field: Tilt: Total tilt away from vertical
    // @Field: UpAcc: Body-up specific force, used for stage detection
    // @Field: q: Dynamic pressure used to schedule the fin gains
    // @Field: Spin: Rotation rate about the airframe long axis
    // @Field: Fin1: Fin 1 deflection command
    // @Field: Fin2: Fin 2 deflection command
    // @Field: Fin3: Fin 3 deflection command
    // @Field: Fin4: Fin 4 deflection command
    { LOG_ROCKET_MSG, sizeof(log_Rocket),
      "RKT", "QBffffffff", "TimeUS,Stage,Tilt,UpAcc,q,Spin,Fin1,Fin2,Fin3,Fin4", "s#dooskkkk", "F---------", true },
};

uint8_t ArduRocket::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

#endif  // HAL_LOGGING_ENABLED
