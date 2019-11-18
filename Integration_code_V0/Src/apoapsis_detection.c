#include "apoapsis_detection.h"

int timeDiff(time_value start, time_value end){
    return (end.tv_sec - start.tv_sec) * 1000 + (end.tv_msec - start.tv_msec);
}

__uint8_t chuteStaticTime(time_value t_liftoff, time_value t_latest){
    if(timeDiff(t_liftoff, t_latest) < 0){
        return 1;
    }else{
        return 0;
    }
}

__uint8_t detectStart(int wire_connection)
{
    if(wire_connection == 0)
    {
        return -1;
    }
    return 0;
}

__uint8_t detectMECO(time_value t_liftoff, double acceleration, time_value t_now){
    // MECO definitely happened after specified time
    if(timeDiff(t_liftoff, t_now) > MECO_LATEST){
        return 1;
    }

    // Circular buffer of accelerations
    static double circ_buffer[MECO_BUFFER_SIZE] = {0};
    static double circ_buffer_mean[MECO_BUFFER_SIZE] = {0};
    static int new_measurement = MECO_BUFFER_SIZE - 1;
    
    // Save measurements in buffer
    new_measurement = (new_measurement + 1) % MECO_BUFFER_SIZE;
    circ_buffer[new_measurement] = acceleration;
    
    // Mean measurements
    double mean = 0;
    for(int i = 0; i < MECO_MEANFILTERSIZE; i++){
        mean += circ_buffer[(new_measurement - i + MECO_BUFFER_SIZE) % MECO_BUFFER_SIZE];
    }
    mean /= MECO_MEANFILTERSIZE;
    circ_buffer_mean[new_measurement] = mean;

    int j;
    for(int i = 0; i < MECO_BUFFER_SIZE - 1; i++){
        j = (new_measurement + i + 1) % MECO_BUFFER_SIZE;
        if(circ_buffer_mean[(j + 1) % MECO_BUFFER_SIZE] - circ_buffer_mean[j] < MECO_ACCEL_DIFF / MECO_MEANFILTERSIZE){
            return 1;
        }
    }
    return 0;
}

__uint8_t tApoapsisFromVelocity(double velocity){
#if SIMULATION
    if(velocity < 0){
        velocity = -velocity;
    }
#endif
    // Circular buffer of accelerations
    static double circ_buffer[VELOCITY_BUFFER_SIZE] = {0};
    static double circ_buffer_mean[VELOCITY_BUFFER_SIZE] = {0};
    static int new_measurement = VELOCITY_BUFFER_SIZE - 1;
    
    // Save measurements in buffer
    new_measurement = (new_measurement + 1) % VELOCITY_BUFFER_SIZE;
    circ_buffer[new_measurement] = velocity;
    
    // Mean measurements
    double mean = 0;
    for(int i = 0; i < VELOCITY_MEANFILTERSIZE; i++){
        mean += circ_buffer[(new_measurement - i + VELOCITY_BUFFER_SIZE) % VELOCITY_BUFFER_SIZE];
    }
    mean /= VELOCITY_MEANFILTERSIZE;
    circ_buffer_mean[new_measurement] = mean;



    int count = 0, j;
    for(int i = 0; i < VELOCITY_BUFFER_SIZE - 1 - VELOCITY_MEANFILTERSIZE; i++){
        j = (new_measurement + i + 1) % VELOCITY_BUFFER_SIZE;

        if(circ_buffer_mean[(j + 1) % VELOCITY_BUFFER_SIZE] - circ_buffer_mean[j] <= 0){ 
            count++;
        }else{
            count--;
        }        
    }
    if(count < -VELOCITY_TRIGGER_THRESHOLD){
        return 1;
    }else{
        return 0;
    }
}

time_value tApoapsisFromVelocityAtMECO(time_value t_MECO, double v_MECO){
    double v_t = v_MECO;
    int delta_t = DELTA_T;
    int t_millisec = 0;
    double C = 1/2.0 * PRESSURE_FOR_COMPUTATIION * MOL_AIR / (GAS_CONSTANT * TEMPERATURE) * ROCKET_AREA * DRAG_CONSTANT_ROCKET / ROCKET_MASS;
    while (v_t > 0){
        v_t = v_t - (G + C * v_t * v_t) * delta_t / 1000.0;
        t_millisec += delta_t;
    }
    time_value t_apoapsis;

    t_apoapsis = t_MECO;
    t_apoapsis.tv_sec += t_millisec / 1000;
    t_apoapsis.tv_msec += (t_millisec % 1000);
    return t_apoapsis;
}

__uint8_t tApoapsisFromPressure(double pressure){
    // Circular buffer of accelerations
    static double circ_buffer[PRESSURE_BUFFER_SIZE] = {0};
    static double circ_buffer_mean[PRESSURE_BUFFER_SIZE] = {0};
    static int new_measurement = PRESSURE_BUFFER_SIZE - 1;
    
    // Save measurements in buffer
    new_measurement = (new_measurement + 1) % PRESSURE_BUFFER_SIZE;
    circ_buffer[new_measurement] = pressure;
    
    // Mean measurements
    double mean = 0;
    for(int i = 0; i < PRESSURE_MEANFILTERSIZE; i++){
        mean += circ_buffer[(new_measurement - i + PRESSURE_BUFFER_SIZE) % PRESSURE_BUFFER_SIZE];
    }
    mean /= PRESSURE_MEANFILTERSIZE;
    circ_buffer_mean[new_measurement] = mean;

    int count = 0, j;
    for(int i = 0; i < PRESSURE_BUFFER_SIZE - 1 - PRESSURE_MEANFILTERSIZE; i++){
        j = (new_measurement + i + 1) % PRESSURE_BUFFER_SIZE;

        if(circ_buffer_mean[(j + 1) % PRESSURE_BUFFER_SIZE] - circ_buffer_mean[j] <= 0){ 
            count++;
        }else{
            count--;
        }        
    }
    if(count < -PRESSURE_TRIGGER_THRESHOLD){
        return 1;
    }else{
        return 0;
    }
}


double computeVelocity(double acceleration, time_value t_now){
    static time_value t_last = {0, 0};
    static double velocity = 0;
    if(t_last.tv_sec == 0 && t_last.tv_msec == 0){
        t_last.tv_sec = t_now.tv_sec;
        t_last.tv_msec = t_now.tv_msec;
    }    

    double t_diff = timeDiff(t_last, t_now) / 1000.0;
    velocity += acceleration * t_diff;

    t_last.tv_sec = t_now.tv_sec;
    t_last.tv_msec = t_now.tv_msec;
    return velocity;
}

__uint8_t triggerParachuteTime(time_value t_apoapsis, time_value t_now){
    t_apoapsis.tv_sec += T_AP_BUFFER / 1000;
    t_apoapsis.tv_msec += (T_AP_BUFFER % 1000);
    int t_diff = timeDiff(t_apoapsis, t_now);
    if(t_diff > 0){
        return 1;
    }
    return 0;
}

__uint8_t triggerParachute(__uint8_t parachuteTriggers[4], time_value t_arm, time_value t_latest, time_value t_now){
    int count = 0;
    int t_diff_arm = timeDiff(t_arm, t_now);
    static int t_arm_end = -1;
    if(t_arm_end == -1){
        t_arm_end = timeDiff(t_arm, t_latest);
    }

    // Separate the time into 4 steps
    for(int i = 0; i < 4; i++){
        if(t_arm_end / 4 * (i+1) > t_diff_arm){
            count++;
        } 
    }

    for(int i = 0; i < 4; i++){
        if(parachuteTriggers[i]){
            count--;
        }
    }

    if(count <= 0){
        return 1;
    }
    return 0;
}


//Detect apogee. Returns 8 bits: MSB->LSB:
//  7: Parachute trigger | 6: AP detected by dP | 5: AP detected by dV | 4: AP predicted with v_MECO_acc | 3: AP predicted with v_MECO_pito
//  2: Parachute armed   | 1: MECO detected
//Inputs: double acceleration (m/s^2), double velocity_pito (m/s), double pressure (Pa), uint8 groundConnection (1 = connected, 0 = broken)
__uint8_t detectEventsAndTriggerParachute(double acceleration, double velocity_pito, double pressure, __uint8_t ground_connection, time_value t_now){
    static time_value t_liftoff, t_MECO, t_apoapsis_velocity_static_pito, 
            t_apoapsis_velocity_static_acc, t_apoapsis_pressure, t_apoapsis_velocity,
            t_parachute_arm, t_parachute_latest;
    static __uint8_t parachute_deployed = 0, parachute_armed = 0, MECO = 0, liftOff = 0,
            pressure_AP = 0, velocity_AP = 0, parachute_triggers[4] = {0},
            velocity_AP_MECO_acc = 0, velocity_AP_MECO_pito = 0;

    
    __uint8_t triggers = 0;

    detectMECO(t_liftoff, acceleration, t_now);
    // If not started, we don't care about the rest
    if(!liftOff){
        liftOff = detectStart(ground_connection);
        if(liftOff){
            t_liftoff.tv_sec = t_now.tv_sec;
            t_liftoff.tv_msec = t_now.tv_msec;

            t_parachute_latest.tv_sec = t_liftoff.tv_sec + PARACHUTE_LATEST / 1000;
            t_parachute_latest.tv_msec = t_liftoff.tv_msec + PARACHUTE_LATEST % 1000;
        }
        return 0;
    }
    double velocity_acc = computeVelocity(acceleration, t_now);

    // Detect MECO
    if(!MECO){
        MECO = detectMECO(t_liftoff, acceleration, t_now);
        tApoapsisFromPressure(pressure);
        tApoapsisFromVelocity(velocity_pito);

        if(timeDiff(t_liftoff, t_now) < MECO_EARLIEST){
            MECO = 0;
        }    
        if(MECO){
            t_MECO.tv_sec = t_now.tv_sec;
            t_MECO.tv_msec = t_now.tv_msec;

            // Get velocity at MECO and compute t_apoapsis from this
            double v_MECO_pito = velocity_pito;
            double v_MECO_acc = velocity_acc;
            t_apoapsis_velocity_static_pito = tApoapsisFromVelocityAtMECO(t_MECO, v_MECO_pito);
            t_apoapsis_velocity_static_acc = tApoapsisFromVelocityAtMECO(t_MECO, v_MECO_acc);
        }
        return 0;
    }

    // Arm Parachute
    if(!parachute_armed && timeDiff(t_MECO, t_now) < CHUTE_ARM_TIME){
        tApoapsisFromPressure(pressure);
        tApoapsisFromVelocity(velocity_pito);
        return 0;
    }else{
        if(!parachute_armed){
            parachute_armed = 1;
            t_parachute_arm.tv_sec = t_now.tv_sec;
            t_parachute_arm.tv_msec = t_now.tv_msec;
        }
    }
    
    // Detect apoapsis
    if(!pressure_AP){
        pressure_AP = tApoapsisFromPressure(pressure);
        if(pressure_AP){
            t_apoapsis_pressure.tv_sec = t_now.tv_sec;
            t_apoapsis_pressure.tv_msec = t_now.tv_msec;
        }
    }

    if(!velocity_AP){
        velocity_AP = tApoapsisFromVelocity(velocity_pito);
        if(velocity_AP){
            t_apoapsis_velocity.tv_sec = t_now.tv_sec;
            t_apoapsis_velocity.tv_msec = t_now.tv_msec;
        }
    }

    // Trigger Parachute (t_apoapsis times already have t_buffer in them)
    if(!parachute_triggers[0]){
        // Precomputed t_apoapsis with v_pito at MECO
        parachute_triggers[0] = triggerParachuteTime(t_apoapsis_velocity_static_pito);
    }
    if(!parachute_triggers[1]){
        // Precomputed t_apoapsis with v_acc at MECO
        parachute_triggers[1] = triggerParachuteTime(t_apoapsis_velocity_static_acc);
    }
    if(!parachute_triggers[2] && pressure_AP){
        parachute_triggers[2] = triggerParachuteTime(t_apoapsis_pressure);
    }
    if(!parachute_triggers[3] && velocity_AP){
        parachute_triggers[3] = triggerParachuteTime(t_apoapsis_velocity);
    }
    
    if(!velocity_AP_MECO_acc){
        time_value tmp = t_apoapsis_velocity_static_acc;
        tmp.tv_sec -= T_AP_BUFFER / 1000;
        tmp.tv_msec -= (T_AP_BUFFER % 1000);
        velocity_AP_MECO_acc = triggerParachuteTime(tmp);
    }

    if(!velocity_AP_MECO_pito){
        time_value tmp = t_apoapsis_velocity_static_pito;
        tmp.tv_sec -= T_AP_BUFFER / 1000;
        tmp.tv_msec -= (T_AP_BUFFER % 1000);
        velocity_AP_MECO_pito = triggerParachuteTime(tmp);
    }

    
    // Process parachute triggers
    parachute_deployed = triggerParachute(parachute_triggers, t_parachute_arm, t_parachute_latest);

    // Last resort parachute triggering
    if(!parachute_deployed){
        parachute_deployed = chuteStaticTime(t_liftoff, t_parachute_latest);
    }
    triggers = parachute_deployed << 7;
    triggers += pressure_AP << 6;
    triggers += velocity_AP << 5;
    triggers += velocity_AP_MECO_acc << 4;
    triggers += velocity_AP_MECO_pito << 3;
    triggers += parachute_armed << 2;
    triggers += MECO << 1;

    return triggers;
}