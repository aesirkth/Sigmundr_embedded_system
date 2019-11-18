#ifndef DEFINITIONS_H
#define DEFINITIONS_H


#define TIME_STEP               10             // milliseconds

//Physical simulation variables
#define G                       9.81            // Acceleration of earth
#define MOL_AIR                 0.0289644       // Molar mass of air
#define GAS_CONSTANT            8.3144598       // Gas constant R
#define TEMPERATURE             255             // Temperature of the air in kelvin
#define STATIC_PRESSURE         101325          // Static pressure at ground level

//Measurement uncertainties (+-%, e.g. percentage uncertainty in each direction, times 0.1: 10 = 1%)
#define UNC_PRESSURE            10              // Uncertainty of measurements (x10, percent)
#define UNC_ACCELERATION        10
#define UNC_VELOCITY            50
#define CONST_UNC_PRESSURE      10              // Constant offset of measurements (x10, percent)
#define CONST_UNC_ACCELERATION  10
#define CONST_UNC_VELOCITY      10        


//Rocket specifications
#define MOTOR_THRUST_MAX        800             // Maximum thrust reached
#define MOTOR_THRUST_END        310             // Thrust at t_MECO
#define T_THRUST_MAX            T_LIFTOFF + 500 // Time when maximum thrust is achieved
#define UNC_THRUST              10              // Uncertainty in thrust
#define T_ENGINE_BURN           3000            // Engine burn time
#define ROCKET_AREA             0.05*0.05*3.14  // Area of rocket (for drag computation)
#define DRAG_CONSTANT_ROCKET    0.5            // Drag constant of rocket
#define ROCKET_MASS             7.789               // Dry mass of rocket
//#define ROCKET_MASS             6.113               // Dry mass of rocket
#define FUEL_MASS               1.676               // Mass of fuel

#define DRAG_CONSTANT_CHUTE     1.75            // Drag constant parachute
#define CHUTE_AREA              0.7*0.7*3.14    // Area of parachute

//Simulation times (ms)
#define T_LIFTOFF               2000            // Time of true liftoff
#define T_MECO                  T_LIFTOFF + T_ENGINE_BURN   // Time of MECO

#define CHUTE_ARM_TIME          5000            // Time after MECO for arming the parachute
#define PARACHUTE_LATEST        25000           // Latest time when parachute should open (after liftoff)

#define T_AP_BUFFER             2000            // Buffer time after detected AP by any measure for parachute deployment

#define PRESSURE_FOR_COMPUTATIION 0.8 * 101325  // For precomputing t_apoapsis with velocities
#define DELTA_T                 10             // delta_t for precomputing t_apoapsis with velocities

// MECO detection
#define MECO_LATEST             T_ENGINE_BURN + 1000  // Latest time when MECO should have happened
#define MECO_EARLIEST           T_ENGINE_BURN - 1000  // Earliest time when MECO could have happened
#define MECO_BUFFER_SIZE        16              // Circular buffer size
#define MECO_MEANFILTERSIZE     3               // # of measurements taken into account for mean filter
#define MECO_ACCEL_DIFF         -20             // Difference in acceleration to detect MECO

#define PRESSURE_BUFFER_SIZE    16              // Circular buffer size
#define PRESSURE_MEANFILTERSIZE 3               // # of measurements taken into account for mean filter
#define PRESSURE_TRIGGER_THRESHOLD 4            // Amount of negative derivatives to detect AP

#define VELOCITY_BUFFER_SIZE    16              // Circular buffer size
#define VELOCITY_MEANFILTERSIZE 3               // # of measurements taken into account for mean filter
#define VELOCITY_TRIGGER_THRESHOLD 4            // Amount of negative derivatives to detect AP

#define DEBUG_SIMULATION        0               // 1 = print all true simulation values
#define LOGGING                 0
#define SIMULATION              0

#endif