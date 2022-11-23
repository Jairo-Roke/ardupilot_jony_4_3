//
// Simple test for the AP_InertialSensor driver.
//
/*
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static AP_InertialSensor ins;

static void display_offsets_and_scaling();
static void run_test();

// board specific config
static AP_BoardConfig BoardConfig;

void setup(void);
void loop(void);

void setup(void)
{
    // setup any board specific drivers
    BoardConfig.init();

    hal.console->printf("AP_InertialSensor startup...\n");

    ins.init(100);

    // display initial values
    display_offsets_and_scaling();

    // display number of detected accels/gyros
    hal.console->printf("\n");
    hal.console->printf("Number of detected accels : %u\n", ins.get_accel_count());
    hal.console->printf("Number of detected gyros  : %u\n\n", ins.get_gyro_count());

    hal.console->printf("Complete. Reading:\n");
}

void loop(void)
{
    int16_t user_input;

    hal.console->printf("\n");
    hal.console->printf("%s\n",
    "Menu:\n"
    "    d) display offsets and scaling\n"
    "    l) level (capture offsets from level)\n"
    "    t) test\n"
    "    r) reboot");

    // wait for user input
    while (!hal.console->available()) {
        hal.scheduler->delay(20);
    }

    // read in user input
    while (hal.console->available()) {
        user_input = hal.console->read();

        if (user_input == 'd' || user_input == 'D') {
            display_offsets_and_scaling();
        }

        if (user_input == 't' || user_input == 'T') {
            run_test();
        }

        if (user_input == 'r' || user_input == 'R') {
            hal.scheduler->reboot(false);
        }
    }
}

static void display_offsets_and_scaling()
{
    const Vector3f &accel_offsets = ins.get_accel_offsets();
    const Vector3f &accel_scale = ins.get_accel_scale();
    const Vector3f &gyro_offsets = ins.get_gyro_offsets();

    // display results
    hal.console->printf("\nAccel Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                        (double)accel_offsets.x,
                        (double)accel_offsets.y,
                        (double)accel_offsets.z);
    hal.console->printf("Accel Scale X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                        (double)accel_scale.x,
                        (double)accel_scale.y,
                        (double)accel_scale.z);
    hal.console->printf("Gyro Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                        (double)gyro_offsets.x,
                        (double)gyro_offsets.y,
                        (double)gyro_offsets.z);
}

static void run_test()
{
    Vector3f accel;
    Vector3f gyro;
    uint8_t counter = 0;
    static uint8_t accel_count = ins.get_accel_count();
    static uint8_t gyro_count = ins.get_gyro_count();
    static uint8_t ins_count = MAX(accel_count, gyro_count);

    // flush any user input
    while (hal.console->available()) {
        hal.console->read();
    }

    // clear out any existing samples from ins
    ins.update();

    // loop as long as user does not press a key
    while (!hal.console->available()) {
        // wait until we have a sample
        ins.wait_for_sample();

        // read samples from ins
        ins.update();

        // print each accel/gyro result every 50 cycles
        if (counter++ % 50 != 0) {
            continue;
        }

        // loop and print each sensor
        for (uint8_t ii = 0; ii < ins_count; ii++) {
            char state;

            if (ii > accel_count - 1) {
                // No accel present
                state = '-';
            } else if (ins.get_accel_health(ii)) {
                // Healthy accel
                state = 'h';
            } else {
                // Accel present but not healthy
                state = 'u';
            }

            accel = ins.get_accel(ii);



            hal.scheduler->delay(1000);
            hal.console->printf("%u - Accel (%c) : X:%6.2f Y:%6.2f Z:%6.2f norm:%5.2f",
                                ii, state, (double)accel.x, (double)accel.y, (double)accel.z,
                                (double)accel.length());

            gyro = ins.get_gyro(ii);

            if (ii > gyro_count - 1) {
                // No gyro present
                state = '-';
            } else if (ins.get_gyro_health(ii)) {
                // Healthy gyro
                state = 'h';
            } else {
                // Gyro present but not healthy
                state = 'u';
            }

            hal.console->printf("   Gyro (%c) : X:%6.2f Y:%6.2f Z:%6.2f\n",
                                state, (double)gyro.x, (double)gyro.y, (double)gyro.z);
            auto temp = ins.get_temperature(ii);
            hal.console->printf("   t:%6.2f\n", (double)temp);
        }
    }

    // clear user input
    while (hal.console->available()) {
        hal.console->read();
    }
}

AP_HAL_MAIN();

*/






////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Modo de prueba...
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Simple test for the AP_InertialSensor driver.
//

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Common/AP_Common.h>


const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static AP_InertialSensor ins;

static void display_offsets_and_scaling();
static void run_test();

// board specific config
static AP_BoardConfig BoardConfig;

void setup(void);
void loop(void);

void setup(void)
{
    // setup any board specific drivers
    BoardConfig.init();

    hal.console->printf("AP_InertialSensor startup...\n");

    ins.init(100);

    // display initial values
    display_offsets_and_scaling();

    // display number of detected accels/gyros
    hal.console->printf("\n");
    hal.console->printf("Number of detected accels : %u\n", ins.get_accel_count());
    hal.console->printf("Number of detected gyros  : %u\n\n", ins.get_gyro_count());

    hal.console->printf("Complete. Reading:\n");



    hal.rcout->enable_ch(0);
    hal.rcout->enable_ch(1);
    hal.rcout->enable_ch(2);
    hal.rcout->enable_ch(3);

}

void loop(void)
{
    run_test();
}

static void display_offsets_and_scaling()
{
    const Vector3f &accel_offsets = ins.get_accel_offsets();
    const Vector3f &accel_scale = ins.get_accel_scale();
    const Vector3f &gyro_offsets = ins.get_gyro_offsets();

    // display results
    hal.console->printf("\nAccel Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                        (double)accel_offsets.x,
                        (double)accel_offsets.y,
                        (double)accel_offsets.z);
    hal.console->printf("Accel Scale X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                        (double)accel_scale.x,
                        (double)accel_scale.y,
                        (double)accel_scale.z);
    hal.console->printf("Gyro Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                        (double)gyro_offsets.x,
                        (double)gyro_offsets.y,
                        (double)gyro_offsets.z);
}

static void run_test()
{
    Vector3f accel;
    Vector3f gyro;
    uint8_t counter = 0;
    static uint8_t accel_count = ins.get_accel_count();
    static uint8_t gyro_count = ins.get_gyro_count();
    static uint8_t ins_count = MAX(accel_count, gyro_count);

    int r_gas, r_roll, r_pitch, r_yaw;

    uint16_t radio[8];

    for (uint8_t i = 0; i < 6; i++)
            radio[i] = hal.rcin->read(i);

           r_gas    = (radio[0]);
           r_roll   = (radio[1]);
           r_pitch  = (radio[2]);
           r_yaw    = (radio[3]);

           r_gas = r_gas + 1;
           r_roll = r_roll + 1;
           r_pitch = r_pitch + 1;
           r_yaw = r_yaw + 1;



    // flush any user input
    while (hal.console->available()) {
        hal.console->read();
    }

    // clear out any existing samples from ins
    ins.update();

    // loop as long as user does not press a key
    while (!hal.console->available()) {
        // wait until we have a sample
        ins.wait_for_sample();

        // read samples from ins
        ins.update();

        // print each accel/gyro result every 50 cycles
        if (counter++ % 50 != 0) {
            continue;
        }


        // loop and print each sensor
        for (uint8_t ii = 0; ii < ins_count; ii++) {

            accel = ins.get_accel(ii);
            gyro = ins.get_gyro(ii);

            //hal.console->printf("\n");
            //hal.console->printf(" %6.2f %6.2f %6.2f  %6.1f %6.2f %6.2f \r\n", (double)accel.x, (double)accel.y, (double)accel.z, (double)gyro.x, (double)gyro.y, (double)gyro.z);

        }

    }




    hal.rcout->write(0, r_gas);




}

AP_HAL_MAIN();



/*

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Programa Principal de prueba...
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Simple test for the AP_InertialSensor driver.
//

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static AP_InertialSensor ins;

//Prototipo de funcion (programa principal)
static void run_test();

// board specific config
static AP_BoardConfig BoardConfig;
static AP_AHRS_DCM ahrs;
static AP_Baro baro;

// create compass object
static Compass compass;
static AP_SerialManager serial_manager;

float heading;
float heading_Deg;

uint32_t timer;

//Prototipos de funciones...
void setup(void);
void loop(void);

void setup(void)
{

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////Inicializacion CENTRAL INERCIAL///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////

    // setup any board specific drivers
    BoardConfig.init();

    hal.console->printf("AP_InertialSensor startup...\n");

    ins.init(100);

    // display number of detected accels/gyros
    hal.console->printf("\n");
    hal.console->printf("Number of detected accels : %u\n", ins.get_accel_count());
    hal.console->printf("Number of detected gyros  : %u\n\n", ins.get_gyro_count());

    hal.console->printf("Complete. Reading:\n");
    ///////////////////////////////////////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////Inicializacion COMPASS///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////

    hal.console->printf("Compass library test\n");

    ahrs.init();
    compass.init();
    hal.console->printf("init done - %u compasses detected\n", compass.get_count());

    // set offsets to account for surrounding interference
    compass.set_and_save_offsets(0, Vector3f(0, 0, 0));
    // set local difference between magnetic north and true north
    compass.set_declination(ToRad(0.0f));

    hal.scheduler->delay(1000);
    timer = AP_HAL::micros();
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void loop(void)
{
    run_test();
}


static void run_test()
{
    //Declaracion de variables locales
    Vector3f accel;
    Vector3f gyro;
    uint8_t counter = 0;
    static const uint8_t compass_count = compass.get_count();


    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //----------------------------- Lectura de mediciones - Central Inercial ------------------------------
    ///////////////////////////////////////////////////////////////////////////////////////////////////////

    // flush any user input
    while (hal.console->available()) {
        hal.console->read();
    }

    // clear out any existing samples from ins
    ins.update();

    // loop as long as user does not press a key
    while (!hal.console->available()) {
        // wait until we have a sample
        ins.wait_for_sample();

        // read samples from ins
        ins.update();

        // print each accel/gyro result every 50 cycles
        if (counter++ % 50 != 0) {
            continue;
        }

        //Obteniendo las mediciones de la central inercial...
        accel = ins.get_accel();
        gyro = ins.get_gyro();
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //---------------------------------- Lectura de mediciones - Compass ----------------------------------
    ///////////////////////////////////////////////////////////////////////////////////////////////////////

    // run read() at 10Hz
    if ((AP_HAL::micros() - timer) > 100000L) {
        timer = AP_HAL::micros();
        compass.read();
        //const uint32_t read_time = AP_HAL::micros() - timer;

        for (uint8_t i = 0; i < compass_count; i++) {

            if (!compass.healthy()) {
                continue;
            }

            Matrix3f dcm_matrix;
            // use roll = 0, pitch = 0 for this example
            dcm_matrix.from_euler(0, 0, 0);
            heading = compass.calculate_heading(dcm_matrix, i);
        }
    }

    //Conversion de radianes a grados...
    heading_Deg = (double)ToDeg(heading);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //---------------------------- Control de Orientacion (Attitude control) ------------------------------
    ///////////////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////////////////////
    //Variables para la parte proporcional
    ref_x = 0;
    ref_y = 0;
    ref_z = 0;

    medicion_actual_x = accel.x;
    medicion_actual_y = accel.y;
    medicion_actual_z = heading_Deg;

    error_x = ref_x - medicion_actual_x;
    error_y = ref_y - medicion_actual_y;
    error_z = ref_z - medicion_actual_z;
    //////////////////////////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////////////////////////////////
    //Variables para la parte derivativa
    ref_velx = 0;
    ref_vely = 0;
    ref_velz = 0;

    medicion_actual_velx = gyro.x;
    medicion_actual_vely = gyro.y;
    medicion_actual_velz = gyro.z;

    error_velx = ref_velx - medicion_actual_velx;
    error_vely = ref_vely - medicion_actual_vely;
    error_velz = ref_velz - medicion_actual_velz;
    //////////////////////////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////////////////////////////////
    //Ley de control PD - orientacion
    tau_x = kp_x * accel.x     + kd_x * gyro.x;  //angulo de roll
    tau_y = kp_y * accel.y     + kd_y * gyro.y;  //angulo de pitch
    tau_z = kp_z * heading_Deg + kd_z * gyro.z;  //angulo yaw
    ////////////////////////////////////////////////////////////////////////////////////////////////////////




}

AP_HAL_MAIN();


*/







