/*
 * mode_rolquad.cpp
 *
 *  Created on: 3 nov. 2020
 *      Author: usuario
 */
#include "Copter.h"

#include "UserVariables.h"

/*
static void trajectory()
{

}*/

bool ModeRolQuad::init(bool ignore_checks){
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Iniciando_modo_ROLQUAD");//Se ve en msgs en mission planner

    g.estado = 1;
    return true;
}

void ModeRolQuad::run()
{
    const AP_InertialSensor &ins = AP::ins();
    const Vector3f &gyro = ins.get_gyro();

    posxyz = inertial_nav.get_position()*0.01;
    velxyz = inertial_nav.get_velocity()*0.01;


    //Leyendo radio...
    rd.roll  = ((copter.channel_roll->get_radio_in())-1500)/4;
    rd.pitch = ((copter.channel_pitch->get_radio_in())-1500)/4;
    rd.yaw   = ((copter.channel_yaw->get_radio_in())-1500)/4;
    rd.th    = copter.channel_throttle->get_radio_in();


    rd.aux1 = rc().get_radio_in(4); //Canal auxiliar1
    rd.aux2 = rc().get_radio_in(5); //Canal auxiliar2
    rd.aux3 = rc().get_radio_in(6); //Canal auxiliar3


    //Maquina de estados para modo de vuelo

    //Modo1: Estado = 1 = Estabilzado
    //Modo2: Estado = 2 = Altura
    //Modo3: Estado = 3 = Posicion (Hover)

   // if (rd.aux3 > 1500){ ///J// aux3= arm/disarm

    if(rd.th > 1150 && rd.aux1 > 1400 && rd.aux1 < 1600){ ///J///aux1
        //Modo2

        if(flag_spz){
            sp._z = posxyz.z;
            flag_spz = false;
            flag_spxy = true;
        }

        g.estado = 2;

    }else if(rd.aux1 > 1700){
        //Modo3

        if(flag_spxy){
            sp._x = posxyz.x;
            sp._y = posxyz.y;
            flag_spxy = false;
            flag_spz  = true;
        }
        g.estado = 3;

    } else {
        //Modo1
        flag_spz  = true;
        flag_spxy = true;

        g.estado = 1;
    }

   /// }
    switch(g.estado){

        case 1:           //Estabilizado

            ctrl._x = 0;
            ctrl._y = 0;
            ctrl._z = 0;


            break;

        case 2:           //Altura

            //Calculando los errores de posicion...
              errors._z = sp._z - posxyz.z;

              //Control de altura...
              ctrl._x = 0;
              ctrl._y = 0;
              ctrl._z = kp2._z*(errors._z) + (kp2._z*(-velxyz.z));
              break;

        case 3:                         //Posicion (hover full)

            errors._x = posxyz.z - sp._x;
            errors._y = posxyz.z - sp._y;
            errors._z = sp._z - posxyz.z;

            ctrl._x = kp2._x*(errors._x) + kp2._x*( velxyz.x);
            ctrl._y = kp2._y*(errors._y) + kp2._y*( velxyz.y);
            ctrl._z = kp2._z*(errors._z) + kp2._z*(-velxyz.z);

            break;

        default:
            break;

    }



    //Implementaci�n del control de orientaci�n...
    float c_roll, c_pitch, c_yaw;

   // c_roll  = kp1._roll*(ahrs.roll)   + kd1._roll*(gyro.x);
   // c_pitch = kp1._pitch*(ahrs.pitch) + kd1._pitch*(gyro.y);
   // c_yaw   = kp1._yaw*(ahrs.yaw)     + kd1._yaw*(gyro.z);


    c_roll  = 0*(ahrs.roll)  + 0*(gyro.x);
    c_pitch = 0*(ahrs.pitch) + 0*(gyro.y);
    c_yaw   = 0*(ahrs.yaw)   + 0*(gyro.z);

///



    //Inicializando variables para los motores...
    ///float m1, m2, m3, m4; QUADRI

    ///tandem
    float m1, m2, m3, m4;// m5,m6,m7,m8;

    ////////// MIXER cuadri

    m1 = rd.th + c_roll - c_pitch - c_yaw - rd.roll - rd.pitch + rd.yaw; // + ctrl._z;
    m2 = rd.th - c_roll + c_pitch - c_yaw + rd.roll + rd.pitch + rd.yaw; // + ctrl._z;
    m3 = rd.th - c_roll - c_pitch + c_yaw + rd.roll - rd.pitch - rd.yaw; // + ctrl._z;
    m4 = rd.th + c_roll + c_pitch + c_yaw - rd.roll + rd.pitch - rd.yaw; // + ctrl._z;

    //Mixer tandem

   // m1 = rd.th - c_pitch - rd.pitch; // + ctrl._z; + yawFactor
    //m2 = rd.th + c_pitch + rd.pitch; // + ctrl._z;
    //m3 = rd.roll + c_roll; //
    //m4 = rd.pitch + c_pitch; //
   // m5 = rd.yaw + c_yaw; //
   // m6 = rd.yaw + c_yaw;
   // m7 = rd.yaw + c_yaw;
   // m8 = rd.yaw + c_yaw;
    //Escribiendo hacia los motores...

    //if(rd.th > min_throttle + 50)  QUADRI
 //   if(rd.aux3>1500 && rd.th > 1050){
 //      hal.rcout->write(0, (int)m1);
 //      hal.rcout->write(1, (int)m2);
 //      hal.rcout->write(2, (int)m3);
 //      hal.rcout->write(3, (int)m4);

    //if(rd.th > min_throttle + 50)  QUADRI
    if(rd.aux3>1500 && rd.th > 1050){
       hal.rcout->write(0, (int)m1);
       hal.rcout->write(1, (int)m2);
       hal.rcout->write(2, (int)m3);
       hal.rcout->write(3, (int)m4);
     //  hal.rcout->write(4, (int)m5);
     //  hal.rcout->write(5, (int)m6);
     //  hal.rcout->write(6, (int)m7);
     //  hal.rcout->write(7, (int)m8);

    }else{
        //hal.rcout->write(channel,min_throttle); // QUADRI
        //hal.rcout->write(0, 1000);
        //hal.rcout->write(1, 1000);
        //hal.rcout->write(2, 1000);
        //hal.rcout->write(3, 1000);

        hal.rcout->write(0, 1000);
        hal.rcout->write(1, 1000);
        hal.rcout->write(2, 1000);
        hal.rcout->write(3, 1000);
       // hal.rcout->write(4, 1000);
        //hal.rcout->write(5, 1000);
        //hal.rcout->write(6, 1000);
        //hal.rcout->write(7, 1000);
   }
//

    //g.var1 = rd.th;
/////
   //hal.console->printf("%d	 \n", rd.pitch);
   // hal.console->printf("roll: %.2f \n", ahrs.roll);
    //hal.console->printf("Hola \n piuts");
   // hal.scheduler->delay(500);

}

// ATT nos da los angulos roll, pitch y yaw (escalados)

// BAT no da el voltaje, corriente y potencia (no escalados)

// IMU nos da las velocidades angulares en x, y y z (no escalados)
