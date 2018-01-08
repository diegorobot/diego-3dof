
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define BAUDRATE 115200

//// The following define selects which electronics board you have.
// 1 = Arduino UNO

#define MOTHERBOARD 1

// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins./如果在信号和接地引脚之间直接连接机械端接开关，则需要上拉。
const bool X_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops./设置为true以反转endstop的逻辑
const bool Y_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
const bool Z_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.

#define INVERT_X_DIR false    // for Mendel set to false, for Orca set to true
#define INVERT_Y_DIR true    // for Mendel set to true, for Orca set to false
#define INVERT_Z_DIR false     // for Mendel set to false, for Orca set to true

/////这几个参数是比较容易错的。根据自己机械的类型不同，两个的配置不尽相同。
/////但是原则就是要保证原点应该在打印平台的左下角（原点位置为[0,0]），或右上角（原点位置为[max,max]）。
/////只有这样打印出来的模型才是正确的，否则会是某个轴的镜像而造成模型方位不对。
/////修改对应某个轴的配置（true或false）后，电机会反向


#define min_software_endstops false //If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops false  //If true, axis won't move to coordinates greater than the defined lengths below.

////这几个参数是配置打印尺寸的重要参数。这里需要说明的是坐标原点并不是打印中心，真正的打印中心一般在[(x.max-x.min)/2,(y.max-y.min)/2]的位置。
////中心位置的坐标需要在后面的切片工具中使用到，打印中心坐标应该与这里的参数配置匹配，否则很可能会打印到平台以外。
// Travel limits after homing/归位后的行程限制
#define X_MAX_POS 150
#define X_MIN_POS 0
#define Y_MAX_POS 150
#define Y_MIN_POS 0
#define Z_MAX_POS 146
#define Z_MIN_POS 0

//// MOVEMENT SETTINGS
#define NUM_AXIS 3 // The axis order in all axis related arrays is X, Y, Z,

// default settings/默认设置

#define DEFAULT_AXIS_STEPS_PER_UNIT   {78,78,800}  // default steps per unit for ultimaker/ultimaker的每个单位的默认步长
#define DEFAULT_MAX_FEEDRATE          {200, 200, 5}    // (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {800,800,100}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.

#define DEFAULT_ACCELERATION          3000    // X, Y, Z and E max acceleration in mm/s^2 for printing moves/默认打印加速度
#define DEFAULT_RETRACT_ACCELERATION  3000   // X, Y, Z and E max acceleration in mm/s^2 for r retracts/默认回抽加速度

//该配置为加速度变化率。该值过大同样也会导致电机丢步。
//当该值处于合理范围内的较小值时，打印动作将更平滑、打印机机械应力将更小、材料在换向时将有更好的附着力、打印噪声也将降低；当该值处于合理范围内的较大值时，打印时间将缩短。建议谨慎修改该值，最好不
#define DEFAULT_XYJERK                20.0    // (mm/sec)
#define DEFAULT_ZJERK                 0.5     // (mm/sec)

////DIEGO 3DOF SETTINGS
#define DEFAULT_PI 3.1415926

#define DEFAULT_OA_LENGTH_MM 119.365
#define DEFAULT_AB_LENGTH_MM 155.0
#define DEFAULT_BC_LENGTH_MM 180.0
#define DEFAULT_THETA_DEGREE 1.02463
#define DEFAULT_GAMMA_DEGREE 0.383972435
#define DEFAULT_BETA_DEGREE 0
#define DEFAULT_CD_LENGTH_MM 48.8
#define DEFAULT_DE_LENGTH_MM 15.0

#define DEFAULT_LENGTH_PER_LINE_MM 0.1

#define DEFAULT_STEPS_PER_CYCLE_AXIS_1 200
#define DEFAULT_STEPS_PER_CYCLE_AXIS_2 200
#define DEFAULT_STEPS_PER_CYCLE_AXIS_3 200

#define DEFAULT_REDUCTION_RATE_AXIS_1 10
#define DEFAULT_REDUCTION_RATE_AXIS_2 10
#define DEFAULT_REDUCTION_RATE_AXIS_3 10

#define DEFAULT_MICROSTEPS_AXIS_1 8
#define DEFAULT_MICROSTEPS_AXIS_2 8
#define DEFAULT_MICROSTEPS_AXIS_3 8

#define DEFAULT_POINT_X_MM 0
#define DEFAULT_POINT_Y_MM (67+62+DEFAULT_CD_LENGTH_MM)
#define DEFAULT_POINT_Z_MM (102-DEFAULT_DE_LENGTH_MM)

#define DEFAULT_POINT_X_STEPS 0
#define DEFAULT_POINT_Y_STEPS (-1869)
#define DEFAULT_POINT_Z_STEPS  (DEFAULT_GAMMA_DEGREE/(2*PI / (DEFAULT_STEPS_PER_CYCLE_AXIS_3 * DEFAULT_REDUCTION_RATE_AXIS_3 * DEFAULT_MICROSTEPS_AXIS_3)))

#define DEFAULT_RANGE_AXIS_1_MAX_DEGREE 260
#define DEFAULT_RANGE_AXIS_1_MIN_DEGREE -80
#define DEFAULT_RANGE_AXIS_2_MAX_DEGREE 70
#define DEFAULT_RANGE_AXIS_2_MIN_DEGREE -30
#define DEFAULT_RANGE_AXIS_3_MAX_DEGREE 150
#define DEFAULT_RANGE_AXIS_3_MIN_DEGREE 40

#include "Configuration_adv.h"

#endif //__CONFIGURATION_H
