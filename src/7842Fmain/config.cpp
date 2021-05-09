#include "config.hpp"

/***
 *     _____ _                   _     
 *    /  __ \ |                 (_)    
 *    | /  \/ |__   __ _ ___ ___ _ ___ 
 *    | |   | '_ \ / _` / __/ __| / __|
 *    | \__/\ | | | (_| \__ \__ \ \__ \
 *     \____/_| |_|\__,_|___/___/_|___/                                   
 */
void Robot::_initializeChassis() {
  auto topLeft = std::make_shared<Motor>(11); // top left
  auto topRight = std::make_shared<Motor>(-18); // top right
  auto bottomRight = std::make_shared<Motor>(-19); // bottom right
  auto bottomLeft = std::make_shared<Motor>(2); // bottom left

  topLeft->setBrakeMode(AbstractMotor::brakeMode::brake);
  topRight->setBrakeMode(AbstractMotor::brakeMode::brake);
  bottomRight->setBrakeMode(AbstractMotor::brakeMode::brake);
  bottomLeft->setBrakeMode(AbstractMotor::brakeMode::brake);

  _model = std::make_shared<XDriveModel>(
    // motors
    topLeft, topRight, bottomRight, bottomLeft,
    // sensors
    std::make_shared<IntegratedEncoder>(11), std::make_shared<IntegratedEncoder>(-5),
    // limits
    200, 12000);

  ChassisScales scales({3.25_in, 16_in}, 360);
  Limits<> limits(scales, 200_rpm, 0.75_s, 1, 1);
  _chassis = std::make_shared<XGenerator>(_model, 200_rpm, scales, limits, 10_ms);
}

/***
 *    ______           _               
 *    |  _  \         (_)              
 *    | | | |_____   ___  ___ ___  ___ 
 *    | | | / _ \ \ / / |/ __/ _ \/ __|
 *    | |/ /  __/\ V /| | (_|  __/\__ \
 *    |___/ \___| \_/ |_|\___\___||___/                              
 */
void Robot::_initializeDevices() {
  _screen = std::make_shared<GUI::Screen>(lv_scr_act(), LV_COLOR_ORANGE);

  _vision = std::make_shared<VisionTask>(std::make_shared<Vision::Vision>(10),
                                         _screen->makePage<GUI::VisionPage>("Vision"));

  _roller = std::make_shared<Roller>(std::make_shared<MotorGroup>(MotorGroup {-17, 12}),
                                     std::make_shared<Motor>(4), std::make_shared<Motor>(-20),
                                     std::make_shared<OpticalSensor>(6),
                                     std::make_shared<OpticalSensor>(3));

  _imu = std::make_shared<IMUTurn>(std::make_shared<pros::Imu>(5), _model,
                                   std::make_shared<IterativePosPIDController>(
                                     0.03, 0.0025, 0.0003, 0, TimeUtilFactory().create()));
}

/***
 *     _____ _             _      _              
 *    /  ___(_)           | |    | |             
 *    \ `--. _ _ __   __ _| | ___| |_ ___  _ __  
 *     `--. \ | '_ \ / _` | |/ _ \ __/ _ \| '_ \ 
 *    /\__/ / | | | | (_| | |  __/ || (_) | | | |
 *    \____/|_|_| |_|\__, |_|\___|\__\___/|_| |_|
 *                    __/ |                      
 *                   |___/                       
 */
Robot& Robot::get() {
  static Robot instance;
  return instance;
}

Robot& Robot::initialize() {
  auto& instance = get();
  instance._initializeChassis();
  instance._initializeDevices();
  return instance;
}

#define getDevice(x)                                                                               \
  auto& instance = get();                                                                          \
  if (!instance._##x) throw std::runtime_error("Robot::" #x ": _" #x " is null");                  \
  return instance._##x;

#define ADD(name, type)                                                                            \
  std::shared_ptr<type> Robot::name() {                                                            \
    getDevice(name);                                                                               \
  }
#include "systems.def"
#undef ADD
