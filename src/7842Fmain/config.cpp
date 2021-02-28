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
  auto topRight = std::make_shared<Motor>(-5); // top right
  auto bottomRight = std::make_shared<Motor>(-6); // bottom right
  auto bottomLeft = std::make_shared<Motor>(2); // bottom left

  topLeft->setBrakeMode(AbstractMotor::brakeMode::brake);
  topRight->setBrakeMode(AbstractMotor::brakeMode::brake);
  bottomRight->setBrakeMode(AbstractMotor::brakeMode::brake);
  bottomLeft->setBrakeMode(AbstractMotor::brakeMode::brake);

  _model = std::make_shared<XDriveModel>(
    // motors
    topLeft, // top left
    topRight, // top right
    bottomRight, // bottom right
    bottomLeft, // bottom left
    std::make_shared<IntegratedEncoder>(11), std::make_shared<IntegratedEncoder>(-5),
    // limits
    200, 12000);

  ChassisScales scales({3.25_in, 16_in}, 360);
  Limits limits(scales, 200_rpm, 0.9_s, 1, 1);
  _chassis = std::make_shared<XVisionGenerator>(_model, 200_rpm, scales, limits, 10_ms);
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

  _vision = std::make_shared<VisionTask>(std::make_shared<Vision::Vision>(3),
                                         _screen->makePage<GUI::VisionPage>("Vision"));

  _roller = std::make_shared<Roller>(
    std::make_shared<MotorGroup>(MotorGroup {-10, 12}), std::make_shared<Motor>(7),
    std::make_shared<Motor>(-19), std::make_shared<OpticalSensor>(9),
    std::make_shared<OpticalSensor>(15), _screen->makePage<GUI::Graph>("Roller"));

  _imu = std::make_shared<IMU>(4);
  _imu->calibrate();
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
