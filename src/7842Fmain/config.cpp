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
  _model = std::make_shared<XDriveModel>(
    // motors
    std::make_shared<Motor>(11), // top left
    std::make_shared<Motor>(-5), // top right
    std::make_shared<Motor>(-6), // bottom right
    std::make_shared<Motor>(2), // bottom left
    std::make_shared<IntegratedEncoder>(14), std::make_shared<IntegratedEncoder>(2),
    // limits
    200, 12000);
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
  _roller = std::make_shared<Roller>(std::make_shared<MotorGroup>(MotorGroup {-10, 12}),
                                     std::make_shared<Motor>(7), std::make_shared<Motor>(-19),
                                     std::make_shared<pros::ADIAnalogIn>('D'),
                                     std::make_shared<pros::ADIAnalogIn>('B'));
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
