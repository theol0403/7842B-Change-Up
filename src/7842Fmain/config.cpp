#include "config.hpp"
#include "auton.hpp"

/***
 *     _____ _                   _     
 *    /  __ \ |                 (_)    
 *    | /  \/ |__   __ _ ___ ___ _ ___ 
 *    | |   | '_ \ / _` / __/ __| / __|
 *    | \__/\ | | | (_| \__ \__ \ \__ \
 *     \____/_| |_|\__,_|___/___/_|___/                                   
 */
void Robot::_initializeChassis() {
  _model = std::make_shared<ThreeEncoderXDriveModel>(
    // motors
    std::make_shared<Motor>(8), // top left
    std::make_shared<Motor>(-3), // top right
    std::make_shared<Motor>(-9), // bottom right
    std::make_shared<Motor>(6), // bottom left
    // sensors
    std::make_shared<ADIEncoder>(1, 2, true), //
    std::make_shared<ADIEncoder>(5, 6, true), //
    std::make_shared<ADIEncoder>(3, 4, true), //
    // limits
    200, 12000);

  ChassisScales scales({2.75_in, 11.39_in, 0_in, 2.75_in}, 360);

  _odom = std::make_shared<CustomOdometry>(_model, scales, TimeUtilFactory().create());
  _odom->startTask("Odometry");

  _controller = std::make_shared<OdomXController>(
    _model, _odom,
    //Distance PID - To mm
    std::make_unique<IterativePosPIDController>(
      0.0165, 0.00026, 0.00033, 0, TimeUtilFactory::withSettledUtilParams(10, 5, 150_ms)),
    //Turn PID - To Degree
    std::make_unique<IterativePosPIDController>(
      0.045, 0.002, 0.0006, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 100_ms)),
    //Angle PID - To Degree
    std::make_unique<IterativePosPIDController>(
      0.043, 0, 0, 0, TimeUtilFactory::withSettledUtilParams(2, 1, 150_ms)),
    0_ft, TimeUtilFactory().create());

  Settler::setDefaultAbort(TimeUtilFactory::withSettledUtilParams(1, 1, 1200_ms));

  _follower =
    std::make_shared<PathFollower>(_model, _odom, ChassisScales {{2.75_in, 12_in}, imev5GreenTPR},
                                   0.8_ft, 0_ft, TimeUtilFactory().create());
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

  _rollers = std::make_shared<MotorGroup>(MotorGroup {4, -2}); // left, right rollers
  _tipper = std::make_shared<Motor>(1); // tipper
  _arm = std::make_shared<Motor>(20); // arm

  // set the brake modes of the devices
  _arm->setBrakeMode(AbstractMotor::brakeMode::hold);
  _tipper->setBrakeMode(AbstractMotor::brakeMode::brake);
}

/***
 *     _____                          
 *    /  ___|                         
 *    \ `--.  ___ _ __ ___  ___ _ __  
 *     `--. \/ __| '__/ _ \/ _ \ '_ \ 
 *    /\__/ / (__| | |  __/  __/ | | |
 *    \____/ \___|_|  \___|\___|_| |_|                              
 */
void Robot::_initializeScreen() {
  _screen = std::make_shared<GUI::Screen>(lv_scr_act(), LV_COLOR_ORANGE);
  _screen->startTask("Screen");

  using sides = SideController::sides;

  _selector = static_cast<GUI::Selector*>( //
    &_screen->makePage<GUI::Selector>("Auton")
       .button("None", []() {})
       .newRow()
       .button("TestRed", []() { runAuton(testAuton, sides::red); })
       .button("TestBlue", []() { runAuton(testAuton, sides::blue); })
       .build());

  _screen->makePage<GUI::Actions>("Actions")
    .button("Calibrate", [&]() {})
    .button("Systems Off", [&]() {})
    .newRow()
    .button("Deploy", [&]() { Robot::get().deploy(); })
    .button("Autonomous", [&]() { autonomous(); })
    .build();

  _screen->makePage<GUI::Odom>("Odom").attachOdom(_odom).attachResetter([&]() { _odom->reset(); });
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
  instance._initializeScreen();
  return instance;
}

#define getDevice(x)                                                                               \
  auto& instance = get();                                                                          \
  if (!instance._##x) throw std::runtime_error("Robot::" #x ": _" #x " is null");                  \
  return instance._##x;

std::shared_ptr<ThreeEncoderXDriveModel> Robot::model() {
  getDevice(model);
}

std::shared_ptr<CustomOdometry> Robot::odom() {
  getDevice(odom);
}

std::shared_ptr<OdomXController> Robot::chassis() {
  getDevice(controller);
}

std::shared_ptr<PathFollower> Robot::follower() {
  getDevice(follower);
}

std::shared_ptr<AbstractMotor> Robot::rollers() {
  getDevice(rollers);
}

std::shared_ptr<AbstractMotor> Robot::tipper() {
  getDevice(tipper);
}

std::shared_ptr<AbstractMotor> Robot::arm() {
  getDevice(arm);
}

GUI::Selector* Robot::selector() {
  getDevice(selector);
}

void Robot::deploy() {
  Timer timer;
  timer.placeMark();
  while (timer.getDtFromMark() < 1.4_s) {
    rollers()->moveVoltage(-12000);
    arm()->moveVoltage(12000);
    pros::delay(1);
  }
  rollers()->moveVoltage(0);
  timer.placeMark();
  while (timer.getDtFromMark() < 1_s) {
    arm()->moveVoltage(-12000);
    pros::delay(1);
  }
  arm()->moveVoltage(0);
}
