#include "config.hpp"

/***
 *     _____ _                   _     
 *    /  __ \ |                 (_)    
 *    | /  \/ |__   __ _ ___ ___ _ ___ 
 *    | |   | '_ \ / _` / __/ __| / __|
 *    | \__/\ | | | (_| \__ \__ \ \__ \
 *     \____/_| |_|\__,_|___/___/_|___/
 *                                     
 *                                     
 */
void Robot::_initializeChassis() {
  _model = std::make_shared<ThreeEncoderXDriveModel>(
    // motors
    std::make_shared<Motor>(1), //
    std::make_shared<Motor>(-2), //
    std::make_shared<Motor>(-3), //
    std::make_shared<Motor>(4), //
    // sensors
    std::make_shared<ADIEncoder>(3, 4, true), //
    std::make_shared<ADIEncoder>(5, 6), //
    std::make_shared<ADIEncoder>(1, 2, true), //
    // limits
    200, 12000);

  _odom = std::make_shared<CustomOdometry>(
    _model, ChassisScales({2.75_in, 12.9473263_in, 0.00_in}, 360), TimeUtilFactory().create());
  _odom->startTask("Odometry");

  _controller = std::make_shared<OdomXController>(
    _model, _odom,
    //Distance PID - To mm
    std::make_unique<IterativePosPIDController>(
      0.015, 0.0002, 0.0002, 0, TimeUtilFactory::withSettledUtilParams(10, 10, 100_ms)),
    //Turn PID - To Degree
    std::make_unique<IterativePosPIDController>(
      0.03, 0.00, 0.0003, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 100_ms)),
    //Angle PID - To Degree
    std::make_unique<IterativePosPIDController>(
      0.02, 0, 0, 0, TimeUtilFactory::withSettledUtilParams(4, 2, 100_ms)),
    TimeUtilFactory().create());
}

/***
 *    ______           _               
 *    |  _  \         (_)              
 *    | | | |_____   ___  ___ ___  ___ 
 *    | | | / _ \ \ / / |/ __/ _ \/ __|
 *    | |/ /  __/\ V /| | (_|  __/\__ \
 *    |___/ \___| \_/ |_|\___\___||___/
 *                                     
 *                                     
 */
void Robot::_initializeDevices() {
  _lift = std::make_shared<Lift>(
    std::make_shared<Motor>(9), std::make_shared<Motor>(-10), //
    std::make_shared<Potentiometer>('g'), std::make_shared<Potentiometer>('h'),
    std::make_shared<IterativePosPIDController>(
      0.0025, 0.00, 0.00005, 0.01, TimeUtilFactory().create()),
    std::make_shared<IterativePosPIDController>(
      0.0025, 0.00, 0.00005, 0.01, TimeUtilFactory().create()));

  _claw = std::make_shared<Claw>(std::make_shared<Motor>(7));
}

/***
 *     _____                          
 *    /  ___|                         
 *    \ `--.  ___ _ __ ___  ___ _ __  
 *     `--. \/ __| '__/ _ \/ _ \ '_ \ 
 *    /\__/ / (__| | |  __/  __/ | | |
 *    \____/ \___|_|  \___|\___|_| |_|
 *                                    
 *                                    
 */
void Robot::_initializeScreen() {
  _screen = std::make_shared<GUI::Screen>(lv_scr_act(), LV_COLOR_ORANGE);

  _screen->makePage<GUI::Odom>().attachOdom(_odom).attachResetter([&]() {
    _odom->reset();
  });

  _screen->startTask("Screen");

  _screen->makePage<GUI::Graph>("Lift")
    .withRange(-250, 900)
    .withResolution(50)
    .withSeries(
      "Left Lift Power", LV_COLOR_RED,
      [&]() {
        return _lift->getLeftMotor()->getCurrentDraw();
      })
    .withSeries(
      "Right Lift Power", LV_COLOR_GREEN,
      [&]() {
        return _lift->getRightMotor()->getCurrentDraw();
      })
    .withSeries(
      "Left Lift Temp", LV_COLOR_MAROON,
      [&]() {
        return _lift->getLeftMotor()->getTemperature() * 15;
      })
    .withSeries("Right Lift Temp", LV_COLOR_TEAL, [&]() {
      return _lift->getLeftMotor()->getTemperature() * 15;
    });

  _screen->makePage<GUI::Actions>("Actions")
    .button(
      "Calibrate Lift",
      [&]() {
        _lift->setState(liftStates::calibrate);
      })
    .button(
      "Systems Off",
      [&]() {
        _lift->setState(liftStates::off);
      })
    .newRow()
    .button(
      "Deploy",
      [&]() {
        Robot::get().deploy();
      })
    .button(
      "Autonomous",
      [&]() {
        autonomous();
      })
    .build();
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

std::shared_ptr<Lift> Robot::lift() {
  getDevice(lift);
}

std::shared_ptr<Claw> Robot::claw() {
  getDevice(claw);
}

GUI::Selector* Robot::selector() {
  getDevice(selector);
}

void Robot::deploy() {}
