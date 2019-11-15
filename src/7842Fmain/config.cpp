#include "config.hpp"
#include "auton.hpp"

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

  _odom = std::make_shared<CustomOdometry>(_model, ChassisScales({2.75_in, 13.2_in, 0.00_in}, 360));
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
    //Strafe PID - To mm
    std::make_unique<IterativePosPIDController>(
      0.015, 0.0002, 0.0002, 0, TimeUtilFactory::withSettledUtilParams(10, 10, 100_ms)),
    5_in);
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
    std::make_shared<Motor>(-9), std::make_shared<Motor>(10),
    std::make_shared<IterativePosPIDController>(0.019, 0, 0.0001, 0.3, TimeUtilFactory().create()),
    std::make_shared<IterativePosPIDController>(0.019, 0, 0.0001, 0.3, TimeUtilFactory().create()));

  _clawLeft = std::make_shared<Claw>(
    std::make_shared<Motor>(-11),
    std::make_shared<IterativePosPIDController>(0.008, 0, 0, 0, TimeUtilFactory().create()));

  _clawRight = std::make_shared<Claw>(
    std::make_shared<Motor>(7),
    std::make_shared<IterativePosPIDController>(0.008, 0, 0, 0, TimeUtilFactory().create()));
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
  _screen = std::make_shared<Screen>(lv_scr_act(), LV_COLOR_ORANGE);

  _selector = dynamic_cast<AutonSelector*>(&_screen->makePage<AutonSelector>("Auton")
                                              .button("None", []() {})
                                              .button("FarBlue", farBlueAuton)
                                              .button("FarRed", farRedAuton)
                                              .newRow()
                                              .button("MiddleBlue", middleBlueAuton)
                                              .button("MiddleRed", middleRedAuton)
                                              .build());

  _screen->makePage<OdomDebug>().attachOdom(_odom).attachResetter([&]() {
    _odom->reset();
  });

  _screen->startTask("Screen");

  // _motorWarning = std::make_shared<MotorWarning>();

  // _motorWarning->addMotor(
  //   std::dynamic_pointer_cast<Motor>(_model->getTopLeftMotor()), "TopLeftBase");
  // _motorWarning->addMotor(
  //   std::dynamic_pointer_cast<Motor>(_model->getTopRightMotor()), "TopRightBase");
  // _motorWarning->addMotor(
  //   std::dynamic_pointer_cast<Motor>(_model->getBottomLeftMotor()), "BottomLeftBase");
  // _motorWarning->addMotor(
  //   std::dynamic_pointer_cast<Motor>(_model->getBottomRightMotor()), "BottomRightBase");

  // _motorWarning->addMotor(_lift->getLeftMotor(), "Left Lift");
  // _motorWarning->addMotor(_lift->getRightMotor(), "Right Lift");

  // _motorWarning->addMotor(_clawLeft->getMotor(), "Left Claw");
  // _motorWarning->addMotor(_clawRight->getMotor(), "Right Claw");

  _screen->makePage<Graph>("Lift")
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

  _screen->makePage<ButtonMatrix>("Actions")
    .button(
      "Calibrate Claw",
      [&]() {
        _clawLeft->setState(clawStates::calibrate);
        _clawRight->setState(clawStates::calibrate);
      })
    .button(
      "Calibrate Lift",
      [&]() {
        _lift->setState(liftStates::calibrate);
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

std::shared_ptr<Claw> Robot::clawLeft() {
  getDevice(clawLeft);
}

std::shared_ptr<Claw> Robot::clawRight() {
  getDevice(clawRight);
}

void Robot::deploy() {
  clawLeft()->setState(clawStates::close);
  clawRight()->setState(clawStates::close);
  pros::delay(500);
  clawLeft()->setState(clawStates::off);
  clawRight()->setState(clawStates::off);

  lift()->setPosition({400, 400});
  lift()->setState(liftStates::holdAtPos);
  while (std::abs(lift()->getError()) > 100) {
    pros::delay(20);
  }

  clawLeft()->setState(clawStates::release);
  clawRight()->setState(clawStates::release);
  lift()->setState(liftStates::off);
}