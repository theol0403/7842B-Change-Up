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
    std::make_shared<Motor>(2), // top left
    std::make_shared<Motor>(-8), // top right
    std::make_shared<Motor>(-3), // bottom right
    std::make_shared<Motor>(6), // bottom left
    // sensors
    std::make_shared<ADIEncoder>(1, 2, true), //
    std::make_shared<ADIEncoder>(5, 6, true), //
    std::make_shared<ADIEncoder>(3, 4, true), //
    // limits
    200, 12000);

  ChassisScales scales({2.75_in, 13.068229_in, 0.5_in, 2.75_in * 1.02}, 360);

  _odom = std::make_shared<CustomOdometry>(_model, scales, TimeUtilFactory().create());
  _odom->startTask("Odometry");

  _controller = std::make_shared<OdomXController>(
    _model, _odom,
    //Distance PID - To mm
    std::make_unique<IterativePosPIDController>(
      0.016, 0.00025, 0.0003, 0, TimeUtilFactory::withSettledUtilParams(10, 10, 150_ms)),
    //Turn PID - To Degree
    std::make_unique<IterativePosPIDController>(
      0.05, 0.002, 0.0006, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 100_ms)),
    //Angle PID - To Degree
    std::make_unique<IterativePosPIDController>(
      0.03, 0, 0, 0, TimeUtilFactory::withSettledUtilParams(5, 2, 100_ms)),
    TimeUtilFactory().create());

  _follower =
    std::make_shared<PathFollower>(_model, _odom, scales, 1_ft, TimeUtilFactory().create());
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

  auto leftLift = std::make_shared<Motor>(14);
  auto rightLift = std::make_shared<Motor>(-12);

  _lift = std::make_shared<Lift>(
    leftLift, rightLift, //
    std::make_shared<IntegratedEncoder>(14), std::make_shared<IntegratedEncoder>(12, true),
    std::make_shared<IterativePosPIDController>(0.03, 0.01, 0.0001, 0.1,
                                                TimeUtilFactory().create()),
    std::make_shared<IterativePosPIDController>(0.03, 0.01, 0.0001, 0.1,
                                                TimeUtilFactory().create()));

  _claw = std::make_shared<Claw>(std::make_shared<Motor>(-13));
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

  _screen->makePage<GUI::Odom>("Odom").attachOdom(_odom).attachResetter([&]() { _odom->reset(); });

  _screen->startTask("Screen");

  _screen->makePage<GUI::Graph>("Lift")
    .withRange(-250, 900)
    .withResolution(50)
    .withSeries("Left Lift Power", LV_COLOR_RED,
                [&]() { return _lift->getLeftMotor()->getCurrentDraw(); })
    .withSeries("Right Lift Power", LV_COLOR_GREEN,
                [&]() { return _lift->getRightMotor()->getCurrentDraw(); });

  _screen->makePage<GUI::Actions>("Actions")
    .button("Calibrate Lift", [&]() { _lift->setState(liftStates::calibrate); })
    .button("Systems Off", [&]() { _lift->setState(liftStates::off); })
    .newRow()
    .button("Deploy", [&]() { Robot::get().deploy(); })
    .button("Autonomous", [&]() { autonomous(); })
    .build();

  using sides = SideController::sides;

  _selector = static_cast<GUI::Selector*>( //
    &_screen->makePage<GUI::Selector>("Auton")
       .button("None", []() {})
       .newRow()
       .button("bigRed", []() { runAuton(bigZone, sides::red); })
       .button("bigBlue", []() { runAuton(bigZone, sides::blue); })
       .newRow()
       .button("bigTowerRed", []() { runAuton(bigZoneTower, sides::red); })
       .button("bigTowerBlue", []() { runAuton(bigZoneTower, sides::blue); })
       .newRow()
       .button("bigCloseTowerRed", []() { runAuton(bigZoneCloseTower, sides::red); })
       .button("bigCloseTowerBlue", []() { runAuton(bigZoneCloseTower, sides::blue); })
       .newRow()
       .button("outerProtectedRed", []() { runAuton(bigPreloadProtected, sides::red); })
       .button("outerProtectedBlue", []() { runAuton(bigPreloadProtected, sides::blue); })
       .newRow()
       .button("TestRed", []() { runAuton(testAuton, sides::red); })
       .button("TestBlue", []() { runAuton(testAuton, sides::blue); })
       .build());
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

std::shared_ptr<Lift> Robot::lift() {
  getDevice(lift);
}

std::shared_ptr<Claw> Robot::claw() {
  getDevice(claw);
}

GUI::Selector* Robot::selector() {
  getDevice(selector);
}

void Robot::deploy() {
  claw()->setState(clawStates::open);
  lift()->setState(liftStates::upMedium);
  pros::delay(300);
  claw()->setState(clawStates::off);
  lift()->setState(liftStates::off);
}
