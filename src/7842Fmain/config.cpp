#include "config.hpp"

Robot& Robot::get() {
  static Robot instance;
  return instance;
}

Robot& Robot::initialize() {
  auto& instance = get();
  instance.initializeChassis();
  instance.initializeDevices();
  instance.initializeScreen();
  return instance;
}

Robot& Robot::update() {}

std::shared_ptr<ThreeEncoderXDriveModel> Robot::getModel() {
  auto& instance = get();
  if (!instance.model) throw std::runtime_error("Robot::getModel: model is null");
  return instance.model;
}

std::shared_ptr<CustomOdometry> Robot::getOdom() {
  auto& instance = get();
  if (!instance.odom) throw std::runtime_error("Robot::getOdom: odom is null");
  return instance.odom;
}

std::shared_ptr<OdomController> Robot::getController() {
  auto& instance = get();
  if (!instance.controller) throw std::runtime_error("Robot::getOdom: controller is null");
  return instance.controller;
}

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
void Robot::initializeChassis() {
  model = std::make_shared<ThreeEncoderXDriveModel>(
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

  odom = std::make_shared<CustomOdometry>(model, ChassisScales({2.75_in, 13.2_in, 0.00_in}, 360));
  odom->startTask("Odometry");

  controller = std::make_shared<OdomXController>(
    model, odom,
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
void Robot::initializeDevices() {}

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
void Robot::initializeScreen() {

  screen = std::make_shared<Screen>(lv_scr_act(), LV_COLOR_ORANGE);

  screen->makePage<OdomDebug>().attachOdom(odom).attachResetter([&]() {
    odom->reset();
  });

  screen->startTask("Screen");
}