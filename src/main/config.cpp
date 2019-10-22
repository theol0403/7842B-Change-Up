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

Robot& Robot::update() {
  auto& instance = get();
  instance.updateOdom();
  instance.updateScreen();
}

std::shared_ptr<ThreeEncXDriveModel> Robot::getModel() {
  auto& instance = get();
  if (!instance.model) throw std::runtime_error("Robot::getModel: model is null");
  return instance.model;
}
std::shared_ptr<ThreeEncoderOdometry> Robot::getOdom() {
  auto& instance = get();
  if (!instance.odom) throw std::runtime_error("Robot::getOdom: odom is null");
  return instance.odom;
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
  model = std::make_shared<ThreeEncXDriveModel>(
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

  odom = std::make_shared<ThreeEncoderOdometry>(
    TimeUtilFactory::create(), model, ChassisScales({2.75_in, 13.2_in, 0.001_in}, 360));
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
  odomScreen = std::make_shared<OdomDebug>(lv_scr_act(), LV_COLOR_ORANGE);

  odomScreen->setStateCallback([&](OdomDebug::state_t state) {
    odom->setState({state.x, state.y, state.theta}, StateMode::CARTESIAN);
  });

  odomScreen->setResetCallback([&]() {
    model->resetSensors();
    odom->setState({0_in, 0_in, 0_deg}, StateMode::CARTESIAN);
  });
}

void Robot::updateOdom() {
  odom->step();
}

void Robot::updateScreen() {
  auto state = odom->getState(StateMode::CARTESIAN);
  auto sensors = model->getSensorVals();
  odomScreen->setData(
    {state.x, state.y, state.theta}, {(double)sensors[0], (double)sensors[1], (double)sensors[2]});
}