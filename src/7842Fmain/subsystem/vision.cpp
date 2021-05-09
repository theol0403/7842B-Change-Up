#include "7842Fmain/subsystem/vision.hpp"

VisionTask::VisionTask(const std::shared_ptr<Vision::Vision>& ivision,
                       const std::shared_ptr<GUI::VisionPage>& idrawer) :
  vision(ivision), drawer(idrawer) {
  pros::delay(100); // allow sensors to initialize
  initialize();
  startTask("Roller");
}

const int RED = 2;
const int BLUE = 1;

void VisionTask::initialize() {
  // pros::vision_signature_s_t RED_SIG =
  //   pros::Vision::signature_from_utility(RED, 6553, 8863, 7708, -477, 115, -182, 4.700, 0);
  // vision->set_signature(RED, &RED_SIG);

  // pros::vision_signature_s_t BLUE_SIG =
  //   pros::Vision::signature_from_utility(BLUE, -2409, -1563, -1986, 7527, 10923, 9224, 3.500, 0);
  // vision->set_signature(BLUE, &BLUE_SIG);

  pros::vision_signature_s_t RED_SIG =
    pros::Vision::signature_from_utility(RED, 7423, 10707, 9065, -1479, -109, -794, 3.700, 0);
  vision->set_signature(RED, &RED_SIG);

  pros::vision_signature_s_t BLUE_SIG =
    pros::Vision::signature_from_utility(BLUE, -2409, -1563, -1986, 7527, 10923, 9224, 3.500, 0);
  vision->set_signature(BLUE, &BLUE_SIG);
}

void VisionTask::loop() {
  Rate r;
  while (true) {
    auto container = vision->getAll().sort(Vision::Query::area);
    container.remove(Vision::Query::area, std::less<double>(), 3000)
      .remove(Vision::Query::area, std::greater<double>(), 34000);

    /* std::cout << "Area: " << container.get(0, Vision::Query::area) << std::endl; */

    drawer->clear();
    drawer->makeLayer()
      .withColor(LV_COLOR_RED, LV_COLOR_BLACK, RED)
      .withColor(LV_COLOR_BLUE, LV_COLOR_BLACK, BLUE)
      .draw(container);

    auto reds = container;
    offset = reds.remove(Vision::Query::sig, std::not_equal_to<double>(), RED)
               .get(0, Vision::Query::offsetCenterX);

    auto blues = container;
    blueOffset = blues.remove(Vision::Query::sig, std::not_equal_to<double>(), BLUE)
                   .get(0, Vision::Query::offsetCenterX);

    r.delayUntil(10_ms);
  }
}

double VisionTask::getOffset() const {
  return offset;
}

double VisionTask::getBlueOffset() const {
  return blueOffset;
}
