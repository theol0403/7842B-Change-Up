#include "7842Fmain/subsystem/vision.hpp"
#include <memory>

VisionTask::VisionTask(const std::shared_ptr<Vision::Vision>& ivision,
                       const std::shared_ptr<GUI::VisionPage>& idrawer) :
  vision(ivision), drawer(idrawer) {}

const int RED = 0;
const int BLUE = 1;

void VisionTask::initialize() {
  pros::vision_signature_s_t RED_SIG =
    pros::Vision::signature_from_utility(RED, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
  vision->set_signature(RED, &RED_SIG);

  pros::vision_signature_s_t BLUE_SIG =
    pros::Vision::signature_from_utility(BLUE, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
  vision->set_signature(BLUE, &BLUE_SIG);
}

void VisionTask::loop() {
  while (true) {
    auto container = vision->getAll();

    drawer->clear();
    drawer->makeLayer().withColor(LV_COLOR_RED, RED).withColor(LV_COLOR_BLUE, BLUE).draw(container);
    // container->remove(Vision::Query::area, std::less<double>(), 200);
    // drawer->clear().makeLayer().withColor(LV_COLOR_RED, 1).withColor(LV_COLOR_YELLOW, 2).draw(container);
  }
}
