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
  pros::vision_signature_s_t RED_SIG =
    pros::Vision::signature_from_utility(RED, 6273, 8583, 7428, -615, 291, -162, 2.500, 0);
  vision->set_signature(RED, &RED_SIG);

  pros::vision_signature_s_t BLUE_SIG =
    pros::Vision::signature_from_utility(BLUE, -2371, -1423, -1896, 6257, 10385, 8322, 2.400, 0);
  vision->set_signature(BLUE, &BLUE_SIG);
}

void VisionTask::loop() {
  while (true) {
    auto container = vision->getAll().sort(Vision::Query::area);
    container.remove(Vision::Query::area, std::less<double>(), 2000);

    // std::cout << "Area: " << container.get(0, Vision::Query::area) << std::endl;

    drawer->clear();
    drawer->makeLayer().withColor(LV_COLOR_RED, RED).withColor(LV_COLOR_BLUE, BLUE).draw(container);

    auto reds = container;
    offset = reds.remove(Vision::Query::sig, std::not_equal_to<double>(), RED)
               .get(0, Vision::Query::offsetCenterX);

    auto blues = container;
    blueOffset = blues.remove(Vision::Query::sig, std::not_equal_to<double>(), BLUE)
                   .get(0, Vision::Query::offsetCenterX);

    pros::delay(10);
  }
}

double VisionTask::getOffset() const {
  return offset - 10;
}

double VisionTask::getBlueOffset() const {
  return blueOffset - 10;
}