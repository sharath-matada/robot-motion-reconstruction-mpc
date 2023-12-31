#ifndef ROBOTOC_TEST_HELPER_CONTACT_SEQUENCE_FACTORY_HPP_
#define ROBOTOC_TEST_HELPER_CONTACT_SEQUENCE_FACTORY_HPP_

#include <memory>

#include "robotoc/robot/robot.hpp"
#include "robotoc/planner/contact_sequence.hpp"


namespace robotoc {
namespace testhelper {

std::shared_ptr<ContactSequence> CreateContactSequence(
    const Robot& robot, const int N, const int max_num_impact,
    const double t0, const double event_period);

} // namespace testhelper
} // namespace robotoc

#endif // ROBOTOC_TEST_HELPER_CONTACT_SEQUENCE_FACTORY_HPP_ 