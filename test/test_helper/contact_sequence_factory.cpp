#include "contact_sequence_factory.hpp"

#include "robotoc/planner/discrete_event.hpp"


namespace robotoc {
namespace testhelper {

std::shared_ptr<ContactSequence> CreateContactSequence(
    const Robot& robot, const int N, const int max_num_impact, 
    const double t0, const double event_period) {
  if (robot.maxNumContacts() > 0) {
    std::vector<DiscreteEvent> discrete_events;
    std::vector<double> event_times;
    ContactStatus pre_contact_status = robot.createContactStatus();
    pre_contact_status.setRandom();
    auto contact_sequence = std::make_shared<ContactSequence>(robot, max_num_impact);
    contact_sequence->init(pre_contact_status);
    ContactStatus post_contact_status = pre_contact_status;
    std::random_device rnd;
    for (int i=0; i<max_num_impact; ++i) {
      DiscreteEvent tmp(pre_contact_status, post_contact_status);
      while (!tmp.existDiscreteEvent()) {
        post_contact_status.setRandom();
        tmp.setDiscreteEvent(pre_contact_status, post_contact_status);
      }
      discrete_events.push_back(tmp);
      const double event_time = t0 + i * event_period + 0.1 * event_period * std::abs(Eigen::VectorXd::Random(1)[0]);
      event_times.push_back(event_time);
      pre_contact_status = post_contact_status;
    }
    for (int i=0; i<max_num_impact; ++i) {
      srand((unsigned int) time(0));
      std::random_device rnd;
      const bool sto = (rnd()%2==0);
      contact_sequence->push_back(discrete_events[i], event_times[i], sto);
    }
    return contact_sequence;
  }
  else {
    return std::make_shared<ContactSequence>(robot, N);
  }
}

} // namespace testhelper
} // namespace robotoc