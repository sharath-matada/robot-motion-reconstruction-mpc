#include "solution_factory.hpp"



namespace robotoc {
namespace testhelper {

Solution CreateSolution(const Robot& robot, const int N) {
  Solution s(N+1, SplitSolution(robot));
  for (int i=0; i<=N; ++i) {
    s[i].setRandom(robot);
  }
  return s;
}


Solution CreateSolution(const Robot& robot,  
                        const std::shared_ptr<ContactSequence>& contact_sequence, 
                        const TimeDiscretization& time_discretization) {
  if (robot.maxNumContacts() == 0) {
    return CreateSolution(robot, time_discretization.N());
  }
  else {
    Solution s(time_discretization.size(), SplitSolution(robot));
    for (int i=0; i<time_discretization.size()-1; ++i) {
      const auto& grid = time_discretization[i];
      if (grid.type == GridType::Impact) {
        s[i].setContactStatus(contact_sequence->impactStatus(grid.impact_index));
        s[i].setRandom(robot);
      }
      else {
        s[i].setContactStatus(contact_sequence->contactStatus(grid.phase));
        if (grid.switching_constraint) {
          s[i].setSwitchingConstraintDimension(contact_sequence->impactStatus(grid.impact_index+1).dimf());
        }
        else {
          s[i].setSwitchingConstraintDimension(0);
        }
        s[i].setRandom(robot);
      }
    }
    s[time_discretization.size()-1].setRandom(robot);
    return s;
  }
}

} // namespace testhelper
} // namespace robotoc