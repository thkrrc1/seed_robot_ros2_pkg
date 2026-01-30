#include "structure_robotstate.hpp"


bool operator ==(const RobotState& lhs,const RobotState&rhs){
    return (lhs.jpos.size() == rhs.jpos.size()) && std::equal(lhs.jpos.begin(), lhs.jpos.end(), rhs.jpos.begin(),rhs.jpos.end(),
            [](std::pair<std::string,double> x, std::pair<std::string,double>  y) { return x.first == y.first && dbl_equal(x.second, y.second, 0.001); });
}
