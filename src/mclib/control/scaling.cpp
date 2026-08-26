// mclib
#include "mclib/control/scaling.hpp"

#include <cmath>
void scaleToMin(double &left_output, double &right_output, double min_output)
{
  // enforce a lower bound so both treads overcome static friction when pid wants slow motion
  if (fabs(left_output) <= fabs(right_output) && left_output < min_output && left_output > 0)
  {
    right_output = right_output / left_output * min_output;
    left_output = min_output;
  }
  else if (fabs(right_output) < fabs(left_output) && right_output < min_output && right_output > 0)
  {
    left_output = left_output / right_output * min_output;
    right_output = min_output;
  }
  else if (fabs(left_output) <= fabs(right_output) && left_output > -min_output && left_output < 0)
  {
    right_output = right_output / left_output * -min_output;
    left_output = -min_output;
  }
  else if (fabs(right_output) < fabs(left_output) && right_output > -min_output && right_output < 0)
  {
    left_output = left_output / right_output * -min_output;
    right_output = -min_output;
  }
}

void scaleToMax(double &left_output, double &right_output, double max_output)
{
  // cap magnitude while preserving ratio so curvature demands stay intact under saturation
  if (fabs(left_output) >= fabs(right_output) && left_output > max_output)
  {
    right_output = right_output / left_output * max_output;
    left_output = max_output;
  }
  else if (fabs(right_output) > fabs(left_output) && right_output > max_output)
  {
    left_output = left_output / right_output * max_output;
    right_output = max_output;
  }
  else if (fabs(left_output) > fabs(right_output) && left_output < -max_output)
  {
    right_output = right_output / left_output * -max_output;
    left_output = -max_output;
  }
  else if (fabs(right_output) > fabs(left_output) && right_output < -max_output)
  {
    left_output = left_output / right_output * -max_output;
    right_output = -max_output;
  }
}
