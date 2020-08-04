#include "Console.h"

namespace smart {
void
disableStdoutStream() {
  std::cout.setstate(std::ios::failbit);
  std::cerr.setstate(std::ios::failbit);
};

} /* namespace ydlidar */
