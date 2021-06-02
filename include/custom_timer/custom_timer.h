#ifndef _H_TIMER__
#define _H_TIMER__

#include <chrono>
#include <iostream>
#include <array>

class Timer {

  std::chrono::high_resolution_clock::time_point start_time_;
  std::string text_;
  std::string timing_;
  std::array<std::string, 3> possible_timings_ = {"microseconds", "milliseconds", "seconds"};

public:
  Timer(std::string text = "", std::string timing = "microseconds") :  text_(text), timing_(timing) {
    int timing_index = -1;
    
    for (size_t index = 0; index < possible_timings_.size(); ++index) {
      if (timing_ == possible_timings_.at(index)) {
        timing_index = index;
        break;
      }
    }

    try {
      if (timing_index == -1) {
        throw std::string("Timing value is not valid");
      }
    }
    catch(std::string exception) {
      std::cout << exception << "\n";
      exit(-1);
    }
    start_time_ = std::chrono::high_resolution_clock::now();
  }

  ~Timer() {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time_);
    double duration_time = duration.count();

    if (timing_ == "milliseconds") {
      duration_time /= 1000;
    } else if (timing_ == "seconds") {
      duration_time /= 1000000;
    }

    if (text_.size()) {
      std::cout << text_ << ": ";
    }

    std::cout << duration_time << " " << timing_ << "\n";
  }
};
#endif