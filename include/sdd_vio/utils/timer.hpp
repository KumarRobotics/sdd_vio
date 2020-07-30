/*-------------------------------------------------------------
Copyright 2019 Wenxin Liu, Kartik Mohta, Giuseppe Loianno

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
--------------------------------------------------------------*/

#ifndef KR_COMMON_TIMER_HPP_
#define KR_COMMON_TIMER_HPP_

#include <chrono>
#include <string>
#include <cassert>
#include <ostream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <cmath>

namespace kr {

typedef std::chrono::seconds sec;
typedef std::chrono::milliseconds ms;
typedef std::chrono::microseconds us;
typedef std::chrono::nanoseconds ns;

/**
 * @brief The is_duration struct
 */
template <typename>
struct is_duration : std::false_type {};

template <typename T, typename U>
struct is_duration<std::chrono::duration<T, U>> : std::true_type {};

/**
 * @brief Unit
 * @return unit as a string
 */
template <typename T>
inline std::string DurationUnit() {
  return "unknown unit";
}

template <>
inline std::string DurationUnit<sec>() {
  return "sec";
}

template <>
inline std::string DurationUnit<ms>() {
  return "ms";
}

template <>
inline std::string DurationUnit<us>() {
  return "us";
}

template <>
inline std::string DurationUnit<ns>() {
  return "ns";
}

/**
 * @brief Ratio
 * @return ratio of type NumType and DenType
 */
template <typename NumType, typename DenType>
double Ratio() {
  typedef typename NumType::period NumPeriod;
  typedef typename DenType::period DenPeriod;
  typedef typename std::ratio_divide<NumPeriod, DenPeriod>::type RatioType;
  return static_cast<double>(RatioType::num) / RatioType::den;
}

/**
 * @brief The Timer class
 */
template <typename DurationType>
class Timer {
  static_assert(is_duration<DurationType>::value, "Not a valid duration type");

 public:
  explicit Timer(const std::string& name, int report_every_n_iter = 0)
      : name_(name), report_every_n_iter_(report_every_n_iter) {}

  int iteration() const { return iteration_; }
  const std::string& name() const { return name_; }

  /**
   * @brief Start, start timing
   */
  void Start() {
    assert(!running_);
    running_ = true;
    start_ = std::chrono::high_resolution_clock::now();
  }

  /**
   * @brief Stop, stop timing
   */
  void Stop() {
    elapsed_ = std::chrono::duration_cast<DurationType>(
        std::chrono::high_resolution_clock::now() - start_);
    assert(running_);
    auto prev_avg = iteration_ > 0 ? total_ / iteration_ : elapsed_;
    total_ += elapsed_;
    ++iteration_;
    auto const n = iteration_;
    auto const x = elapsed_ - prev_avg;
    variance_ = (n - 1) * (variance_ + x.count() * x.count() / n) / n;
    min_ = std::min(elapsed_, min_);
    max_ = std::max(elapsed_, max_);
    running_ = false;
    if (report_every_n_iter_ == 0) return;
    if (!(iteration_ % report_every_n_iter_)) Report();
  }

  /**
   * @brief Elapsed, last elapsed time duration
   */
  template <typename T = DurationType>
  double Elapsed() const {
    return elapsed_.count() * Ratio<DurationType, T>();
  }

  /**
   * @brief Min, shortest time duration recorded
   */
  template <typename T = DurationType>
  double Min() const {
    return min_.count() * Ratio<DurationType, T>();
  }

  /**
   * @brief Max, longest time duration recorded
   */
  template <typename T = DurationType>
  double Max() const {
    return max_.count() * Ratio<DurationType, T>();
  }

  /**
   * @brief Average, average time duration
   */
  template <typename T = DurationType>
  double Average() const {
    return total_.count() * Ratio<DurationType, T>() / iteration_;
  }

  /**
   * @brief StdDev, standard deviation of the recorded times
   */
  template <typename T = DurationType>
  double StdDev() const {
    return std::sqrt(variance_) * Ratio<DurationType, T>();
  }

  /**
   * @brief Reset timer
   */
  void Reset() {
    iteration_ = 0;
    running_ = false;
  }

  template <typename T = DurationType>
  void Sleep(int tick) {
    T duration(tick);
    std::this_thread::sleep_for(duration);
  }

  /**
   * @brief BaseUnit
   * @return base unit of the timer when it's instantiated
   */
  std::string BaseUnit() { return DurationUnit<DurationType>(); }

  /**
   * @brief Unit
   * @return unit of the timer with duration type T
   */
  template <typename T = DurationType>
  std::string Unit() {
    return DurationUnit<T>();
  }

  /**
   * @brief Report
   * @param unit_name A string representing the unit
   */
  template <typename T = DurationType>
  void Report(std::ostream& os = std::cout) const {
    os << name_ << " - iterations: " << iteration_
       << ", unit: " << DurationUnit<T>() << ", average: " << Average<T>()
       << ", stdev: " << StdDev<T>()
       << ", min: " << Min<T>() << ", max: " << Max<T>() << std::endl;
  }

 private:
  std::string name_{"timer"};
  int iteration_{0};
  int report_every_n_iter_{0};
  bool running_{false};
  std::chrono::high_resolution_clock::time_point start_;
  DurationType min_{DurationType::max()};
  DurationType max_{DurationType::min()};
  DurationType elapsed_{0};
  DurationType total_{0};
  double variance_{0};
};

typedef Timer<sec> TimerSec;
typedef Timer<ms> TimerMs;
typedef Timer<us> TimerUs;
typedef Timer<ns> TimerNs;

template <typename ClockType>
void PrintClockData() {
  std::cout << "- precision: ";
  // if time unit is less or equal one millisecond
  typedef typename ClockType::period RatioType;  // type of time unit
  if (std::ratio_less_equal<RatioType, std::milli>::value) {
    // convert to and print as milliseconds
    typedef typename std::ratio_multiply<RatioType, std::kilo>::type TT;
    std::cout << std::fixed << static_cast<double>(TT::num) / TT::den << " ms"
              << std::endl;
  } else {
    // print as seconds
    std::cout << std::fixed << static_cast<double>(RatioType::num) /
                                   RatioType::den << " sec" << std::endl;
  }
  std::cout << "- is_steady: " << std::boolalpha << ClockType::is_steady
            << std::endl;
}

}  // namespace kr

#endif  // KR_COMMON_TIMER_HPP_
