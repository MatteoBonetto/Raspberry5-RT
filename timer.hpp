/*
  _____ _                            _
 |_   _(_)_ __ ___   ___ _ __    ___| | __ _ ___ ___
   | | | | '_ ` _ \ / _ \ '__|  / __| |/ _` / __/ __|
   | | | | | | | | |  __/ |    | (__| | (_| \__ \__ \
   |_| |_|_| |_| |_|\___|_|     \___|_|\__,_|___/___/

Signal-based timed loop
Author: paolo.bosetti@unitn.it
*/
#include <chrono>
#include <cmath>
#include <cstring>
#include <errno.h> // for errno
#include <map>
#include <signal.h> // for signal
#include <sstream>
#include <stdexcept> // for runtime_error
#include <string.h>  // for strerror
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

/*
Time scheme:

          Dt                           Dt                              
├──────────────────────────►├──────────────────────────►│              
│                           │                           │              
│                           │                           │              
│ TET                       │                           │              
├───────►│                  │                           │              
│        │                  │                           │              
▼────────┼──────────────────▼───────────────────────────▼─────────────┐
│########│::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::│
│########│::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::│
└────────┼───────────────────────────────────┬────────────────────────┘
         │                                   │                         
         │            Dt_max                 │                         
         ├──────────────────────────────────►│                         
 */ 


#define NSEC_PER_SEC 1000000000ULL

using namespace std;
using namespace chrono;

template <typename DurationType = duration<double>, bool EnableStats = false>
class Timer {
public:
  enum TimerErrorType {
    TIMER_OK = 0,
    TIMER_ERR_SIGNAL_LATE = -1,
    TIMER_ERR_MAX_WAIT_EXCEEDED = -2
  };

  // LIFE-CYCLE ----------------------------------------------------------------
  template <typename IntervalType, typename MaxWaitType>
  explicit Timer(IntervalType interval, MaxWaitType max_wait,
                 bool rt_sched = false) {
    _interval = duration_cast<DurationType>(interval);
    _max_wait = duration_cast<DurationType>(max_wait);
    time_to_time_struct(_interval, _rep.it_interval);
    time_to_time_struct(_interval, _rep.it_value);
    time_to_time_struct(_max_wait, _rqtp);
  }

  ~Timer() { stop(); };

  void enable_rt_scheduler() {
#ifdef ENABLE_RT_SCHEDULER
    // Set the scheduler to SCHED_FIFO with priority 1
    sched_param param;
    param.sched_priority = 1;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
      throw runtime_error(string("Failed to set scheduler: ") +
                          strerror(errno));
    }
#else
    throw runtime_error("Real-time scheduler not enabled in this build");
#endif
  }

  string what() {
    stringstream ss;
    ss << "Interval: " << _rep.it_value.tv_sec + _rep.it_value.tv_usec / 1.0E6
       << endl
       << "Max wait: " << _rqtp.tv_sec + _rqtp.tv_nsec / 1.0E9 << endl;
    return ss.str();
  }

  // METHODS -------------------------------------------------------------------

  void start() {
    _last = system_clock::now().time_since_epoch();
#ifdef ENABLE_RT_SCHEDULER
    clock_gettime(CLOCK_REALTIME, &_now_ts);
    timespec_add_interval(&_now_ts);
#else
    struct itimerval itimer;
    // First interval:
    itimer.it_interval.tv_sec = 0;
    itimer.it_interval.tv_usec = 250 * 1000;
    // Subsequent intervals:
    itimer.it_value.tv_sec = 0;
    itimer.it_value.tv_usec = 250 * 1000;

    if (setitimer(ITIMER_REAL, &_rep, NULL) != 0) {
      throw runtime_error(std::strerror(errno));
    }
    signal(SIGALRM, [](int signo) {});
#endif
    _started = true;
  }

  void stop() {
    struct itimerval timer;
    timerclear(&timer.it_value);
    timerclear(&timer.it_interval);
    setitimer(ITIMER_REAL, &timer, NULL);
    signal(SIGALRM, SIG_DFL);
    _n = 0;
    _min = INFINITY;
    _max = 0;
    _mean = 0;
    _tet = 0;
    _sd = 0;
    _started = false;
    _first = true;
  }

  DurationType remaining() {
    auto sec = seconds(_rmtp.tv_sec);
    auto nsec = nanoseconds(_rmtp.tv_nsec);
    return duration_cast<DurationType>(sec + nsec);
  }

  DurationType elapsed() { return _max_wait - remaining(); }

  TimerErrorType wait() {
    if (!_started) {
      throw runtime_error("Timer: not started");
    }
    TimerErrorType ret = TIMER_OK;
    double dt = 0;
    auto now = system_clock::now().time_since_epoch();
#ifdef ENABLE_RT_SCHEDULER
    ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &_now_ts, NULL);
    timespec_add_interval(&_now_ts);
    time_to_time_struct(now - _now, _rmtp);
    _now = now; // update the current time
    dt = elapsed().count();
    if (dt > _max_wait.count()) {
      _rmtp.tv_sec = _rmtp.tv_nsec = 0; 
      ret = -1;
    }
#else
    // call NOT interrupted by SIGALRM
    if (nanosleep(&_rqtp, &_rmtp) == 0) {
      _rmtp.tv_sec = _rmtp.tv_nsec = 0;
      ret = TIMER_ERR_SIGNAL_LATE;
    }
    // call interrupted by SIGALRM
    else {
      ret = TIMER_OK;
    }
    dt = duration_cast<DurationType>(now - _last).count();
#endif
    if constexpr (EnableStats) {
      if (!_first) {
        _min = min(_min, dt);
        _max = max(_max, dt);
        _tet = dt - duration_cast<DurationType>(_max_wait - remaining()).count();
        if (ret == TIMER_OK) update_stats(dt); // don't update on signals
      }
      _first = false;
    }
    if (dt > _max_wait.count()) {
      _rmtp.tv_sec = _rmtp.tv_nsec = 0; // reset remaining time
      ret = TIMER_ERR_MAX_WAIT_EXCEEDED; // indicate that max wait time exceeded
    }
    _last = now;
    return ret;
  }

  void wait_throw() {
    switch (wait()) {
    case TIMER_ERR_SIGNAL_LATE:
      throw runtime_error("Timer: signal was late");
    case TIMER_ERR_MAX_WAIT_EXCEEDED:
      throw runtime_error("Timer: cycle time exceeded maximum: " +
                          to_string(_max_wait.count()) + " sec");
    default:
      return;
    }
  }

  map<string, double> stats() {
    if constexpr (EnableStats) {
      return {{"n", _n},
              {"min", _min},
              {"max", _max},
              {"mean", _mean},
              {"sd", _sd},
              {"tet", _tet}};
    } else {
      throw runtime_error("Timer: stats not enabled");
    }
  }

private:
  // ATTRIBUTES ----------------------------------------------------------------
  DurationType _interval;
  DurationType _max_wait;
  struct itimerval _rep;
  struct timespec _rqtp, _rmtp;
  map<string, double> _stats;
  size_t _n = 0;
  double _min = INFINITY, _max = 0, _mean = 0, _sd = 0, _tet = 0;
  bool _started = false, _first = true;
  struct timespec _now_ts;
  duration<double> _last;

  // PRIVATE METHODS -----------------------------------------------------------
  void update_stats(double x) {
    _n++;
    if (_n <= 1) { // recursion formula: first element (base-1)
      _mean = x;
      _sd = 0;
    } else {
      const double n1 = _n - 1;
      const double n2 = _n - 2;
      const double nr = 1.0 / _n;
      const double n1r = 1.0 / n1;
      const double nn1 = _n / n1;
      _mean = nr * (n1 * _mean + x);
      _tet = nr * (n1 * _tet + x);
      _sd = sqrt(n1r * (n2 * pow(_sd, 2) + nn1 * pow(_mean - x, 2)));
    }
  }

  template <typename T, typename S>
  static void time_to_time_struct(T d, S &ts) {
    ts.tv_sec = duration_cast<seconds>(d).count();
    if constexpr (is_same<S, struct timeval>::value) {
      ts.tv_usec = duration_cast<microseconds>(d).count() - ts.tv_sec * 1E6;
    } else if constexpr (is_same<S, struct timespec>::value) {
      ts.tv_nsec = duration_cast<nanoseconds>(d).count() - ts.tv_sec * 1E9;
    } else {
      throw runtime_error("Unsupported time struct type");
    }
  }

#ifdef ENABLE_RT_SCHEDULER
  inline void timespec_add_interval(struct timespec *t) {
    long dns = duration_cast<nanoseconds>(_interval).count();
    t->tv_nsec += dns;
    t->tv_sec += t->tv_nsec / NSEC_PER_SEC;
    t->tv_nsec %= NSEC_PER_SEC;
  }
#endif
};

/*
  _____ _____ ____ _____
 |_   _| ____/ ___|_   _|
   | | |  _| \___ \ | |
   | | | |___ ___) || |
   |_| |_____|____/ |_|

*/

#ifdef TIMER_MAIN

#include <iostream>
#include <thread>
#ifdef ENABLE_SCHEDULER
#include <sched.h>
#endif

bool Running = true;

int main(int argc, const char *argv[]) {
  double delay = 0.1;
  if (argc == 2)
    delay = atof(argv[1]);

  signal(SIGINT, [](int signo) { Running = false; });

  duration<double> d(delay);
  duration<double> max_d(delay * 1.1); // 1 second

  // Default template parameter is duration<double> in secs
  Timer<duration<double>, true> t(d, max_d, true);
  // Or:
  // Timer<milliseconds> t(milliseconds(200), milliseconds(1000));

  try {
    t.enable_rt_scheduler();
  } catch (const runtime_error &e) {
    cerr << "Error enabling real-time scheduler: " << e.what() << endl;
  }

  cerr << t.what();
  t.start();
  double min = 0, max = INFINITY;

  cout << "n,dt,min,max,mean,sd,tet" << endl;
  while (Running) {
    cout << t.stats()["n"] << "," << t.elapsed().count() << ","
         << t.stats()["min"] << "," << t.stats()["max"] << ","
         << t.stats()["mean"] << "," << t.stats()["sd"] << ","
         << t.stats()["tet"] << endl;
    this_thread::sleep_for(chrono::milliseconds(75));
    t.wait_throw();
  }
  cout << endl;
  cout << "Timer stopped after " << t.stats()["n"] << " events." << endl;
  cout << "Min time: " << t.stats()["min"] << " sec" << endl;
  cout << "Max time: " << t.stats()["max"] << " sec" << endl;
  cout << "Mean time: " << t.stats()["mean"] << " sec" << endl;
  cout << "Mean TET: " << t.stats()["tet"] << " sec" << endl;
  cout << "Standard deviation: " << t.stats()["sd"] << " sec" << endl;

  t.stop();
  return 0;
}

#endif