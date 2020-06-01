/* Usage: include this header file somewhere in your code (eg. precompiled header), and then use like:

PROFILING_START("Session Name");          // Begin session
{
  PROFILE_SCOPE("A scope");               // Profile current scope
  // Code
  void anyFunction()
  {
    PROFILE_FUNCTION();                   // Profile current function scope (scope name automatically set to function
                                          // name)
  }
}
PROFILING_END();                          // End Session

*/

#pragma once

#include <algorithm>
#include <chrono>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#if (PROFILING)
#define PROFILING_START(session_name) Instrumentor::instance().beginSession(session_name)
#define PROFILING_END() Instrumentor::instance().endSession()
#define PROFILE_SCOPE(name) InstrumentationTimer timer(name)
#define PROFILE_FUNCTION() PROFILE_SCOPE(__FUNCTION__)  // NOLINT
#else
#define PROFILING_START(session_name)
#define PROFILING_END(session_name)
#define PROFILE_SCOPE(name)
#define PROFILE_FUNCTION()
#endif

struct ProfileResult
{
  std::string name;
  long long start;
  long long end;
  std::thread::id thread_id;
};

struct InstrumentationSession
{
  std::string name = "None";
};

class Instrumentor
{
private:
  InstrumentationSession current_session_;
  std::ofstream output_stream_;
  int profile_count_ = 0;
  std::mutex lock_;
  bool active_session_ = false;

public:
  static Instrumentor& instance()
  {
    static Instrumentor instance;
    return instance;
  }

  void beginSession(const std::string& name, const std::string& filepath = "results.json")
  {
    if (active_session_)
    {
      endSession();
    }
    active_session_ = true;
    output_stream_.open(filepath);
    writeHeader();
    current_session_ = InstrumentationSession{ name };
  }

  void endSession()
  {
    if (!active_session_)
    {
      return;
    }
    active_session_ = false;
    writeFooter();
    output_stream_.close();
    profile_count_ = 0;
  }

  void writeProfile(const ProfileResult& result)
  {
    std::lock_guard<std::mutex> lock(lock_);

    if (profile_count_++ > 0)
    {
      output_stream_ << ",";
    }

    std::string name = result.name;
    std::replace(name.begin(), name.end(), '"', '\'');

    output_stream_ << "{";
    output_stream_ << R"("cat":"function",)";
    output_stream_ << R"("dur":)" << (result.end - result.start) << ',';
    output_stream_ << R"("name":")" << name << R"(",)";
    output_stream_ << R"("ph":"X",)";
    output_stream_ << R"("pid":0,)";
    output_stream_ << R"("tid":)" << result.thread_id << ",";
    output_stream_ << R"("ts":)" << result.start;
    output_stream_ << "}";
  }

  void writeHeader()
  {
    output_stream_ << R"({"otherData": {},"traceEvents":[)";
  }

  void writeFooter()
  {
    output_stream_ << "]}";
  }
};

class InstrumentationTimer
{
  ProfileResult result_;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_timepoint_;
  bool stopped_ = false;

public:
  explicit InstrumentationTimer(const char* name) : result_({ name, 0, 0 })
  {
    start_timepoint_ = std::chrono::high_resolution_clock::now();
  }
  InstrumentationTimer(InstrumentationTimer& t) = default;
  InstrumentationTimer(InstrumentationTimer&& t) = default;
  InstrumentationTimer& operator=(const InstrumentationTimer& t) = default;
  InstrumentationTimer& operator=(InstrumentationTimer&& t) = default;

  ~InstrumentationTimer()
  {
    if (!stopped_)
    {
      stop();
    }
  }

  void stop()
  {
    auto end_timepoint = std::chrono::high_resolution_clock::now();

    result_.start =
        std::chrono::time_point_cast<std::chrono::microseconds>(start_timepoint_).time_since_epoch().count();
    result_.end = std::chrono::time_point_cast<std::chrono::microseconds>(end_timepoint).time_since_epoch().count();
    result_.thread_id = std::this_thread::get_id();

    Instrumentor::instance().writeProfile(result_);

    stopped_ = true;
  }
};