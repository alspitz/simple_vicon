#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <string>

#include "Client.h"

namespace ViconSDK = ViconDataStreamSDK::CPP;

class ViconResult {
  public:
    double latency{0.0};
    std::array<double, 3> pos{0, 0, 0};
    std::array<double, 4> quat{0, 0, 0};
};

class ViconDriver {
  typedef std::function<void(ViconResult)> callback_type;

  public:
    bool init(const std::string& host, const std::string& target_subject, callback_type callback);
    void run_loop();
    void stop();

  private:
    ViconSDK::Client client_;
    std::string target_subject_;
    std::atomic<bool> running_;
    callback_type callback_;
};
