#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#include "src/VescUart.h"
#include <iostream>
#include <string>
#include "spdlog/spdlog.h"
#include "spdlog/logger.h"
#include <thread>


enum Options { START, STOP, GET_RPM, QUIT };

Options resolveOptions(const std::string& input) {
  if (input == "s")
    return Options::START;
  if (input == "t")
    return Options::STOP;
  if (input == "v")
    return Options::GET_RPM;
  if (input == "q")
    return Options::QUIT;
}

int main(int argc, char **argv) {
  std::shared_ptr<VescUart> vesc = std::make_shared<VescUart>();
  std::string input_string;
  spdlog::set_level(spdlog::level::debug);
  SPDLOG_DEBUG("Starting app ...");
  spdlog::debug("Starting app with other spdlog method");

  int current_rpm{0};
  bool should_quit{false};
  std::thread alive_thread_;
  alive_thread_ = std::thread([vesc, &should_quit]() {
    auto loop_start = std::chrono::high_resolution_clock::now();
    auto time_between_alive_msgs_s = std::chrono::milliseconds(500);
    while(!should_quit){
      //auto elapsed_time = std::chrono::high_resolution_clock::now() - loop_start;
      //if( elapsed_time < time_between_alive_msgs_s){
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      //}else{
        vesc->sendAlive();
      //}
    }
    spdlog::debug("Exiting alive thread");
  });

  while (!should_quit) {
    SPDLOG_INFO("Waiting for input...");
    std::getline(std::cin, input_string);
    switch (resolveOptions(input_string)) {
    case (Options::START):
      SPDLOG_DEBUG("Starting motor ...");
      vesc->setDuty(0.1);
      break;
    case (Options::STOP):
      SPDLOG_DEBUG("Stopping motor ...");
      vesc->setDuty(0);
      break;
    case (Options::GET_RPM):
      SPDLOG_DEBUG("printing values ...");
      vesc->getVescValues();
      vesc->printVescValues();
      break;
    case (Options::QUIT):
      SPDLOG_DEBUG("Quitting ...");
      vesc->setDuty(0);
      should_quit = true;
      break;
    default:
      vesc->setDuty(0);
    }
  }
  alive_thread_.join();
  return 0;
}
