#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#include "src/VescUart.h"
#include <iostream>
#include <string>
#include "spdlog/spdlog.h"
#include "spdlog/logger.h"


enum Options { START, STOP };

Options resolveOptions(const std::string& input) {
  if (input == "s")
    return Options::START;
  if (input == "t")
    return Options::STOP;
}

int main(int argc, char **argv) {
  VescUart vesc;
  std::string input_string;
  spdlog::set_level(spdlog::level::debug);
  SPDLOG_DEBUG("Starting app ...");

  int current_rpm{0};

  while (true) {
    SPDLOG_INFO("Waiting for input...");
    std::getline(std::cin, input_string);
    switch (resolveOptions(input_string)) {
    case (Options::START):
      SPDLOG_DEBUG("Starting motor ...");
      vesc.setRPM(200);
      break;
    case (Options::STOP):
      SPDLOG_DEBUG("Stopping motor ...");
      vesc.setRPM(0);
      break;
    default:
      vesc.setRPM(0);
    }
  }

}