#include "vision_core/config_loader.hpp"
#include <exception>
#include <iostream>
int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: config_check <vision_algorithm.yaml>\n";
    return 2;
  }
  try {
    (void)vision_core::LoadAlgorithmConfig(argv[1]);
    std::cout << "Algorithm YAML valid: " << argv[1] << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
