#include <Eigen/Dense>

#include <cstdlib>
#include <exception>
#include <iostream>

#include "fileio/loadanddump.h"
#include "imufusion/imufusion.h"

std::ofstream timing;
int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <config.yaml>" << std::endl;
    return EXIT_FAILURE;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(argv[1]);
  } catch (const std::exception &exception) {
    std::cerr << "Failed to load config file '" << argv[1]
              << "': " << exception.what() << std::endl;
    return EXIT_FAILURE;
  }

  Paras paras;
  if (!loadConfig(config, paras)) {
    std::cerr << "Invalid configuration file: " << argv[1] << std::endl;
    return EXIT_FAILURE;
  }

  std::string timeoutputpath = paras.outputpath + "/timing.txt";
  timing.open(timeoutputpath.c_str());
  if (!timing.is_open()) {
    std::cerr << "Failed to open timing output file: " << timeoutputpath
              << std::endl;
    return EXIT_FAILURE;
  }
  timing.setf(std::ios::fixed, std::ios::floatfield);
  timing.precision(6);

  try {
    IMUFusion imufusion(paras);
    imufusion.run();
  } catch (const std::exception &exception) {
    std::cerr << "Failed to process bag '" << paras.bagpath
              << "': " << exception.what() << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}
