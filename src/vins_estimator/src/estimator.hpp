#pragma once

#include "logger.hpp"

class Estimator {
public:
    Estimator();
    void processImage();

    void clearState();
};
