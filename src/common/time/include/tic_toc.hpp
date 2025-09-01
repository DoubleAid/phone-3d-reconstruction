#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc {
public:
    TicToc() {
        tic();
    }

    void tic() {
        start = std::chrono::system_clock::now();
    }

    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elasped_seconds = end - start;
        return elasped_seconds.count() * 1000;
    }
private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};