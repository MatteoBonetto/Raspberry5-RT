/*
  _____ _                       _            _   
 |_   _(_)_ __ ___   ___ _ __  | |_ ___  ___| |_ 
   | | | | '_ ` _ \ / _ \ '__| | __/ _ \/ __| __|
   | | | | | | | | |  __/ |    | ||  __/\__ \ |_ 
   |_| |_|_| |_| |_|\___|_|     \__\___||___/\__|
                                                 

Compile with: clang++ -std=c++17 -o timer timer.cpp
Run as:       sudo ./timer 0.05
*/
#define TIMER_MAIN
#define ENABLE_SCHEDULER
#include "timer.hpp"

