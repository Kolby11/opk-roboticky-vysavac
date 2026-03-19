#include "include/environment/Environment.h"
#include <iostream>

int main() {
    environment::Config cfg;
    //absolutna cesta
    cfg.map_filename = "/home/veronika/Documents/zadanie1-VeronikaSilharova/resources/opk-map.png";
    cfg.resolution = 0.5;
    
    //konstruktor
    environment::Environment env(cfg);
    return 0;
}