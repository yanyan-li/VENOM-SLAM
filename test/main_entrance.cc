/*
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-10-06 02:40:42
 * @LastEditTime: 2022-10-16 03:13:53
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /venom/test/main_entrance.cc
 */
#include <iostream>
#include <string>
#include "src/visulizer/Interface.hpp"


int main(int argc, char **argv)
{
    if (argc != 1)
    {
        std::cout << "usage: ./main_entrance " << std::endl;
        return -1;
    }

    std::cout << std::endl << 
        "\033[0;32mVenom SLAM Simulator Software Copyright (C) 2022 Yanyan Li, Technical University of Munich." << std::endl <<
        "This is a free software that is used for learning, teaching, and testing SLAM strategies." << std::endl <<
        "And you are welcome to contribute it and redistribute it under certain conditions. See LICENSE.txt. \033[0m" << std::endl << std::endl;

    // interface
    simulator::Interface venom_entrance;
    venom_entrance.StartVenom();

    //
    return 0;
}
