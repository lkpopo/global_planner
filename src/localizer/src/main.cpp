#include <iostream>
#include "localizer.h"



int main() {
    std::string pcd_path = "/home/zxhc/Workspace/Test/Test_code/test.pcd";
    localizer::ICPLocalizer localizer(pcd_path);

    return 0;
}