// Copyright 2022 Institute of Automatic Control RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and limitations under the License.
//
// Author: Felix Widmayer (f.widmayer@irt.rwth-aachen.de)
//

// evologics_driver
#include <evologics_node.hpp>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "evologics_node");
    ros::NodeHandle nh;
    auto en = std::make_shared<evologics::EvologicsNode>(nh);

    ros::spin();
}