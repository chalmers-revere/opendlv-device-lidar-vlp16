/*
 * opendlv-device-lidar-vlp16 decodes VLP16 data realized for OpenDLV.
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "vlp16-decoder.hpp"

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("vlp16_port")) || (0 == commandlineArguments.count("cid")) ) {
        std::cerr << argv[0] << " decodes pointcloud data from a VelodyneLidar VLP16 unit and publishes it to a running OpenDaVINCI session using the OpenDLV Standard Message Set." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " [--vlp16_ip=<IPv4-address>] --vlp16_port=<port> --cid=<OpenDaVINCI session> [--id=<Identifier in case of multiple lidars>] [--verbose] [--intensity=<number of higher bits used for intensity>]" << std::endl;
        std::cerr << "         --intensity: VLP16 is using 16 bits to encode distances by default; when specifying this parameter with" << std::endl;
        std::cerr << "                      a value n > 0, the higher n bits will be used to encode intensity values for a given" << std::endl;
        std::cerr << "                      distance and thus, not using these n bits for distances. Thus, specifying this" << std::endl;
        std::cerr << "                      parameter might impose a *SAFETY RISK* as this software would not report objects" << std::endl;
        std::cerr << "                      that are within that particular range to the sensor. USE THIS PARAMETER AT YOUR OWN RISK!!" << std::endl;
        std::cerr << "                      Possible value range is [0..6]." << std::endl;
        std::cerr << "Example: " << argv[0] << " --vlp16_ip=0.0.0.0 --vlp16_port=2368 --cid=111" << std::endl;
        retCode = 1;
    } else {
        const uint32_t ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
        int32_t INTENSITY{(commandlineArguments["intensity"].size() != 0) ? static_cast<int32_t>(std::stoi(commandlineArguments["intensity"])) : 0};
        if ( (INTENSITY < 0) || (INTENSITY > 6) ) {
            std::cerr << "[lidar-vlp16] Specified value for intensity is not within the range [0..6]; using 0." << std::endl;
            INTENSITY = 0;
        }
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Interface to a running OpenDaVINCI session (ignoring any incoming Envelopes).
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"])),
            [](auto){}
        };

        // Interface to VelodyneLidar VLP16 unit.
        const std::string VLP16_ADDRESS((commandlineArguments.count("vlp16_ip") == 0) ? "0.0.0.0" : commandlineArguments["vlp16_ip"]);
        const uint32_t VLP16_PORT(std::stoi(commandlineArguments["vlp16_port"]));
        VLP16Decoder vlp16Decoder(INTENSITY);
        cluon::UDPReceiver fromDevice(VLP16_ADDRESS, VLP16_PORT,
            [&od4Session = od4, &decoder = vlp16Decoder, senderStamp = ID, VERBOSE](std::string &&d, std::string &&/*from*/, std::chrono::system_clock::time_point &&tp) noexcept {
            auto retVal = decoder.decode(d);
            if (retVal.first) {
                cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
                od4Session.send(retVal.second, sampleTime, senderStamp);
                // Print values on console.
                if (VERBOSE) {
                    std::cout << "[lidar-vlp16] Decoded data into PointCloudReading." << std::endl;
                }
            }
        });

        // Just sleep as this microservice is data driven.
        using namespace std::literals::chrono_literals;
        while (od4.isRunning()) {
            std::this_thread::sleep_for(1s);
        }
    }
    return retCode;
}
