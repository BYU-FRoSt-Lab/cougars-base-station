#include <iostream>
#include <stdio.h>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Messages.h>
#include <seatrac_driver/commands.h>
using namespace narval::seatrac;

class MyDriver : public SeatracDriver
{
    public:

    MyDriver(const std::string& serialPort = "/dev/ttyUSB0") :
        SeatracDriver(serialPort)
    {}

    void ping_beacon(BID_E target, AMSGTYPE_E pingType = MSG_REQU) {
        messages::PingSend::Request req;
        req.target   = target;
        req.pingType = pingType;

        this->send(sizeof(req), (const uint8_t*)&req);
    }

    // this method is called on any message returned by the beacon.
    void on_message(CID_E msgId, const std::vector<uint8_t>& data) {
        //replace code in this method by your own
        switch(msgId) {
            default:
                std::cout << "Got message : " << msgId << std::endl << std::flush;
                break;

            case CID_PING_RESP: {
                    messages::PingResp response;        //struct that contains response fields
                    response = data;                    //operator overload fills in response struct with correct data
                    std::cout << response << std::endl; //operator overload prints out response data

                    //sends another ping to the other beacon, creating a feedback loop between ping sends and ping responses
                    this->ping_beacon(response.acoFix.srcId, MSG_REQU);

                } break;
            case CID_PING_ERROR: {
                    messages::PingError response;
                    response = data;
                    std::cout << response << std::endl;

                    this->ping_beacon(response.beaconId, MSG_REQU);

                } break;
            case CID_PING_SEND: {
                    messages::PingSend response;
                    response = data;
                    std::cout << response << std::endl;
                } break;

            case CID_DAT_RECEIVE: {               
                    messages::DataReceive response;
                    response = data;
                    std::cout << response << std::endl; 
                } break;
            case CID_DAT_ERROR: {
                    messages::DataError response;
                    response = data;
                    std::cout << response << std::endl;
                } break;

            case CID_STATUS:
                // too many STATUS messages so bypassing display.
                break;
        }

    }
};

int main(int argc, char *argv[])
{

    std::string serial_port;
    if (argc == 1) serial_port = "/dev/ttyUSB0";
    else serial_port = argv[1];

    MyDriver seatrac(serial_port);    

    seatrac.ping_beacon(BEACON_ID_15, MSG_REQU);

    getchar();

    return 0;
}
