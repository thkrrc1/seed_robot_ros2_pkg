#include <chrono>
#include "adget_controller.hpp"
#include "aero_controller.hpp"

namespace aero_controller {
    int AdGetController::getAd(AeroController *controller, int ad_no, std::vector<int>& value_data){
        //Ad値取得コマンドを発行
        controller->sendAdGetCommand(ad_no);

        //応答を待つ
        while(1) {
            bool timeout = false;
            auto lock = adget_recvd.waitForData(10,timeout);
            if(timeout){
                return 0;
            }

            return adget_recvd.get_ad_value(value_data);
        }
    }
}
