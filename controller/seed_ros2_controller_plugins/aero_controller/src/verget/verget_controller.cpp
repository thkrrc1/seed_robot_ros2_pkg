#include <chrono>
#include "verget_controller.hpp"
#include "aero_controller.hpp"

namespace aero_controller {

std::string VerGetController::getVersion(AeroController *controller, int ms, int mc) {
    //バージョン取得コマンドを発行
    controller->sendVerGetCommand(ms, mc);

    //応答を待つ
    while (1) {
        bool timeout = false;
        auto lock = verget_recvd.waitForData(10, timeout);
        if (timeout) {
            return "";
        }

        //送信したものと同じMSIDでの受信が届いたら、バージョンを取得
        if (ms == verget_recvd.get_msid()) {
            return verget_recvd.get_version();
        }
    }
}

}
