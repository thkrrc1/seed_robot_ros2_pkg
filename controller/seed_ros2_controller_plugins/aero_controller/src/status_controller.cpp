#include "status_controller.hpp"

#include "status.hpp"

namespace aero_controller {

StatusController::StatusController(){
    periodic_thread = std::thread(&StatusController::periodic_send,this);
};

StatusController::~StatusController(){
    shutdown.store(true);
    if (periodic_thread.joinable()) {
        periodic_thread.join();
    }
}

controller_interface::CallbackReturn StatusController::on_configure(const rclcpp_lifecycle::State &previous_state) {
    status_pub = get_node()->create_publisher<aero_controller_msgs::msg::Status>(std::string(get_node()->get_name()) + "/status",1);
    if(status_parse){
        status_parsed_pub = get_node()->create_publisher<aero_controller_msgs::msg::StatusParsed>(std::string(get_node()->get_name()) + "/status_parsed",1);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

void StatusController::periodic_send(){
    while (!shutdown.load()) {

        auto mslist = getMsList();
        for (int &ms : mslist) {
            BuffRaw buff;
            makeBuff(getProtocol(ms), buff, ms);
            addSendData(ms, buff);
        }

        //100[ms]ごとに要求を投げる
        usleep(100000);
    }
}

void StatusController::execute(const std::string &protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) {
    //checksumの確認は実施済みなので、ここでは実行しない。

    if (getData(protocol, *buf_recv, status_msg)) {
        status_pub->publish(status_msg);
    }
    // 0x61の内容を項目ごとに変換,使わない場合は"status_parse=false"に
    if(status_parse){
        StatusController::parse_status(status_msg);
        status_parsed_pub->publish(status_parsed_msg);
    }
}

void StatusController::parse_status(const aero_controller_msgs::msg::Status &status){
    status_parsed_msg.msid = status.msid;

    std::bitset<8> bs;
    // ロボット状態
    bs = std::bitset<8>(status.data[1]);
    status_parsed_msg.robot_status.aerocom = bs[0];
    status_parsed_msg.robot_status.scenario = bs[1];
    status_parsed_msg.robot_status.blank_2 = bs[2];
    status_parsed_msg.robot_status.blank_3 = bs[3];
    status_parsed_msg.robot_status.protection_stop_soft = bs[4];
    status_parsed_msg.robot_status.protection_stop_hard = bs[5];
    status_parsed_msg.robot_status.emergency_stop_soft = bs[6];
    status_parsed_msg.robot_status.emergency_stop_hard = bs[7];

    bs = std::bitset<8>(status.data[2]);
    status_parsed_msg.robot_status.system = bs[0];
    status_parsed_msg.robot_status.charging = bs[1];
    status_parsed_msg.robot_status.power_supply = bs[2];
    status_parsed_msg.robot_status.ssr = bs[3];
    status_parsed_msg.robot_status.servo = bs[4];
    status_parsed_msg.robot_status.overcurrent1 = bs[5];
    status_parsed_msg.robot_status.overcurrent2 = bs[6];
    status_parsed_msg.robot_status.blank_1 = bs[7];

    // MS-MS間通信
    bs = std::bitset<8>(status.data[3]);
    status_parsed_msg.ms_ms.system = bs[0];
    status_parsed_msg.ms_ms.msid1 = bs[1];
    status_parsed_msg.ms_ms.msid2 = bs[2];
    status_parsed_msg.ms_ms.msid3 = bs[3];
    status_parsed_msg.ms_ms.msid4 = bs[4];
    status_parsed_msg.ms_ms.msid5 = bs[5];
    status_parsed_msg.ms_ms.msid6 = bs[6];
    status_parsed_msg.ms_ms.msid7 = bs[7];

    // PDBスイッチ
    bs = std::bitset<8>(status.data[6]);
    status_parsed_msg.pdb_switch.emergency_stop = bs[0];
    status_parsed_msg.pdb_switch.external_stop = bs[1];
    status_parsed_msg.pdb_switch.blue_button = bs[2];
    status_parsed_msg.pdb_switch.green_button = bs[3];
    status_parsed_msg.pdb_switch.sw1 = bs[4];
    status_parsed_msg.pdb_switch.sw2 = bs[5];
    status_parsed_msg.pdb_switch.sw3 = bs[6];
    status_parsed_msg.pdb_switch.sw4 = bs[7];

    // MS-PC間接続
    bs = std::bitset<8>(status.data[7]);
    status_parsed_msg.ms_pc.pc_power = bs[0];
    status_parsed_msg.ms_pc.usb = bs[1];
    status_parsed_msg.ms_pc.pc = bs[2];
    status_parsed_msg.ms_pc.sgs = bs[3];
    status_parsed_msg.ms_pc.blank = bs[4];
    status_parsed_msg.ms_pc.video_full = bs[5];
    status_parsed_msg.ms_pc.video_mix = bs[6];
    status_parsed_msg.ms_pc.video_normal = bs[7];

    // 電流・電圧
    std::bitset<16> bs2;
    float_t decimal_value;
    // 電圧
    bs2 = std::bitset<16>((status.data[13] << 8) | status.data[14]);
    decimal_value = static_cast<uint16_t>(bs2.to_ulong());
    status_parsed_msg.electrical_status.voltage = decimal_value / 1000;
    // 電流(Motor)
    bs2 = std::bitset<16>((status.data[15] << 8) | status.data[16]);
    decimal_value = static_cast<uint16_t>(bs2.to_ulong());
    status_parsed_msg.electrical_status.current_motor = decimal_value / 1000;
    // 電流(PWR SR)
    bs2 = std::bitset<16>((status.data[17] << 8) | status.data[18]);
    decimal_value = static_cast<uint16_t>(bs2.to_ulong());
    status_parsed_msg.electrical_status.current_pwr_sr = decimal_value / 1000;
    // 電流(常時on)
    bs2 = std::bitset<16>((status.data[19] << 8) | status.data[20]);
    decimal_value = static_cast<uint16_t>(bs2.to_ulong());
    status_parsed_msg.electrical_status.current_on = decimal_value / 1000;
    // 電流(DCDC)
    bs2 = std::bitset<16>((status.data[21] << 8) | status.data[22]);
    decimal_value = static_cast<uint16_t>(bs2.to_ulong());
    status_parsed_msg.electrical_status.current_dcdc = decimal_value / 1000;

    // PDB-IO
    bs = std::bitset<8>(status.data[23]);
    status_parsed_msg.pdb_io.stpcn_i = bs[0];
    status_parsed_msg.pdb_io.stpcn_i_s = bs[1];
    status_parsed_msg.pdb_io.dcdc1en = bs[2];
    status_parsed_msg.pdb_io.dcdc2en = bs[3];
    status_parsed_msg.pdb_io.reserved_1 = bs[4];
    status_parsed_msg.pdb_io.reserved_2 = bs[5];
    status_parsed_msg.pdb_io.reserved_3 = bs[6];
    status_parsed_msg.pdb_io.reserved_4 = bs[7];

    bs = std::bitset<8>(status.data[24]);
    status_parsed_msg.pdb_io.s3_s = bs[0];
    status_parsed_msg.pdb_io.s4 = bs[1];
    status_parsed_msg.pdb_io.s4_s = bs[2];
    status_parsed_msg.pdb_io.s2_s = bs[3];
    status_parsed_msg.pdb_io.sw1led = bs[4];
    status_parsed_msg.pdb_io.sw2led = bs[5];
    status_parsed_msg.pdb_io.stpcn_o = bs[6];
    status_parsed_msg.pdb_io.stpcn_o_s = bs[7];

    bs = std::bitset<8>(status.data[25]);
    status_parsed_msg.pdb_io.ssr_o1 = bs[0];
    status_parsed_msg.pdb_io.ssr_o1_s = bs[1];
    status_parsed_msg.pdb_io.ssr_o2 = bs[2];
    status_parsed_msg.pdb_io.ssr_o2_s = bs[3];
    status_parsed_msg.pdb_io.s1 = bs[4];
    status_parsed_msg.pdb_io.s3 = bs[5];
    status_parsed_msg.pdb_io.s1_s = bs[6];
    status_parsed_msg.pdb_io.s2 = bs[7];

    bs = std::bitset<8>(status.data[26]);
    status_parsed_msg.pdb_io.vcc_o = bs[0];
    status_parsed_msg.pdb_io.vcc_o2 = bs[1];
    status_parsed_msg.pdb_io.stpsw1 = bs[2];
    status_parsed_msg.pdb_io.stpsw1_s = bs[3];
    status_parsed_msg.pdb_io.stpsw2 = bs[4];
    status_parsed_msg.pdb_io.stpsw2_s = bs[5];
    status_parsed_msg.pdb_io.sw1 = bs[6];
    status_parsed_msg.pdb_io.sw2 = bs[7];
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aero_controller::StatusController, controller_interface::ControllerInterface)
