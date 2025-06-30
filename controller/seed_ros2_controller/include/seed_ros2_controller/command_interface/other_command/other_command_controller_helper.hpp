#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include <seed_ros2_controller/command_interface/controller_base.hpp>
#include <seed_ros2_controller/command_interface/command_interface.hpp>
#include <seed_ros2_controller/command_interface/other_command/other_command_interface.hpp>
#include <seed_ros2_controller/command_interface/swap_buffer.hpp>


namespace seed_ros2_controller{
namespace command_interface{
class OtherCommandControllerHelper: public ControllerBase<OtherCommandInterface> {
public:
    OtherCommandControllerHelper() {
        thread_nrt = std::thread(&OtherCommandControllerHelper::nrt_thread, this);
    }

    virtual ~OtherCommandControllerHelper(){
        running.store(false);
        mtx.lock();
        cond.notify_one();
        mtx.unlock();
        if(thread_nrt.joinable()){
            thread_nrt.join();
        }
    }

    virtual bool activate() {
        return false;
    }

    virtual void deactivate(){

    }

    /**
     * 非RTスレッドで実行される。
     */
    virtual void execute(const BuffRaw *buf_recv, BuffRaw *buf_send) {
    }

    /**
     * 非RTスレッドで実行される。
     */
    virtual void execute(const std::string &protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) {
        execute(buf_recv, buf_send);
    }

    /**
     * 非RTスレッドで実行される。
     */
    virtual void execute(int msid, const std::string &protocol, const BuffRaw *buf_recv, BuffRaw *buf_send) {
        execute(protocol, buf_recv, buf_send);
    }

    virtual std::vector<std::pair<uint8_t,uint8_t>> handleHeaders(){
        return std::vector<std::pair<uint8_t,uint8_t>>();
    }

    bool addSendData(int msid, const BuffRaw &buf_send) {
        std::unique_lock < std::mutex > lk(user_send_mtx, std::try_to_lock);
        if (!lk.owns_lock()) {
            //LOG_ERROR_STREAM()<<"owns lock error"<<std::endl;
            return false;
        }

        auto itr = std::find_if(ports_info.begin(),ports_info.end(),[&](const PortInfo& info){return info.msid == msid;});
        if(itr != ports_info.end()){
            if (itr->swp_buff_send.writeFromA(buf_send)) {
                return true;
            }
        }

//        LOG_ERROR_STREAM()<<"write error"<<std::endl;
        return false;
    }

    std::vector<int> getMsList() const{
        std::vector<int> ret;
        for(auto &port_info:ports_info){
            ret.push_back(port_info.msid);
        }
        return ret;
    }

    /**
     * MSに対する関節
     */
    std::vector<std::string> getJointNames(int msid) const{
        std::vector<std::string> ret;
        auto itr = std::find_if(ports_info.begin(),ports_info.end(),[&](const PortInfo& info){return info.msid == msid;});
        if(itr != ports_info.end()){
            ret = itr->joint_names;
        }
        return ret;
    }

    std::string getProtocol(int msid) const{
        std::string ret = "";
        auto itr = std::find_if(ports_info.begin(),ports_info.end(),[&](const PortInfo& info){return info.msid == msid;});
        if(itr != ports_info.end()){
            ret = itr->protocol;
        }
        return ret;
    }

private:

    void activate_hardware(OtherCommandInterface& hw) override{
        auto port_names = hw.get_names();
        ports_info.resize(port_names.size());
        for (size_t idx = 0; idx < port_names.size(); ++idx) {
            PortInfo &info = ports_info[idx];
            info.name = port_names[idx];
            try {
                info.msid = std::stoi(port_names[idx]);
            } catch (...) {
                //LOG_ERROR_STREAM()<<"invalid msid: "<<port_names[idx]<<LOG_END;
            }
            info.handle = hw.get_handle(info.name);
            info.joint_names = info.handle.get_joint_names();
            info.protocol = info.handle.get_protocol();
        }
        handle_headers = handleHeaders();
        activate();
    }

    void release_hardware() override {
        deactivate();
        std::scoped_lock lk { mtx, user_send_mtx };
        ports_info.clear();
    }

    virtual controller_interface::CallbackReturn on_init() override{
        return controller_interface::CallbackReturn::SUCCESS;
    };

    virtual controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override{
        return controller_interface::CallbackReturn::SUCCESS;
    };

    controller_interface::return_type  update(const rclcpp::Time &time, const rclcpp::Duration &period) override final {
        bool cmd_exist = false;

        for(auto &port_info : ports_info){
            port_info.buff.size = 0;

            //受信バッファから読み出す
            for (size_t idx = 0; idx < handle_headers.size(); ++idx) {
                port_info.handle.get_cmd(handle_headers[idx].first, handle_headers[idx].second, &port_info.buff);
            }

            //受信データを非RT通信用領域に詰める
            for (int idx = 0; idx < port_info.buff.size; ++idx) {
                port_info.swp_buff_recv.writeFromA(port_info.buff.buf[idx]);
                cmd_exist = true;
            }
        }

        bool goto_next_step = false;
        for (auto &port_info : ports_info) {
            //送信バッファを詰める
            while (1) {
                if (port_info.send_buff_reserved) {
                    if (!port_info.handle.send_cmd(port_info.send_buff_reserved)) {
                        //LOG_INFO_STREAM() << "buffer was full. try again in the next step" << std::endl;
                        //送信データはあるが、送信バッファがつまっている場合
                        //全msのデータのタイミングを一致させる必要があるかもしれないので、一つが失敗したら全て次周期に送る。
                        goto_next_step = true;
                        break;
                    }
                }

                port_info.send_buff_reserved = port_info.swp_buff_send.readFromB();
                if (!port_info.send_buff_reserved) {
                    //送信データがなければ、次のポートを見る
                    break;
                }
            }

            if (goto_next_step) {
                break;
            }
        }

        //非RTスレッドを動かす
        if (cmd_exist) {
            std::unique_lock < std::mutex > lk(mtx, std::try_to_lock);
            if (lk.owns_lock()) {
                cond.notify_one();
            }
        }

        return controller_interface::return_type::OK;
    }


    void nrt_thread() {
        while (running.load()) {
            std::unique_lock < std::mutex > lk(mtx);
            cond.wait(lk);

            for(auto &port_info:ports_info){
                while (1) {
                    BuffRaw data_send;
                    data_send.size = 0;
                    const BuffRaw *data_recv = port_info.swp_buff_recv.readFromB();
                    if (!data_recv) {
                        break;
                    }

                    execute(port_info.msid, port_info.protocol, data_recv, &data_send);

                    if (data_send.size != 0) {
                        port_info.swp_buff_send.writeFromA(data_send);
                    }
                }
            }
        }
    }


private:
    struct PortInfo{
        PortInfo(){}
        PortInfo(const PortInfo&) = delete;
        PortInfo(PortInfo&& info){};

        int msid = -1;                                //!< MSID
        std::string protocol = "";                    //!< プロトコルバージョン
        std::string name;                             //!< ポート(MS)名
        std::vector<std::string> joint_names;         //!< 関節名(送信順に並び替えられたもの)
        OtherCommandHandle handle;//!< コマンド送受信ハンドラ
        SwapBuffer<BuffRaw, 100> swp_buff_recv;       //!< 非RTとの通信用受信バッファ
        SwapBuffer<BuffRaw, 100> swp_buff_send;       //!< 非RTとの通信用送信バッファ
        BuffList buff;                                //!< RTからのデータ取得用バッファ

        BuffRaw* send_buff_reserved = nullptr;        //!< 次送信予定データ
    };

    std::vector<PortInfo> ports_info;

    std::atomic<bool> running = true;
    std::mutex user_send_mtx; //!< ユーザーからの送信要求用

    std::mutex mtx;               //!< 条件変数用
    std::condition_variable cond; //!< リアルタイムスレッドからの受信通知用

    std::vector<std::pair<uint8_t,uint8_t>> handle_headers; //!< 利用するパケット特定用データ

    std::thread thread_nrt;
};

}
}

