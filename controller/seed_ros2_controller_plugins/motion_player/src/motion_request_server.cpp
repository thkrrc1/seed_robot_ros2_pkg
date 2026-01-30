#include <iostream>
#include <filesystem>

#include <Eigen/Dense>
#include <urdf/model.h>

#include "motion_request_server.hpp"
#include "keyframe_motion2.hpp"
#include "fileio_json.hpp"
#include "spline.hpp"
#include "urdf_helper.hpp"
#include "utilities.hpp"
#include "arm_target.hpp"

namespace motion_player {

void dump(const char *str, uint8_t *data, int len) {
    printf("[%s] data size: %02d hex: ", str, len);
    for (int idx = 0; idx < len; ++idx) {
        if (idx % 4 == 0) {
            printf("  ");
        }
        printf("%02x", data[idx]);
    }
    printf("\n");
}

int sendArmTarget(double dt_sec, int step, double control_period_sec, std::vector<double> &prev_target, const std::vector<motion_editor2::ArmTarget2> &arm_tgts, const ArmInfoList &arm_info, const std::shared_ptr<seed_converter::StrokeConverter> stroke_converter_, reqBuffType *req_buff) {
    auto jnames = arm_info.getJointNames();
    std::vector < int16_t > strokes;
    strokes.resize(jnames.size(), 0x7FFF);
    //アーム動作

    int send_num = 0;

    //メディアの再生用に、空データを入れておく
    if (arm_tgts.size() == 0) {
        Request req;
        req.msid = -1;
        req.step = std::round(step * (dt_sec / control_period_sec));
        bool ok = false;
        while (!ok) {
            ok = req_buff->writeFromA(req);
            usleep(1000);
        }
        ++send_num;
    }

    for (auto &arm_tgt : arm_tgts) {
        double duration_sec = dt_sec * (arm_tgt.step_end - step);
        uint16_t duration = static_cast<uint16_t>(duration_sec * 100.);

        for (size_t jidx = 0; jidx < jnames.size(); ++jidx) {
            auto itr = arm_tgt.tgt_state.find(jnames[jidx]);
            if (itr != arm_tgt.tgt_state.end()) {
                auto jvalue = itr->second.pos;
                prev_target[jidx] = jvalue;
            }
        }

        if (stroke_converter_) {
            stroke_converter_->Angle2Stroke(strokes, prev_target);
        }

        int idx = 0;
        for (size_t msidx = 0; msidx < arm_info.info.size(); ++msidx) {
            Request req;
            req.msid = arm_info.info[msidx].msid;
            req.step = std::round(step * (dt_sec / control_period_sec));
            makeBuff(arm_info.info[msidx].protocol, req.rawdata, req.msid, arm_info.info[msidx], &(strokes.data()[idx]), duration);
            idx += arm_info.info[msidx].jinfo.size();

            bool ok = false;
            while (!ok) {
                ok = req_buff->writeFromA(req);
                usleep(1000);
            }
            ++send_num;
        }
    }

    return send_num;
}

motion_editor2::MotionBase2* loadKeyframeMotion(std::string &path_str, std::vector<std::string> joint_names, JointTarget &jtarget) {
    if (path_str.empty()) {
        return nullptr;
    }

    motion_editor2::KeyframeMotion2 *keyframe_motion = new motion_editor2::KeyframeMotion2;

    try {
        keyframe_motion->load(path_str);
        keyframe_motion->createDecoder(getParentPath(path_str));

        for (size_t idx = 0; idx < joint_names.size(); ++idx) {
            jtarget.tgtpos[idx] = 0;
        }

    } catch (...) {
        delete keyframe_motion;
        keyframe_motion = nullptr;
    }

    return keyframe_motion;
}

ArmInfoList createArmInfo(const seed_ros2_controller::command_interface::OtherCommandInterface& hw) {
    ArmInfoList ret;

    for (auto &handle_name : hw.get_names()) {
        auto handle = hw.get_handle(handle_name);

        ArmInfo arminfo;
        arminfo.msid = std::stoi(handle.get_name());
        arminfo.protocol = handle.get_protocol();
        auto jnames = handle.get_joint_names();
        for (size_t jidx = 0; jidx < jnames.size(); ++jidx) {
            if(jnames[jidx].empty()){
                continue;
            }
            JointInfo jinfo;
            jinfo.name = jnames[jidx];
            jinfo.sendid = jidx;
            arminfo.jinfo.push_back(jinfo);
        }
        ret.info.push_back(arminfo);
    }

    return ret;
}

MotionRequestServer::~MotionRequestServer() {
    shutdown.store(true);
    mtx.lock();
    cond.notify_one();
    mtx.unlock();
    if (send_thread.joinable()) {
        send_thread.join();
    }

    mtx.lock();
    if (cur_motion) {
        cur_miter = nullptr;
        delete cur_motion;
        cur_motion = nullptr;
    }
    mtx.unlock();
}

bool MotionRequestServer::load(const std::shared_ptr<motion_player_msgs::srv::Load::Request> req, std::shared_ptr<motion_player_msgs::srv::Load::Response> res) {
    std::string file_path = req->file_path;

    LOG_INFO_STREAM() << "motion file is to be loaded :" << file_path << LOG_END;

    std::unique_lock < std::mutex > lock(mtx, std::defer_lock);
    if (!lock.try_lock()) {
        LOG_ERROR_STREAM() << "motion is currently running" << LOG_END;
    }

    if (cur_motion) {
        cur_miter = nullptr;
        delete cur_motion;
        cur_motion = nullptr;
    }
    std::filesystem::path fpath = file_path;
    if (fpath.extension() == ".kf") {
        cur_motion = loadKeyframeMotion(file_path, joint_names, jtarget);
    }

    if (cur_motion) {
        cur_miter = cur_motion->getIterator();
        dt_sec = cur_miter->getStepTime();
        media_player.setMediaList(cur_motion->getAudioList(), cur_motion->getVideoList());
        media_player.set_step_time(cur_miter->getStepTime());
        res->result = true;
    } else {
        LOG_ERROR_STREAM() << "failed to load motion : " << file_path << LOG_END;
        res->result = false;
    }

    return true;
}


void MotionRequestServer::execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<PlayAction>> goal_handle){
    auto feedback = std::make_shared<PlayAction::Feedback>();
    auto result = std::make_shared<PlayAction::Result>();

    LOG_INFO_STREAM() << "motion start" << LOG_END;

    std::unique_lock < std::mutex > lock(mtx, std::defer_lock);
    if (!lock.try_lock()) {
        LOG_ERROR_STREAM() << "motion is currently running" << LOG_END;
    }

    if (!cur_motion || !cur_miter) {
        //"Motion has not loaded yet."
        goal_handle->abort(result);
        return;
    }

    start_step = goal_handle->get_goal()->start_step;
    int step_max = cur_miter->getMaxStep();
    media_player.seek(start_step, true);

    //JointTrajectoryControllerを一旦止める
    using namespace std::chrono_literals;
    auto list_req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto switch_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    switch_req->activate_asap = false; //全コントローラが切り替わってから、update処理を呼び始める
    switch_req->timeout = rclcpp::Duration(5.0s);
    if (!list_client_->wait_for_service(1s) || !switch_client_->wait_for_service(1s)) {
        return;
    }

    auto future = list_client_->async_send_request(list_req);
    auto controllers = future.get()->controller;
    for (auto &controller : controllers) {
        if (controller.type == "joint_trajectory_controller/JointTrajectoryController" && controller.state == "active") {
            switch_req->deactivate_controllers.push_back(controller.name);
        }
    }

    if (switch_req->deactivate_controllers.size() != 0) {
        deactivated_controllers = switch_req->deactivate_controllers;
        auto future = switch_client_->async_send_request(switch_req);
        future.get();
    }

    feedback->step = start_step;
    feedback->percent = 0;
    goal_handle->publish_feedback(feedback); //ここで実行中にしておく

    cond.notify_one(); //RT送信スレッドを立ち上げる
    lock.unlock();

    int cycle_noresp = 0;

    while (1) { //RT実行監視ループ

        if (!cancel.load() && goal_handle->is_canceling()) {
            cancel.store(true);
            stop_request->store(true); //RT側に終了要求を出す
            req_buff->setPermissionRead(false); //読み出し不可にしておく
            req_buff->clear(); //書き込み側が処理完了できるように、全データを削除しておく
            lock.lock(); //書き込み完了待ち
            req_buff->clear(); //書き込まれたデータをすぐ消去する
            lock.unlock();
        }

        if (cancel.load() && cycle_noresp > 1000) {
            //モーションが再生される前の停止要求がありうるので、少し待って応答が帰ってこない場合はループを抜ける
            resp_buff->clear();
            rclcpp::sleep_for(std::chrono::milliseconds(400));
            break;
        }

        auto resp = resp_buff->readFromB(false);
        if (!resp) {
            cycle_noresp++;
            usleep(1000);
            continue;
        }
        cycle_noresp = 0;

        double fb_step = resp->step * control_period_sec / dt_sec;
        if (fb_step < start_step) {
            fb_step = start_step;
        }
        feedback->step = static_cast<uint32_t>(fb_step);
        if (step_max > 0) {
            feedback->percent = static_cast<uint8_t>((100 * feedback->step) / step_max);
        }

        if (fb_step > start_step) {
            media_player.play_until(static_cast<int>(feedback->step));
        }

        goal_handle->publish_feedback(feedback);

        if (resp->done) {
            break;
        }
    }

    media_player.stop();

    //JointTrajectoryControllerを再開させる
    if (switch_req->deactivate_controllers.size() != 0) {
        auto restore_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        restore_req->activate_controllers = deactivated_controllers;
        restore_req->deactivate_controllers.clear();
        restore_req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
        restore_req->activate_asap = false;
        restore_req->timeout = rclcpp::Duration(5.0s);
        auto future_restore = switch_client_->async_send_request(restore_req);
        future_restore.get();
    }
    LOG_INFO_STREAM() << "Controller state restored after motion." << LOG_END;
    // 少し待ってcontrollerがアクティブになりトピック購読等が整うのを許す
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    deactivated_controllers.clear();

    if (!cancel.load()) {
        goal_handle->succeed(result);
        LOG_INFO_STREAM() << "motion was successfully completed" << LOG_END;
    } else {
        goal_handle->canceled(result);
        cancel.store(false);
        LOG_INFO_STREAM() << "motion was canceled" << LOG_END;
    }

}

sensor_msgs::msg::JointState MotionRequestServer::getCurrentJointState() {
    bool jstate_updated = false;
    std::mutex jstate_mtx;
    sensor_msgs::msg::JointState jstate;

    auto jstate_sub_ = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, [&](const sensor_msgs::msg::JointState::ConstSharedPtr &joint_state){
        jstate_mtx.lock();
        jstate_updated = true;
        jstate = *joint_state;
        jstate_mtx.unlock(); 
        LOG_INFO_STREAM() << "joint_states get it" << LOG_END;
    });

    //現在値が拾えるまで待つ
    while (!cancel.load() && !shutdown.load()) {
        usleep(1000);
        std::lock_guard<std::mutex> lock(jstate_mtx);
        if (jstate_updated) {
            LOG_INFO_STREAM() << "joint_states update it" << LOG_END;
            break;
        }
    }

    return jstate;
}

double MotionRequestServer::getIntroductionTrajMinSec(int start_step, const sensor_msgs::msg::JointState &jstate) {
    double min_sec = 0.0;
    if (!cur_miter->hasElement(start_step)) {
        return 0;
    }

    //ファイル読み込みを待つ
    while (!cur_miter->valueAvailable()) {
        usleep(1000);
    }
    auto data = cur_miter->getValue();

    //全関節が速度リミットを満たせる最小の開始ステップを求める
    for (size_t idx = 0; idx < joint_names.size(); ++idx) {
        auto jname = joint_names[idx];
        if (joint_limits.count(jname) == 0) {
            continue;
        }
        JointLimitType jlimit = joint_limits[jname];

        //現在の状態を拾ってくる
        JointState2 state1;
        auto itr_jstate = std::find(jstate.name.begin(), jstate.name.end(), jname);
        if (itr_jstate == jstate.name.end()) {
            continue;
        }
        auto idx_jstate = std::distance(jstate.name.begin(), itr_jstate);
        state1.pos = jstate.position[idx_jstate];
        state1.vel = 0; //jstate.velocity[idx_jstate];//ゼロ想定なので、ゼロを入れておく
        state1.acc = 0;

        //目標状態を拾ってくる
        bool found = false;
        JointState2 state2;
        for (auto &arm_tgt : data.arm_tgt) {
            if (arm_tgt.tgt_state.count(jname) == 0) {
                continue;
            }
            state2 = arm_tgt.tgt_state[jname];
            found = true;
            break;
        }
        if (!found) {
            continue;
        }

        auto pdiff = state2.pos - state1.pos;
        auto vdiff = state2.vel - state1.vel;
        if (dbl_equal(pdiff, 0) && dbl_equal(vdiff, 0)) {
            continue;
        }

        //とりあえず速度ゼロになる時間をデフォルトとして扱う
        double diff_sec = (15.0 * fabs(pdiff) / jlimit.velocity) / 8.0;
        if (min_sec < diff_sec) {
            min_sec = diff_sec;
        }

        //速度が最大(最小)の時の時刻を求める
        double prev_vel = std::numeric_limits<double>::max();
        while (1) {
            double cur_vel = 0;

#if 1//解析法(初期速度・加速度がゼロの場合のみOK)
            double a0, a1, a2, a3, a4, a5;
            get_spline_coeff(min_sec, state1, state2, a0, a1, a2, a3, a4, a5);

            if (9 * a4 * a4 - 30 * a3 * a5 >= 0) {
                //速度最大の時間と、その時の速度を求めて、cur_velを最大速度で更新する
                double b = std::sqrt(9 * a4 * a4 - 30 * a3 * a5);
                double t1 = (-3 * a4 + b) / (10 * a5);
                double t2 = (-3 * a4 - b) / (10 * a5);
                if (0 < t1 && t1 < min_sec) { //時間が範囲外の場合は無視
                    double pos, vel, acc, jerk;
                    interpolate(t1, min_sec, state1, state2, pos, vel, acc, jerk);
                    if (fabs(cur_vel) < fabs(vel)) {
                        cur_vel = vel;
                    }
                }

                if (0 < t2 && t2 < min_sec) { //時間が範囲外の場合は無視
                    double pos, vel, acc, jerk;
                    interpolate(t2, min_sec, state1, state2, pos, vel, acc, jerk);
                    if (fabs(cur_vel) < fabs(vel)) {
                        cur_vel = vel;
                    }
                }
            }
#else
            //ニュートン法
            double prev_vel_tmp = 0;
            double cur_sec = diff_sec/2.0; //探索初期値
            while (1) {
                double pos, acc, jerk;

                interpolate(cur_sec, min_sec, state1, state2, pos, cur_vel, acc, jerk);
                cur_sec = cur_sec - (acc / jerk);

                //収束チェック
                if (fabs(cur_vel - prev_vel) < 0.0001) {
                    if (cur_sec < 0) {
                        cur_vel = state1.vel;
                    } else if (cur_sec > min_sec) {
                        cur_vel = state2.vel;
                    }
                    break;
                }

                prev_vel_tmp = cur_vel;
            }
#endif

            double cur_vel_abs = fabs(cur_vel);
            double prev_vel_abs = fabs(prev_vel);

            if (cur_vel_abs > jlimit.velocity) {
                if (jlimit.velocity >= prev_vel_abs) {
                    //今回リミットを超えるようになった場合は前回値で終了
                    min_sec -= 0.1;
                    break;
                }

                min_sec += 0.1; //速度が早ければ時間を増やす
                if (min_sec >= 10) { //10[s]を超えたら、失敗
                    break;
                }
            } else {
                if (jlimit.velocity < prev_vel_abs) {
                    //今回リミットを超えなくなった場合は終了
                    break;
                }

                min_sec -= 0.1; //速度が遅ければ時間を減らす
                if (min_sec < 0) {
                    break;
                }
            }

            prev_vel = cur_vel;
        }
    }

    if (min_sec < 0) {
        min_sec = 0;
    }

    return min_sec;
}

motion_editor2::ArmTarget2 MotionRequestServer::calcIntroductionTraj(int step, int step_st, int step_end, const sensor_msgs::msg::JointState &jstate, const std::vector<motion_editor2::ArmTarget2> &arm_tgts) {
    motion_editor2::ArmTarget2 tgt;
    tgt.step_end = step + 1;
    for (size_t idx = 0; idx < joint_names.size(); ++idx) {
        auto jname = joint_names[idx];

        auto itr_jstate = std::find(jstate.name.begin(), jstate.name.end(), jname);
        if (itr_jstate == jstate.name.end()) {
            continue;
        }
        //現在の値
        auto idx_jstate = std::distance(jstate.name.begin(), itr_jstate);
        JointState2 prev_jval;
        prev_jval.pos = jstate.position[idx_jstate];
        prev_jval.vel = 0; //jstate.velocity[idx_jstate];
        prev_jval.acc = 0;

        for (auto &arm_tgt : arm_tgts) {
            if (arm_tgt.tgt_state.count(jname) == 0) {
                continue;
            }
            //モーション開始位置
            JointState2 next_jval = arm_tgt.tgt_state.at(jname);
            JointState2 tgt_jval;

            //補間して目標値に詰める
            interpolate(step, dt_sec, step_st, step_end, prev_jval, next_jval, tgt_jval);

            JointLimitType jlimit = joint_limits[jname];
            if (tgt_jval.pos < jlimit.lower || tgt_jval.pos > jlimit.upper) {
                LOG_ERROR_STREAM() << "exceed joint pos limit : " << jname << " pos: " << tgt_jval.pos << " limit: " << jlimit.lower << " " << jlimit.upper << LOG_END;
            }
            if (fabs(tgt_jval.vel) > jlimit.velocity) {
                LOG_ERROR_STREAM() << "exceed joint velocity limit : " << jname << " vel: " << tgt_jval.vel << " limit: " << jlimit.velocity << LOG_END;
            }

            tgt.tgt_state[jname] = tgt_jval;
            break;
        }
    }
    return tgt;
}

void MotionRequestServer::sendThread() {
    while (1) {
        std::unique_lock < std::mutex > lk(mtx);
        cond.wait(lk);
        if (shutdown.load()) {
            break;
        }

        if (!cur_motion || !cur_miter || cancel.load()) {
            continue;
        }
        

        req_buff->setPermissionRead(false);
        req_buff->clear();

        int send_size = 0;
        bool readable = false;
        double dt_sec = cur_miter->getStepTime();
        int step_max = cur_miter->getMaxStep();
        int data_step = start_step;

        auto jstate = getCurrentJointState();
        auto min_sec = getIntroductionTrajMinSec(start_step, jstate);

        int step_st = start_step - static_cast<int>(std::floor(min_sec / dt_sec));

        for (int step = step_st; step <= step_max; ++step) {
            if (cancel.load() || shutdown.load()) {
                break;
            }

            if (step < start_step) {
                data_step = start_step;
            } else {
                data_step = step;
            }

            if (cur_miter->hasElement(data_step)) {
                //ファイル読み込みを待つ
                while (!cur_miter->valueAvailable()) {
                    usleep(1000);
                }

                auto data = cur_miter->getValue();
                std::vector<motion_editor2::ArmTarget2> arm_tgt;

                if (step < start_step) {
                    auto tgt = calcIntroductionTraj(step, step_st, start_step, jstate, data.arm_tgt);
                    arm_tgt.push_back(tgt);
                } else {
                    arm_tgt = data.arm_tgt;
                }

                //ここでバッファの書き込み待ちが発生する
                send_size += sendArmTarget(dt_sec, step, control_period_sec, jtarget.tgtpos, arm_tgt, arm_info, stroke_converter_, req_buff);

                if ((send_size >= REQ_BUFF_SIZE / 2 || step >= step_max) && !readable) {
                    req_buff->setPermissionRead(true); //データが十分に溜まったら、読み出し可能にする
                    readable = true;
                }

            }

            //最後に空リクエストを入れておく
            if (step == step_max) {
                Request req;
                req.msid = -1;
                req.step = std::round(step_max * (dt_sec / control_period_sec));
                req_buff->writeFromA(req);
            }

        }
    }

}

rclcpp_action::GoalResponse MotionRequestServer::goal_received_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PlayAction::Goal> goal){
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionRequestServer::goal_cancelled_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PlayAction>> goal_handle){
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionRequestServer::goal_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<PlayAction>> goal_handle){
    std::thread{std::bind(&MotionRequestServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}


void MotionRequestServer::update_hardware(const seed_ros2_controller::command_interface::OtherCommandInterface &hw, double control_period_sec) {
    this->control_period_sec = control_period_sec;
    arm_info = createArmInfo(hw);
    this->joint_names = arm_info.getJointNames();
    jtarget.tgtpos.resize(joint_names.size(), 0);
    stroke_converter_->setJointNames(joint_names);
}

void MotionRequestServer::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, reqBuffType *req_buff, respBuffType *resp_buff, std::atomic<bool> *stop_request, const std::string &converter_lib) {
    this->req_buff = req_buff;
    this->resp_buff = resp_buff;
    this->stop_request = stop_request;
    this->node = node;

    switch_client_ =  node->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
    list_client_ =  node->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");

    using namespace std::placeholders;
    load_srv = node->create_service < motion_player_msgs::srv::Load > (std::string(node->get_name()) + "/load", std::bind(&MotionRequestServer::load, this, _1, _2));
    
    play_as = rclcpp_action::create_server<PlayAction, std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>(
            node,
            std::string(node->get_name()) + "/play",
            std::bind(&MotionRequestServer::goal_received_callback, this, _1, _2),
            std::bind(&MotionRequestServer::goal_cancelled_callback, this, _1),
            std::bind(&MotionRequestServer::goal_accepted_callback, this, _1));
    
    LOG_INFO_STREAM() << "Converter Library : " << converter_lib << LOG_END;
    
//    if (seed_nh.getParam("robot_model_plugin", robot_model_plugin_name)) {
    try {
        stroke_converter_ = converter_loader_.createSharedInstance(converter_lib);
        stroke_converter_->initialize();
    } catch (pluginlib::PluginlibException &ex) {
        LOG_ERROR_STREAM() << "The plugin failed to load : " << ex.what() << LOG_END;
    }
//    } else {
//        LOG_ERROR_STREAM() << "robot_model_plugin param is not set" << LOG_END;
//    }
//

    std::atomic<bool> received = false;
    rclcpp::QoS custom_qos(rclcpp::KeepLast(1), rmw_qos_profile_default);
    custom_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    auto desc_sub_ = node->create_subscription < std_msgs::msg::String > ("/robot_description", custom_qos, [&](const std_msgs::msg::String::ConstSharedPtr &description) {
        joint_limits.clear();
        auto robot_model = urdf::parseURDF(description->data);
        extractJointLimits(*robot_model, joint_limits);
        received.store(true);
    });

    while (!received.load()) {
        usleep(1000000);
    }


    send_thread = std::thread(&MotionRequestServer::sendThread, this);
}

}
