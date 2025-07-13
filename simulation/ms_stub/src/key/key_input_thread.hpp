#pragma once
#include "key_input_mode.hpp"

class KeyInputThread{
public:
    KeyInputThread(std::vector<KeyInputMode*> modes):modes(modes){

    }

    ~KeyInputThread(){
        is_shutdown.store(true);
        if(th.joinable()){
            th.join();
        }
    }

    void run(){
        th = std::thread(&KeyInputThread::exec,this);
    }

    void exec(){

        fd_set fds, readfds;
        int fd_in = fileno(stdin);
        int maxfd = fd_in;

        FD_ZERO(&readfds);
        FD_SET(fd_in, &readfds);

        //タイムアウト
        struct timeval tv_to;

        displayMenu();
        fflush(stdout);

        while(!is_shutdown.load()){
            std::string input_str;
            char buff[256];

            memcpy(&fds, &readfds, sizeof(fd_set));
            tv_to.tv_sec = 1;
            tv_to.tv_usec = 0;
            select(maxfd+1, &fds, NULL, NULL, &tv_to);

            if (FD_ISSET(fd_in, &fds)) {
                std::cin.getline(buff, 256);
                input_str = std::string(buff);
                if(input_str.empty()){
                    displayMenu();
                    fflush(stdout);
                    continue;
                }
            } else {
                continue;
            }

            execute(input_str);

            displayMenu();
            fflush(stdout);
        }
    }

    void displayMenu(){
        if(current_mode == MODE_SELECT){
            displayModeSelectionMenu();
        }else{
            modes[current_mode]->displayMenu();
            std::cout<<"Esc + Enter : back"<<std::endl;
        }
    }

    void displayModeSelectionMenu() {
        std::cout << "[select mode]" << std::endl;
        int idx = 0;
        for (auto mode : modes) {
            auto name = mode->getName();
            std::cout << "(" << idx++ << ") : " << name << std::endl;
        }
    }

    void execute(std::string str){
        try {
            if(str[0] == 0x1b){
                if (current_mode != MODE_SELECT) {
                    modes[current_mode]->exit();
                    current_mode = MODE_SELECT;
                }
            }else if (current_mode == MODE_SELECT) {
                int new_mode = std::stoi(str);
                if (new_mode >= 0 && new_mode < static_cast<int>(modes.size())) {
                    current_mode = new_mode;
                    modes[current_mode]->entry();
                }
            } else {
                auto mode = modes[current_mode];
                mode->execute(str);
            }
        } catch (...) {

        }
    }

private:
    std::atomic<bool> is_shutdown = false;
    static constexpr int MODE_SELECT = -1;
    std::vector<KeyInputMode*> modes;
    int current_mode = -1;
    std::thread th;
};
