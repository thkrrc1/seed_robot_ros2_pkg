#pragma once

#include <seed_ros2_controller/command_interface/other_command/other_cmd_buff.hpp>
#include "serial_com.hpp"
#include "header.hpp"

enum PARSE_RESULT {
    CMD_INVALID,
    CMD_PARSED,
    CMD_OTHER,
};

class AeroCommand {

public:
    AeroCommand() {
    }

    virtual ~AeroCommand() {
    }

    virtual size_t getExpectSize(const Header *header) = 0;
    virtual PARSE_RESULT parseData(uint8_t *recvd) = 0;

    virtual void sendPGET(SerialCommunication &serial_com, int msid) = 0;
    virtual void sendMOVE(SerialCommunication &serial_com, int msid, const double &tgt_time_sec, int16_t *data) = 0;
    virtual void sendTURN(SerialCommunication &serial_com, int msid, int16_t *data) = 0;

    virtual int16_t getpos(int msid, int joint) = 0;
    virtual uint16_t getstatus(int msid) = 0;

    //非RT
    virtual std::string getProtocol() = 0;

private:

};

class AeroDriver {
public:

    AeroDriver();
    ~AeroDriver();

    void setCommand(AeroCommand *command_base) {
        this->command_base = command_base;
    }

    bool openPort(std::string _port, unsigned int _baud_rate);
    void closePort();

    void read();

    bool connected();

    size_t getExpectSize(const Header *header) {
        if (command_base) {
            return command_base->getExpectSize(header);
        }
        return 0;
    }

    PARSE_RESULT parseData(uint8_t *recvd) {
        if (command_base) {
            return command_base->parseData(recvd);
        }
        return PARSE_RESULT::CMD_INVALID;
    }

    void sendPGET(int msid) {
        if (command_base) {
            command_base->sendPGET(serial_com, msid);
        }
    }

    void sendMOVE(int msid, const double &tgt_time_sec, int16_t *data) {
        if (command_base) {
            command_base->sendMOVE(serial_com, msid, tgt_time_sec, data);
        }
    }

    void sendTURN(int msid, int16_t *data) {
        if (command_base) {
            command_base->sendTURN(serial_com, msid, data);
        }
    }

    int16_t getpos(int msid, int joint) {
        if (command_base) {
            return command_base->getpos(msid, joint);
        }
        return 0;
    }

    uint16_t getstatus(int msid) {
        if (command_base) {
            return command_base->getstatus(msid);
        }
        return 0;
    }

    void sendOtherCommands(int msid, BuffList &cmds);
    void getOtherCommands(int msid, BuffList &cmds);

private:
    AeroCommand *command_base = nullptr;
    BuffList other_cmd_recvd;
    SerialCommunication serial_com;
};

