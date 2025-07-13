#pragma once
#include <string>

class KeyInputMode{
public:
    virtual ~KeyInputMode(){

    }

    virtual void execute(std::string input){

    }

    virtual void entry(){

    }

    virtual void exit(){

    }

    virtual std::string getName(){
        return "";
    }

    virtual void displayMenu(){

    }
};
