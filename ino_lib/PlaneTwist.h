/*
 * File:    PlaneTwist.h
 * 
 * This file is distributed under the MIT license. See "LICENSE" for text.
 * 
 * Author:  TaiyouKomazawa
 */

#ifndef PLANE_TWIST_H_
#define PLANE_TWIST_H_

#include <stdint.h>

#include "MessageID.h"

#include "Structure.h"

class PlaneTwist : public StructMem
{
public:

    inline float x(float x){
        return _data.x = x;
    }
    inline float y(float y){
        return _data.y = y;
    }
    inline float yaw(float yaw){
        return _data.yaw = yaw;
    }
    
    inline float x(){
        return _data.x;
    }
    inline float y(){
        return _data.y;
    }
    inline float yaw(){
        return _data.yaw;
    }

    virtual inline int size(){ return MAX_P_TWIST_SIZE; }
    virtual inline uint8_t *ptr(){ return _data.ptr; }

    virtual inline message_t msg_id(){
        return PLANE_TWIST;
    }

private:
    enum{
        MAX_P_TWIST_SIZE = 12, //byte
    };

    typedef union _DataFrameType{
        uint8_t ptr[MAX_P_TWIST_SIZE];
        struct{
            float x;
            float y;
            float yaw;
        };
    }_data_frame_t;

    _data_frame_t _data;
};

#endif
