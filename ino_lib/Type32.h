/*
 * File:    Type32.h
 * 
 * This file is distributed under the MIT license. See "LICENSE" for text.
 * 
 * Author:  TaiyouKomazawa
 */

#ifndef TYPE_32_H_
#define TYPE_32_H_

#include <stdint.h>

#include "MessageID.h"

#include "Structure.h"

class Int32 : public StructMem
{
public:

    inline int32_t val(int32_t val){
        return _data.val = val;
    }
    
    inline int32_t val(){
        return _data.val;
    }

    virtual inline int size(){ return MAX_INT32_SIZE; }
    virtual inline uint8_t *ptr(){ return _data.ptr; }

    virtual inline message_t msg_id(){
        return INT32;
    }

private:
    enum{
        MAX_INT32_SIZE = 4, //byte
    };

    typedef union _DataFrameType{
        uint8_t ptr[MAX_INT32_SIZE];
        struct{
            int32_t val;
        };
    }_data_frame_t;

    _data_frame_t _data;
};

class Float32 : public StructMem
{
public:

    inline float val(float val){
        return _data.val = val;
    }
    
    inline float val(){
        return _data.val;
    }

    virtual inline int size(){ return MAX_FLOAT32_SIZE; }
    virtual inline uint8_t *ptr(){ return _data.ptr; }

    virtual inline message_t msg_id(){
        return FLOAT32;
    }

private:
    enum{
        MAX_FLOAT32_SIZE = 4, //byte
    };

    typedef union _DataFrameType{
        uint8_t ptr[MAX_FLOAT32_SIZE];
        struct{
            float val;
        };
    }_data_frame_t;

    _data_frame_t _data;
};

#endif
