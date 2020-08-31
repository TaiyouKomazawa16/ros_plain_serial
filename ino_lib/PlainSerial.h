/*
 * File:    PlainSerial.h
 * 
 * This file is distributed under the MIT license. See "LICENSE" for text.
 * 
 * Author:  TaiyouKomazawa
 */

#ifndef PLAIN_SERIAL_H_
#define PLAIN_SERIAL_H_

#define BUFFER_LEN 8

#define FRAME_LEN 4

#include <Arduino.h>

#include "Structure.h"
#include "MessageID.h"

class PlainSerial
{
public:
    PlainSerial(HardwareSerial *dev)
    {
        _dev = dev;
    }
    
    int read(int target_id, StructMem *str)
    {
        int size = FRAME_LEN+str->size();
        if(_dev->available() > size){
            uint8_t check_sum = 0;
            int data_sum = 0;

            if (_read_once(&data_sum) != HEADER){
                return -2;
            }
            if (_read_once(&data_sum) != target_id){
                return -3;
            }
            if (_read_once(&data_sum) != str->msg_id()){
                return -4;
            }

            for(int i = 0; i < str->size(); i++){
                uint8_t c;
                str->ptr()[i] = _read_once(&data_sum);
            }

            check_sum = _dev->read();

            if (_read_once(&data_sum) != END){
                return -5;
            }

            if((data_sum & 0xFF) == check_sum){
                return 0;
            }
            else{
                return -6;
            }
        }
        return -1;
    }

    void write(int id, StructMem *str)
    {
        int data_sum = 0;
        _write_once(HEADER, &data_sum);
        _write_once(id, &data_sum);
        _write_once(str->msg_id(), &data_sum);
        for (int i = 0; i < str->size(); i++)
        {
            _write_once(str->ptr()[i], &data_sum);
        }
        data_sum += END;
        _dev->write((uint8_t)(data_sum & 0xFF)); //下位ビット8bitを送信
        _dev->write(END);
        _dev->flush();
    }

protected:

    typedef enum ControlCharType{
        HEADER=':',
        END='\n',
    }ctrl_char_t;
    
private:
    HardwareSerial *_dev;
    inline void _write_once(uint8_t c, int *data_sum)
    {
        _dev->write(c);
        *data_sum += c;
    }

    inline uint8_t _read_once(int *data_sum)
    {
        uint8_t c = _dev->read();
        *data_sum += c;
        return c;
    }
};


#endif
