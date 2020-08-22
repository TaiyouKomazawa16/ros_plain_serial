#ifndef PLAIN_SERIAL_H_
#define PLAIN_SERIAL_H_

#define BUFFER_LEN 9

#include <Arduino.h>

class PlainSerial
{
public:
    PlainSerial(HardwareSerial *dev)
    {
        _dev = dev;
    }

    double read(int *id);
    void write_32bit(int8_t id, int32_t val);
    void write_float(int8_t id, double val);
protected:
    typedef struct BufferType{
        uint8_t str[10] ={};
        int len = 0;
    }buffer_t;

    typedef enum ControlCharType{
        HEADER=':',
        END='\n',
    }ctrl_char_t;

    typedef enum MessageType{
        INT32 =0,
        FLOAT32,
        BOOL,
    }message_t;

    inline void _write(int8_t id, message_t msg_id, uint8_t *c, int len);
    int _read(buffer_t *buff);
    double _decode(buffer_t *buff, int *id);
private:
    HardwareSerial *_dev;
    inline void _write_once(uint8_t c, int *data_sum);
};


double PlainSerial::read(int *id)
{
    buffer_t buffer;
    if(!_read(&buffer)){
        double result = _decode(&buffer, id);
        return result;
    }
    else{
        *id = -1;
        return 0;
    }
}

void PlainSerial::write_32bit(int8_t id, int32_t val)
{
    union{
        uint8_t c[4];
        int32_t i;
    } i_to_c;
    i_to_c.i = val;

    _write(id, INT32, i_to_c.c, 4);
}  


void PlainSerial::write_float(int8_t id, double val)
{
    union{
        uint8_t c[4];
        double f;
    } f_to_c;
    f_to_c.f = val;

    _write(id, FLOAT32, f_to_c.c, 4);
}

inline void PlainSerial::_write_once(uint8_t c, int *data_sum)
{
    _dev->write(c);
    *data_sum += c;
}

inline void PlainSerial::_write(int8_t id, PlainSerial::message_t msg_id, uint8_t *c, int len)
{
    int data_sum = 0;
    _write_once(HEADER, &data_sum);
    _write_once(id, &data_sum);
    _write_once(msg_id, &data_sum);
    for (int i = 0; i < len; i++) {
        _write_once(c[i], &data_sum);
    }
    data_sum += END;
    _dev->write((uint8_t)(data_sum & 0xFF)); //下位ビット8bitを送信
    _dev->write(END);
    _dev->flush();
}

int PlainSerial::_read(PlainSerial::buffer_t *buff){
    if(_dev->available() >= BUFFER_LEN){
        bool got_frame = false;
        uint8_t check_sum = 0;
        int data_sum = 0;
        while(1){
            uint8_t data = _dev->read();
            data_sum += data;
            if(buff->len >= BUFFER_LEN){
                buff->len = 0;
                got_frame = false;
                return -1;
            }
            switch(data){
                case HEADER:
                    got_frame = true;
                break;
                case END:
                    check_sum = buff->str[buff->len-1];
                    data_sum -= check_sum;
                    if((data_sum & 0xFF) == check_sum){
                        buff->str[buff->len] = END;
                        buff->len++;
                        return 0;
                    }else{
                        buff->len = 0;
                        return -1;
                    }
                break;
                default:
                    buff->str[buff->len] = data;
                    buff->len++;
                break;
            };
        }
    }
    return -1;
}

double PlainSerial::_decode(PlainSerial::buffer_t *buff, int *id){
    if(buff->len == 0){
        *id = -1;
        return 0;
    }
    *id = buff->str[0];
    uint8_t *c = &(buff->str[2]);
    switch(buff->str[1]){
        case INT32:
            union{
                uint8_t c[4];
                int32_t i;
            } i_to_c;
            for(int i = 0; i < 4; i++)
                i_to_c.c[i] = c[i];
            return (double)i_to_c.i;
        break;
        case FLOAT32:
            union{
                uint8_t c[4];
                double f;
             } f_to_c;
             for(int i = 0; i < 4; i++)
                f_to_c.c[i] = c[i];
             return f_to_c.f;
        break;
    };
}

#endif
