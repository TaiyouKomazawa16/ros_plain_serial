/*
 * File:    Structure.h
 * 
 * This file is distributed under the MIT license. See "LICENSE" for text.
 * 
 * Author:  TaiyouKomazawa
 */

#ifndef STRUCTURE_H_
#define STRUCTURE_H_

#include <stdint.h>

#include "MessageID.h"

class StructMem
{
public:

    virtual inline int size() = 0;
    virtual inline uint8_t *ptr() = 0;
    virtual inline message_t msg_id() = 0;
    
};

#endif
