/*
   Implementation of MaxBotix MB1242 sonar driver

   Copyright (C) 2016 Simon D. Levy and Matt Lubas

   This file is part of MB1242.

   MB1242 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   MB1242 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with MB1242.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "roverbot/MB1242.h"


void MB1242::begin(uint8_t address, uint8_t bus)
{
    _addr = address;
    i2cCtrl = I2Cdev::getInstance();
}

void MB1242::changeAddress(uint8_t oldaddr, uint8_t newaddr)
{
    // cpi2c_writeRegister_16_8(oldaddr, 0xAAA5, newaddr<<1); // support seven-bit addressing
    uint8_t data[]={9,9,9,5,newaddr};
    i2cCtrl->writeBytes(oldaddr, 0, 5, data);
}

void MB1242::requestDistance(void)
{
    // I2Cdev::writeByte(_addr, 0x00, 0x51);
    i2cCtrl->writeByte(_addr, 224, 81);
    // cpi2c_writeRegister(_addr, 0x00, 0x51);
}

int MB1242::getDistance(void)
{
    uint8_t tmp[2];
    // i2cCtrl->readWord(_addr, 225,&tmp);
    i2cCtrl->readBytes(_addr,224,2,tmp);
    // uint16_t tmp = cpi2c_readRegister_8_16(_addr, 0x00);

    // Reverse endianness to get distance
    // uint16_t distance = (tmp>>8) | (tmp<<8);
    int distance = tmp[0]*256+tmp[1];
    return distance;

}
