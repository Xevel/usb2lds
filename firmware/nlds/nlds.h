/*
 * nlds.h
 *
 * Created: 07/03/2014 09:13:55
 *  Author: Xevel
 */ 


#ifndef NLDS_H_
#define NLDS_H_

void        nlds_parse(uint8_t c); // parse data from the LDS
uint8_t     nlds_rpm_updated_get(); // 1 if the RPM has been updated
void        nlds_rpm_updated_clear(); // clears the RPM Updated Flag 
uint16_t    nlds_rpm_get(); // get the latest RPM value received

#endif /* NLDS_H_ */