/*--------------------------------------------------------------------
This file is part of the Arduino M590 library.

The Arduino M590 library is free software: you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

The Arduino M590 library is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with The Arduino M590 library.  If not, see
<http://www.gnu.org/licenses/>.
--------------------------------------------------------------------*/

#ifndef ModemDebug_H
#define ModemDebug_H

#include <stdio.h>

// Change _MODEMLOGLEVEL_ to set tracing and logging verbosity
// 0: DISABLED: no logging
// 1: ERROR: errors
// 2: WARN: errors and warnings
// 3: INFO: errors, warnings and informational (default)
// 4: DEBUG: errors, warnings, informational and debug

#define _MODEMLOGLEVEL_ 3


#define LOGERROR(x)      if(_MODEMLOGLEVEL_>0) { Serial.print(F("[M590] ")); Serial.println(x); }
#define LOGERROR1(x,y)   if(_MODEMLOGLEVEL_>2) { Serial.print(F("[M590] ")); Serial.print(x); Serial.print(F(" ")); Serial.println(y); }
#define LOGWARN(x)       if(_MODEMLOGLEVEL_>1) { Serial.print(F("[M590] ")); Serial.println(x); }
#define LOGWARN1(x,y)    if(_MODEMLOGLEVEL_>2) { Serial.print(F("[M590] ")); Serial.print(x); Serial.print(F(" ")); Serial.println(y); }
#define LOGINFO(x)       if(_MODEMLOGLEVEL_>2) { Serial.print(F("[M590] ")); Serial.println(x); }
#define LOGINFO1(x,y)    if(_MODEMLOGLEVEL_>2) { Serial.print(F("[M590] ")); Serial.print(x); Serial.print(F(" ")); Serial.println(y); }
#define LOGINFO2(x,y,z)  if(_MODEMLOGLEVEL_>2) { Serial.print(F("[M590] ")); Serial.print(x); Serial.print(F(" ")); Serial.print(y); Serial.print(F(" ")); Serial.println(z);}

#define LOGDEBUG(x)      if(_MODEMLOGLEVEL_>3) { Serial.println(x); }
#define LOGDEBUG0(x)     if(_MODEMLOGLEVEL_>3) { Serial.print(x); }
#define LOGDEBUG1(x,y)   if(_MODEMLOGLEVEL_>3) { Serial.print(x); Serial.print(F(" ")); Serial.println(y); }
#define LOGDEBUG2(x,y,z) if(_MODEMLOGLEVEL_>3) { Serial.print(x); Serial.print(F(" ")); Serial.print(y); Serial.print(F(" ")); Serial.println(z); }


#endif

/* vim: set ft=cpp ai ts=2 sts=2 et sw=2 sta nowrap nu : */
