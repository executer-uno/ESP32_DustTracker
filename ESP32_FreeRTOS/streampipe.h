// add classic C stream pipe operator to arduino C

#pragma once
#define EOL "\r\n"

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } 
