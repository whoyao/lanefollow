//
// Created by huyao on 18-1-27.
//

#ifndef PATH_PLANNING_LOG_H
#define PATH_PLANNING_LOG_H

#include <iostream>

#define OUTPUT(x,y) std::cout << (x) << (y) << std::endl
#define DEBUG(x) std::cout << (x) << std::endl

#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */

#define INFO(x)     OUTPUT(BLUE,x)
#define WARNING(x)  OUTPUT(YELLOW,x)
#define ERROR(x)    OUTPUT(RED,x)
#define GREEN_INFO(x)    OUTPUT(MAGENTA,x)


#endif //PATH_PLANNING_LOG_H
