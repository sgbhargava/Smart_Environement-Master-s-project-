/*
 * common_includes.h
 *
 *  Created on: Jul 17, 2016
 *      Author: SPIRIT
 */

#ifndef L5_APPLICATION_EXAMPLES_COMMON_INCLUDES_HPP_
#define L5_APPLICATION_EXAMPLES_COMMON_INCLUDES_HPP_
#include "string"
typedef struct sysStatStruct
{
	std::string deviceID;
	std::string deviceTemp;
	std::string deviceBat;
	std::string deviceCPU;
	std::string deviceMem;
}sysStat;



#endif /* L5_APPLICATION_EXAMPLES_COMMON_INCLUDES_HPP_ */
