#ifndef __LOADER_H__
#define __LOADER_H__

#include "mapper.h"

class Loader : public Mapper
{
public:
    /**
     * @brief Construct a new Loader object
     * 
     */
    Loader();

    /**
     * @brief Destroy the Loader object
     * 
     */
    ~Loader();

    /**
     * @overload spin()
     * 
     */
    virtual void spin();
};

#endif // __LOADER_H__