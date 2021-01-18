#ifndef __SUPERVISOR_H__
#define __SUPERVISOR_H__

#include "mapper.h"

class Supervisor : public Mapper
{

public:
    Supervisor();
    ~Supervisor();

    virtual void spin();
};


#endif // __SUPERVISOR_H__