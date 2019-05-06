

#include "AP_TestLib.h"


#define PARAM1_DEFAULT 0.0f



const AP_Param::GroupInfo AP_TestLib::var_info[] = {

    // @Param: _PARAM1
    // @DisplayName: param1 name
    // @Description: param1 description
    // @Units: m
    // @Range: 0.01 2
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_PARAM1", 0, AP_TestLib, _param1, PARAM1_DEFAULT),


    AP_GROUPEND
};






AP_TestLib::AP_TestLib() {
	AP_Param::setup_object_defaults(this, var_info);
}






