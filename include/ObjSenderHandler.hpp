#ifndef HANDLER_H
#define HANDLER_H
#include <iostream>
#include "stdafx.hpp"
//#include "../dbms/include/Dbms.h"

using namespace std;
using namespace web;
using namespace http;
using namespace utility;
using namespace http::experimental::listener;


class ObjSenderHandler
{
    public:
    ObjSenderHandler();
    ObjSenderHandler(utility::string_t url);
        virtual ~ObjSenderHandler();

        pplx::task<void>open(){return m_listener.open();}
        pplx::task<void>close(){return m_listener.close();}

    protected:

    private:

//        ObjSender* mObjSender;

        void handle_get(http_request message);
        void handle_get_fromMemory(http_request message);


        void handle_put(http_request message);
        void handle_post(http_request message);
        void handle_delete(http_request message);
        void handle_error(pplx::task<void>& t);
        http_listener m_listener;
};

#endif // HANDLER_H


