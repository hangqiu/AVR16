#include "ObjSenderHandler.hpp"
#include "globals.hpp"

ObjSenderHandler::ObjSenderHandler()
{
    //ctor
}
ObjSenderHandler::ObjSenderHandler(utility::string_t url):m_listener(url)
{
    m_listener.support(methods::GET, std::bind(&ObjSenderHandler::handle_get, this, std::placeholders::_1));
    m_listener.support(methods::PUT, std::bind(&ObjSenderHandler::handle_put, this, std::placeholders::_1));
    m_listener.support(methods::POST, std::bind(&ObjSenderHandler::handle_post, this, std::placeholders::_1));
    m_listener.support(methods::DEL, std::bind(&ObjSenderHandler::handle_delete, this, std::placeholders::_1));

}
ObjSenderHandler::~ObjSenderHandler()
{
    //dtor
}

void ObjSenderHandler::handle_error(pplx::task<void>& t)
{
    try
    {
        t.get();
    }
    catch(...)
    {
        // Ignore the error, Log it if a logger is available
    }
}


//
// Get Request 
//
void ObjSenderHandler::handle_get(http_request message)
{

//    concurrency::streams::fstream::open_istream(U("/home/hang/AVRComm/cam0/dynamicPCFrame1.yml"), std::ios::in)
//            .then([=](concurrency::streams::istream ifs)
//                  {
//                      message.reply(status_codes::OK, ifs, U("text/html"));
//                  });


    ucout <<  message.to_string() << endl;
/// parse query
//    auto query = http::uri::split_query(http::uri::decode(message.relative_uri().query()));
//    auto cntEntry = query.find(U("id"));
//
//    if (cntEntry != query.end() && !cntEntry->second.empty()){
//        std::cout << "got parameter\n";
//        string id = cntEntry->second;
//        std::cout << id;
//    }
    /// parse path
    int id =0;
    auto paths = http::uri::split_path(http::uri::decode(message.relative_uri().path()));
    for (auto p = paths.begin();p!=paths.end();p++){
        id = stoi(*p);
        std::cout << id << endl;
    }
    message.relative_uri().path();
	//Dbms* d  = new Dbms();
    //d->connect();
    char dir[100];
//    id = id < FRAME_ID ? id : FRAME_ID-1;
//    sprintf(dir, "%s/cam0/Frame%d.yml", commPath.c_str(), id);
    sprintf(dir, "%s/cam0/%s_%s_%s_%d.yml",commPath.c_str(),
            FRAME.c_str(), TCW.c_str(), TIMESTAMP.c_str(), FRAME_ID-1);
    std::cout << "returning " << dir << endl;

      concurrency::streams::fstream::open_istream(U(dir), std::ios::in).then([=](concurrency::streams::istream is)
    {
        message.reply(status_codes::OK, is,  U("text/html"))
		.then([](pplx::task<void> t)
		{
			try{
				t.get();
			}
			catch(...){
				//
			}
	    });
    }).then([=](pplx::task<void>t)
	{
		try{
			t.get();
		}
		catch(...){
			message.reply(status_codes::InternalError,U("INTERNAL ERROR "));
		}
	});

    return;

};

//
// A POST request
//
void ObjSenderHandler::handle_post(http_request message)
{
    ucout <<  message.to_string() << endl;


     message.reply(status_codes::OK,message.to_string());
    return ;
};

//
// A DELETE request
//
void ObjSenderHandler::handle_delete(http_request message)
{
     ucout <<  message.to_string() << endl;

        string rep = U("WRITE YOUR OWN DELETE OPERATION");
      message.reply(status_codes::OK,rep);
    return;
};


//
// A PUT request 
//
void ObjSenderHandler::handle_put(http_request message)
{
    ucout <<  message.to_string() << endl;
     string rep = U("WRITE YOUR OWN PUT OPERATION");
     message.reply(status_codes::OK,rep);
    return;
};
