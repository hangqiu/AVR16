#include "serverHandler.hpp"

serverHandler::serverHandler()
{
    //ctor
}
serverHandler::serverHandler(utility::string_t url):m_listener(url)
{
    m_listener.support(methods::GET, std::bind(&serverHandler::handle_get, this, std::placeholders::_1));
    m_listener.support(methods::PUT, std::bind(&serverHandler::handle_put, this, std::placeholders::_1));
    m_listener.support(methods::POST, std::bind(&serverHandler::handle_post, this, std::placeholders::_1));
    m_listener.support(methods::DEL, std::bind(&serverHandler::handle_delete, this, std::placeholders::_1));

}
serverHandler::~serverHandler()
{
    //dtor
}

void serverHandler::handle_error(pplx::task<void>& t)
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
void serverHandler::handle_get(http_request message)
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
    auto paths = http::uri::split_path(http::uri::decode(message.relative_uri().path()));
    for (auto p = paths.begin();p!=paths.end();p++){
        int id = stoi(*p);
        std::cout << id << endl;
    }
    message.relative_uri().path();
	//Dbms* d  = new Dbms();
    //d->connect();


      concurrency::streams::fstream::open_istream(U("/home/hang/AVRComm/cam0/dynamicPCFrame1.yml"), std::ios::in).then([=](concurrency::streams::istream is)
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
void serverHandler::handle_post(http_request message)
{
    ucout <<  message.to_string() << endl;


     message.reply(status_codes::OK,message.to_string());
    return ;
};

//
// A DELETE request
//
void serverHandler::handle_delete(http_request message)
{
     ucout <<  message.to_string() << endl;

        string rep = U("WRITE YOUR OWN DELETE OPERATION");
      message.reply(status_codes::OK,rep);
    return;
};


//
// A PUT request 
//
void serverHandler::handle_put(http_request message)
{
    ucout <<  message.to_string() << endl;
     string rep = U("WRITE YOUR OWN PUT OPERATION");
     message.reply(status_codes::OK,rep);
    return;
};
