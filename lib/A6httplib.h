#ifndef A6httplib_h
#define A6httplib_h

#include <Arduino.h>
//#include <A6lib.h>

class A6httplib
{
private:
    String _host;
    int _port;
    String _path;
    String _apn;
    A6lib *_A6l;

public:
    A6httplib();
    A6httplib(A6lib *);

    ~A6httplib();

    bool ConnectGPRS(String apn);

    bool AddHeader(String header);
    bool Post(String host, String path, String body);
    String Get(String host, String path);
    String getResponseData(String);
};
#endif


