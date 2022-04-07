#ifndef __APP_EXCEPTION_H__
#define __APP_EXCEPTION_H__

#include <string>

const int empty_code_error(0);
const std::string default_exception("Exception");

class AppException :public std::exception
{
private:
    std::string msg_err;
    int code_err;
public:
    AppException(const std::string & msg = default_exception, const int & code = empty_code_error);
    AppException(const int & code);
    ~AppException();
    const char* what() const noexcept; // return message of exception
    const int & code() const; // return code of exception
    void print_error() const; // print exception to console
};

#endif
