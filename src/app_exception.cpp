#include <iostream> // use for cout


#include "app_exception.h"

AppException::AppException(const std::string & msg, const int & code) : msg_err(msg), code_err(code) {}
AppException::AppException(const int & code) : msg_err(default_exception), code_err(code) {}
AppException::~AppException() {}
const char* AppException::what() const noexcept
    {
        return msg_err.c_str();
    }
    const int & AppException::code() const
    {
        return code_err;
    }
    void AppException::print_error() const
    {
        std::cout << msg_err << std::endl;
        if (code_err != empty_code_error)
        {
            std::cout << "Error code: " << code_err << std::endl;
        }
    }

